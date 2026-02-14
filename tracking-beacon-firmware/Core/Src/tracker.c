#include "tracker.h"
#include "passthrough.h"
#include "rssi.h"
#include "stepper.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define EARTH_RADIUS 6371000.0f
#define COBS_MAX_FRAME 256

extern UART_HandleTypeDef huart1;

/* Center antenna stream reader (shares passthrough rx buffer) */
static const uint8_t *center_rx_buf;
static volatile uint16_t write_pos;
static uint16_t read_pos;

/* COBS frame accumulator */
static uint8_t frame_buf[COBS_MAX_FRAME];
static uint16_t frame_pos;
static uint8_t decode_buf[COBS_MAX_FRAME];

/* Ground station position (radians, meters) */
static float ground_lat_rad, ground_lon_rad, ground_alt;

/* Tracking state */
static float initial_az, initial_el;
static float current_az, current_el;
static uint8_t has_initial_fix;
static uint8_t has_rocket_fix;
static uint32_t last_rssi_check;

/* ---- COBS decoder ---- */

static int cobs_decode(const uint8_t *in, size_t in_len,
                       uint8_t *out, size_t out_max)
{
    size_t rp = 0, wp = 0;
    while (rp < in_len) {
        uint8_t code = in[rp++];
        if (code == 0)
            return -1;
        uint8_t len = code - 1;
        if (rp + len > in_len || wp + len > out_max)
            return -1;
        memcpy(&out[wp], &in[rp], len);
        rp += len;
        wp += len;
        if (code < 0xFF && rp < in_len) {
            if (wp >= out_max)
                return -1;
            out[wp++] = 0x00;
        }
    }
    return (int)wp;
}

/* ---- Geodetic math ---- */

static float calc_bearing(float lat1, float lon1, float lat2, float lon2)
{
    float dlon = lon2 - lon1;
    float y = sinf(dlon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) -
              sinf(lat1) * cosf(lat2) * cosf(dlon);
    return atan2f(y, x);
}

static float calc_horiz_dist(float lat1, float lon1, float lat2, float lon2)
{
    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;
    float a = sinf(dlat / 2) * sinf(dlat / 2) +
              cosf(lat1) * cosf(lat2) * sinf(dlon / 2) * sinf(dlon / 2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
    return EARTH_RADIUS * c;
}

static float angle_wrap(float d)
{
    while (d > M_PI)  d -= 2 * M_PI;
    while (d < -M_PI) d += 2 * M_PI;
    return d;
}

/* ---- Packet handling ---- */

static void handle_gps_packet(const GpsPacket *pkt)
{
    float lat_rad = pkt->latitude  * DEG_TO_RAD;
    float lon_rad = pkt->longitude * DEG_TO_RAD;

    float az   = calc_bearing(ground_lat_rad, ground_lon_rad, lat_rad, lon_rad);
    float dist = calc_horiz_dist(ground_lat_rad, ground_lon_rad, lat_rad, lon_rad);
    float el   = atan2f(pkt->altitude - ground_alt, dist);

    if (!has_initial_fix) {
        initial_az = az;
        initial_el = el;
        has_initial_fix = 1;
    }

    current_az = az;
    current_el = el;
    has_rocket_fix = 1;
}

static void process_frame(void)
{
    if (frame_pos < 2) {
        frame_pos = 0;
        return;
    }

    int len = cobs_decode(frame_buf, frame_pos, decode_buf, sizeof(decode_buf));
    frame_pos = 0;

    if (len == (int)sizeof(GpsPacket) && decode_buf[0] == GPS_PACKET_ID)
        handle_gps_packet((const GpsPacket *)decode_buf);
}

static void process_byte(uint8_t byte)
{
    if (byte == 0x00) {
        process_frame();
        return;
    }
    if (frame_pos < COBS_MAX_FRAME)
        frame_buf[frame_pos++] = byte;
    else
        frame_pos = 0;
}

/* ---- Tracking modes ---- */

static uint8_t rssi_available(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < RSSI_NUM_OUTER; i++) {
        const RssiReading *r = RSSI_GetReading(i);
        if (!r->valid || (now - r->timestamp) >= TRACKER_RSSI_TIMEOUT)
            return 0;
    }
    return 1;
}

static void rssi_track(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - last_rssi_check) < TRACKER_RSSI_POLL_MS)
        return;
    last_rssi_check = now;

    /* Azimuth: outer1 (+az) vs outer2 (-az), compare local RSSI */
    const RssiReading *r1 = RSSI_GetReading(RSSI_ANT_OUTER1);
    const RssiReading *r2 = RSSI_GetReading(RSSI_ANT_OUTER2);
    int16_t diff_az = (int16_t)r1->left - (int16_t)r2->left;

    if (diff_az > TRACKER_RSSI_DEADZONE)
        Stepper_SetTarget(STEPPER_AZ, Stepper_GetPosition(STEPPER_AZ) + 1);
    else if (diff_az < -TRACKER_RSSI_DEADZONE)
        Stepper_SetTarget(STEPPER_AZ, Stepper_GetPosition(STEPPER_AZ) - 1);

    /* Elevation: outer3 (+el) vs outer4 (-el) */
    const RssiReading *r3 = RSSI_GetReading(RSSI_ANT_OUTER3);
    const RssiReading *r4 = RSSI_GetReading(RSSI_ANT_OUTER4);
    int16_t diff_el = (int16_t)r3->left - (int16_t)r4->left;

    if (diff_el > TRACKER_RSSI_DEADZONE)
        Stepper_SetTarget(STEPPER_EL, Stepper_GetPosition(STEPPER_EL) + 1);
    else if (diff_el < -TRACKER_RSSI_DEADZONE)
        Stepper_SetTarget(STEPPER_EL, Stepper_GetPosition(STEPPER_EL) - 1);
}

static void gps_track(void)
{
    if (!has_rocket_fix || !has_initial_fix)
        return;

    float daz = angle_wrap(current_az - initial_az);
    float del = current_el - initial_el;

    Stepper_SetTarget(STEPPER_AZ,
                      (int32_t)(daz * RAD_TO_DEG * TRACKER_STEPS_PER_DEG));
    Stepper_SetTarget(STEPPER_EL,
                      (int32_t)(del * RAD_TO_DEG * TRACKER_STEPS_PER_DEG));
}

/* ---- Public API ---- */

void Tracker_Init(float ground_lat, float ground_lon, float ground_alt_m)
{
    center_rx_buf = Passthrough_GetCenterRxBuf();
    write_pos = 0;
    read_pos  = 0;
    frame_pos = 0;

    has_initial_fix = 0;
    has_rocket_fix  = 0;
    last_rssi_check = 0;

    ground_lat_rad = ground_lat * DEG_TO_RAD;
    ground_lon_rad = ground_lon * DEG_TO_RAD;
    ground_alt     = ground_alt_m;
}

void Tracker_Poll(void)
{
    /* Drain new bytes from center antenna */
    uint16_t wp = write_pos;
    while (read_pos != wp) {
        process_byte(center_rx_buf[read_pos]);
        read_pos++;
        if (read_pos >= PT_BUF_SIZE)
            read_pos = 0;
    }

    /* RSSI primary, GPS fallback */
    if (rssi_available())
        rssi_track();
    else
        gps_track();
}

void Tracker_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
        write_pos = Size % PT_BUF_SIZE;
}
