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

/* ---- Center antenna stream reader ---- */
static const uint8_t *center_rx_buf;
static volatile uint16_t write_pos;
static uint16_t read_pos;

/* COBS frame accumulator */
static uint8_t frame_buf[COBS_MAX_FRAME];
static uint16_t frame_pos;
static uint8_t decode_buf[COBS_MAX_FRAME];

/* ---- Ground station (radians, meters) ---- */
static float ground_lat_rad, ground_lon_rad, ground_alt;

/* ---- Tracking state ---- */
static float initial_az, initial_el;
static float current_az, current_el;
static uint8_t has_initial_fix;
static uint8_t has_rocket_fix;

/* ---- Velocity estimation ---- */
static float prev_alt;
static float prev_az_for_vel;
static uint32_t prev_gps_tick;
static float v_up;          /* smoothed vertical velocity (m/s) */
static float horiz_dist;    /* horizontal distance to rocket (m) */
static float vel_az;         /* smoothed azimuth angular rate (rad/s) */
static uint8_t has_velocity;

/* ---- Control loop ---- */
static uint32_t last_ctrl_tick;

/* ================================================================
 * COBS decoder
 * ================================================================ */

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

/* ================================================================
 * Geodetic math
 * ================================================================ */

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

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ================================================================
 * GPS fix handling + velocity estimation
 * ================================================================ */

static void update_gps_fix(float lat_deg, float lon_deg, float alt_m)
{
    float lat_rad = lat_deg * DEG_TO_RAD;
    float lon_rad = lon_deg * DEG_TO_RAD;

    float az   = calc_bearing(ground_lat_rad, ground_lon_rad, lat_rad, lon_rad);
    float dist = calc_horiz_dist(ground_lat_rad, ground_lon_rad, lat_rad, lon_rad);
    float el   = atan2f(alt_m - ground_alt, dist);

    /* Record initial pointing direction */
    if (!has_initial_fix) {
        initial_az = az;
        initial_el = el;
        has_initial_fix = 1;
    }

    /* Velocity estimation (needs ≥2 fixes) */
    uint32_t now = HAL_GetTick();
    if (has_rocket_fix) {
        float dt = (now - prev_gps_tick) / 1000.0f;
        if (dt > 0.01f) {
            /* Vertical velocity — the primary signal for related rates */
            float raw_v_up = (alt_m - prev_alt) / dt;
            v_up = TRACKER_VEL_ALPHA * raw_v_up +
                   (1.0f - TRACKER_VEL_ALPHA) * v_up;

            /* Azimuth angular velocity — captures wind drift */
            float raw_vel_az = angle_wrap(az - prev_az_for_vel) / dt;
            raw_vel_az = clampf(raw_vel_az, -TRACKER_VEL_MAX, TRACKER_VEL_MAX);
            vel_az = TRACKER_VEL_ALPHA * raw_vel_az +
                     (1.0f - TRACKER_VEL_ALPHA) * vel_az;

            has_velocity = 1;
        }
    }

    /* Snap position to GPS truth */
    current_az = az;
    current_el = el;
    horiz_dist = dist;
    has_rocket_fix = 1;

    /* Store for next velocity diff */
    prev_alt = alt_m;
    prev_az_for_vel = az;
    prev_gps_tick = now;
}

/* ================================================================
 * Protobuf GPS parsing — TODO
 * ================================================================ */

/* TODO: implement when protobuf schema is available.
 * Decode the protobuf payload and extract GPS fields.
 * Returns 1 if a GPS message was found, fills out lat/lon/alt. */
static int parse_gps_protobuf(const uint8_t *data, size_t len,
                               float *lat, float *lon, float *alt)
{
    (void)data; (void)len;
    (void)lat; (void)lon; (void)alt;
    return 0;
}

/* ================================================================
 * COBS frame processing
 * ================================================================ */

static void process_frame(void)
{
    if (frame_pos < 2) {
        frame_pos = 0;
        return;
    }

    int len = cobs_decode(frame_buf, frame_pos, decode_buf, sizeof(decode_buf));
    frame_pos = 0;
    if (len < 0)
        return;

    float lat, lon, alt;
    if (parse_gps_protobuf(decode_buf, (size_t)len, &lat, &lon, &alt))
        update_gps_fix(lat, lon, alt);
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

/* ================================================================
 * Fused tracking controller
 * GPS feedforward (related rates) + RSSI feedback (proportional)
 * ================================================================ */

static void tracking_update(void)
{
    uint32_t now = HAL_GetTick();
    float dt_since_fix = (now - prev_gps_tick) / 1000.0f;

    /* ── 1. Feedforward: predict angle from velocity ── */
    float pred_az, pred_el;
    float omega_az = 0.0f, omega_el = 0.0f;

    if (has_velocity) {
        /* Related rates: elevation angular velocity from vertical speed
         * dθ/dt = v_up · cos²(θ) / d */
        float cos_el = cosf(current_el);
        if (horiz_dist > 1.0f)
            omega_el = v_up * cos_el * cos_el / horiz_dist;

        omega_az = vel_az;

        pred_az = current_az + omega_az * dt_since_fix;
        pred_el = current_el + omega_el * dt_since_fix;
    } else {
        pred_az = current_az;
        pred_el = current_el;
    }

    /* ── 2. Feedback: RSSI proportional correction ── */
    float rssi_corr_az = 0.0f;
    float rssi_corr_el = 0.0f;

    const RssiReading *r1 = RSSI_GetReading(RSSI_ANT_OUTER1);
    const RssiReading *r2 = RSSI_GetReading(RSSI_ANT_OUTER2);
    if (r1->valid && r2->valid &&
        (now - r1->timestamp) < TRACKER_RSSI_TIMEOUT &&
        (now - r2->timestamp) < TRACKER_RSSI_TIMEOUT) {
        rssi_corr_az = TRACKER_K_RSSI *
                       ((int16_t)r1->left - (int16_t)r2->left);
    }

    const RssiReading *r3 = RSSI_GetReading(RSSI_ANT_OUTER3);
    const RssiReading *r4 = RSSI_GetReading(RSSI_ANT_OUTER4);
    if (r3->valid && r4->valid &&
        (now - r3->timestamp) < TRACKER_RSSI_TIMEOUT &&
        (now - r4->timestamp) < TRACKER_RSSI_TIMEOUT) {
        rssi_corr_el = TRACKER_K_RSSI *
                       ((int16_t)r3->left - (int16_t)r4->left);
    }

    /* ── 3. Combined target ── */
    float target_az = pred_az + rssi_corr_az;
    float target_el = pred_el + rssi_corr_el;

    float daz = angle_wrap(target_az - initial_az);
    float del = target_el - initial_el;

    Stepper_SetTarget(STEPPER_AZ,
                      (int32_t)(daz * RAD_TO_DEG * TRACKER_STEPS_PER_DEG));
    Stepper_SetTarget(STEPPER_EL,
                      (int32_t)(del * RAD_TO_DEG * TRACKER_STEPS_PER_DEG));

    /* ── 4. Match stepper speed to angular velocity ── */
    float corr_rate = 1000.0f / TRACKER_CTRL_INTERVAL_MS;
    float total_omega_az = fabsf(omega_az) + fabsf(rssi_corr_az) * corr_rate;
    float total_omega_el = fabsf(omega_el) + fabsf(rssi_corr_el) * corr_rate;

    float speed_az = total_omega_az * RAD_TO_DEG * TRACKER_STEPS_PER_DEG;
    float speed_el = total_omega_el * RAD_TO_DEG * TRACKER_STEPS_PER_DEG;

    Stepper_SetSpeed(STEPPER_AZ,
                     speed_az > 1.0f ? (uint32_t)(1000.0f / speed_az) : 2);
    Stepper_SetSpeed(STEPPER_EL,
                     speed_el > 1.0f ? (uint32_t)(1000.0f / speed_el) : 2);
}

/* ================================================================
 * Public API
 * ================================================================ */

void Tracker_Init(float ground_lat, float ground_lon, float ground_alt_m)
{
    center_rx_buf = Passthrough_GetCenterRxBuf();
    write_pos = 0;
    read_pos  = 0;
    frame_pos = 0;

    has_initial_fix = 0;
    has_rocket_fix  = 0;
    has_velocity    = 0;
    last_ctrl_tick  = 0;

    v_up     = 0.0f;
    vel_az   = 0.0f;
    prev_alt = 0.0f;
    prev_az_for_vel = 0.0f;
    prev_gps_tick   = 0;
    horiz_dist      = 0.0f;

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

    /* Run fused control loop at fixed rate */
    uint32_t now = HAL_GetTick();
    if (has_rocket_fix &&
        (now - last_ctrl_tick) >= TRACKER_CTRL_INTERVAL_MS) {
        last_ctrl_tick = now;
        tracking_update();
    }
}

void Tracker_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
        write_pos = Size % PT_BUF_SIZE;
}
