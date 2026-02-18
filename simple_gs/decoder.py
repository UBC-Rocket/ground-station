#!/usr/bin/env python3
"""
Serial decoder for COBS-encoded, CRC-protected protobuf packets.

Packet format (before COBS encoding):
    [protobuf_data][crc32_le (4 bytes)]

COBS framing:
    - Packets are COBS encoded
    - 0x00 byte is used as packet delimiter
"""

import argparse
import csv
import sys
from datetime import datetime
from cobs import cobs
import crcmod
import serial
from google.protobuf.message import DecodeError

# Import the generated protobuf modules
from simple_gs.TelemetryPacket_pb2 import TelemetryPacket, FlightState


# CRC-16-CCITT Kermit variant (polynomial 0x1021, init 0x0000, reflected)
crc16_func = crcmod.predefined.mkCrcFun('kermit')


def hexdump(data: bytes, prefix: str = "  ") -> str:
    """Format bytes as a hex dump with ASCII representation."""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_part = " ".join(f"{b:02x}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{prefix}{i:04x}: {hex_part:<48} |{ascii_part}|")
    return "\n".join(lines)


def decode_packet(raw_data: bytes, debug: bool = False) -> TelemetryPacket | None:
    """
    Decode a COBS-encoded, CRC-protected protobuf packet.

    Args:
        raw_data: COBS-encoded data (without the trailing 0x00 delimiter)
        debug: If True, print detailed debug information

    Returns:
        Decoded TelemetryPacket or None if decoding failed
    """
    if debug:
        print(f"[DEBUG] Raw COBS data ({len(raw_data)} bytes):")
        print(hexdump(raw_data))

    # Step 1: COBS decode
    try:
        decoded = cobs.decode(raw_data)
    except cobs.DecodeError as e:
        print(f"[ERROR] COBS decode failed: {e}", file=sys.stderr)
        print(f"[DEBUG] Raw bytes: {raw_data.hex()}", file=sys.stderr)
        return None

    if debug:
        print(f"[DEBUG] After COBS decode ({len(decoded)} bytes):")
        print(hexdump(decoded))

    if len(decoded) < 2:
        print(f"[ERROR] Packet too short ({len(decoded)} bytes), need at least 2 for CRC", file=sys.stderr)
        print(f"[DEBUG] Decoded bytes: {decoded.hex()}", file=sys.stderr)
        return None

    # Step 2: Extract payload and CRC (CRC-16 is last 2 bytes, little-endian)
    payload = decoded[:-2]
    crc_bytes = decoded[-2:]
    received_crc = int.from_bytes(crc_bytes, byteorder='little')

    if debug:
        print(f"[DEBUG] Payload ({len(payload)} bytes):")
        print(hexdump(payload))
        print(f"[DEBUG] CRC-16 bytes: {crc_bytes.hex()} -> 0x{received_crc:04X}")

    # Step 3: Verify CRC-16
    computed_crc = crc16_func(payload)
    if computed_crc != received_crc:
        print(f"[WARNING] CRC mismatch! Received: 0x{received_crc:04X}, Computed: 0x{computed_crc:04X}", file=sys.stderr)
        # Continue anyway to show what we got

    # Step 4: Decode protobuf
    try:
        packet = TelemetryPacket()
        packet.ParseFromString(payload)
        return packet
    except DecodeError as e:
        print(f"[ERROR] Protobuf decode failed: {e}", file=sys.stderr)
        print(f"[DEBUG] Payload bytes that failed to decode:", file=sys.stderr)
        print(hexdump(payload, prefix="  "), file=sys.stderr)
        return None


def flight_state_name(state: FlightState) -> str:
    """Convert FlightState enum to human-readable string."""
    names = {
        FlightState.STANDBY: "STANDBY",
        FlightState.ASCENT: "ASCENT",
        FlightState.MACH_LOCK: "MACH_LOCK",
        FlightState.DROGUE_DESCENT: "DROGUE_DESCENT",
        FlightState.MAIN_DESCENT: "MAIN_DESCENT",
        FlightState.LANDED: "LANDED",
    }
    return names.get(state, f"UNKNOWN({state})")


# CSV column definitions
CSV_COLUMNS = [
    "recv_time",
    "counter",
    "timestamp_ms",
    "state",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "kf_altitude",
    "kf_velocity",
    "baro0_healthy",
    "baro1_healthy",
    "ground_altitude",
    "gps_latitude",
    "gps_longitude",
    "gps_altitude",
    "gps_speed",
    "gps_sats",
    "gps_fix",
]


def packet_to_csv_row(packet: TelemetryPacket) -> list:
    """Convert a TelemetryPacket to a CSV row."""
    return [
        datetime.now().isoformat(timespec='milliseconds'),
        packet.counter,
        packet.timestamp_ms,
        flight_state_name(packet.state),
        packet.accel_x,
        packet.accel_y,
        packet.accel_z,
        packet.gyro_x,
        packet.gyro_y,
        packet.gyro_z,
        packet.kf_altitude,
        packet.kf_velocity,
        packet.baro0_healthy,
        packet.baro1_healthy,
        packet.ground_altitude,
        packet.gps_latitude,
        packet.gps_longitude,
        packet.gps_altitude,
        packet.gps_speed,
        packet.gps_sats,
        packet.gps_fix,
    ]


def read_cobs_packet(ser: serial.Serial) -> bytes | None:
    """
    Read a complete COBS packet from serial (delimited by 0x00).

    Returns:
        The raw COBS-encoded data (without delimiter), or None on timeout/error
    """
    buffer = bytearray()

    while True:
        byte = ser.read(1)
        if not byte:
            # Timeout
            if buffer:
                print(f"[WARNING] Timeout with {len(buffer)} bytes in buffer", file=sys.stderr)
            return None

        if byte[0] == 0x00:
            # End of packet
            if buffer:
                return bytes(buffer)
            # Empty packet, keep reading
            continue

        buffer.append(byte[0])

        # Sanity check - COBS packets shouldn't be huge
        if len(buffer) > 4096:
            print("[ERROR] Buffer overflow, discarding", file=sys.stderr)
            buffer.clear()


def main():
    parser = argparse.ArgumentParser(
        description="Decode COBS/CRC/Protobuf packets from serial port"
    )
    parser.add_argument(
        "port",
        help="Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0, COM3)"
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=57600,
        help="Baud rate (default: 57600)"
    )
    parser.add_argument(
        "-t", "--timeout",
        type=float,
        default=1.0,
        help="Read timeout in seconds (default: 1.0)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show all fields (default shows compact one-line output)"
    )
    parser.add_argument(
        "-d", "--debug",
        action="store_true",
        help="Show detailed debug output (hex dumps at each decode stage)"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        metavar="FILE",
        help="Log telemetry data to CSV file"
    )

    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")

    # Set up CSV logging if requested
    csv_file = None
    csv_writer = None
    if args.output:
        csv_file = open(args.output, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(CSV_COLUMNS)
        print(f"Logging to {args.output}")

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
            print(f"Connected. Listening for packets...\n")

            packet_count = 0
            while True:
                raw_data = read_cobs_packet(ser)
                if raw_data is None:
                    continue

                packet_count += 1

                if args.verbose:
                    print(f"[{packet_count}] Raw COBS ({len(raw_data)} bytes): {raw_data.hex()}")

                packet = decode_packet(raw_data, debug=args.debug)
                if packet:
                    # Log to CSV if enabled
                    if csv_writer:
                        csv_writer.writerow(packet_to_csv_row(packet))
                        csv_file.flush()  # Ensure data is written immediately

                    if args.verbose:
                        # Full output - all fields
                        print(f"[{packet_count}] TelemetryPacket:")
                        print(f"    counter:         {packet.counter}")
                        print(f"    timestamp_ms:    {packet.timestamp_ms}")
                        print(f"    state:           {flight_state_name(packet.state)}")
                        print(f"    accel_x:         {packet.accel_x:.4f}")
                        print(f"    accel_y:         {packet.accel_y:.4f}")
                        print(f"    accel_z:         {packet.accel_z:.4f}")
                        print(f"    gyro_x:          {packet.gyro_x:.4f}")
                        print(f"    gyro_y:          {packet.gyro_y:.4f}")
                        print(f"    gyro_z:          {packet.gyro_z:.4f}")
                        print(f"    kf_altitude:     {packet.kf_altitude:.4f}")
                        print(f"    kf_velocity:     {packet.kf_velocity:.4f}")
                        print(f"    baro0_healthy:   {packet.baro0_healthy}")
                        print(f"    baro1_healthy:   {packet.baro1_healthy}")
                        print(f"    ground_altitude: {packet.ground_altitude:.4f}")
                        print(f"    gps_latitude:    {packet.gps_latitude:.6f}")
                        print(f"    gps_longitude:   {packet.gps_longitude:.6f}")
                        print(f"    gps_altitude:    {packet.gps_altitude:.2f}")
                        print(f"    gps_speed:       {packet.gps_speed:.2f}")
                        print(f"    gps_sats:        {packet.gps_sats}")
                        print(f"    gps_fix:         {packet.gps_fix}")
                        print()
                    else:
                        # Compact one-line output
                        baro_status = []
                        if packet.baro0_healthy:
                            baro_status.append("B0:OK")
                        else:
                            baro_status.append("B0:FAIL")
                        if packet.baro1_healthy:
                            baro_status.append("B1:OK")
                        else:
                            baro_status.append("B1:FAIL")

                        gps_info = f"GPS: {packet.gps_latitude:.5f},{packet.gps_longitude:.5f} ({packet.gps_sats}sats)"

                        print(f"[{packet_count}] t={packet.timestamp_ms:>8}ms | "
                              f"{flight_state_name(packet.state):14} | "
                              f"Alt: {packet.kf_altitude:>8.2f}m | "
                              f"Vel: {packet.kf_velocity:>7.2f}m/s | "
                              f"{gps_info} | "
                              f"{' '.join(baro_status)}")

    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if csv_file:
            csv_file.close()
            print(f"CSV saved to {args.output}")


if __name__ == "__main__":
    main()
