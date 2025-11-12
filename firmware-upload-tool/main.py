"""Firmware uploader using the project's SUP (Simple UART Protocol).

This script implements a small firmware upload client that sends a
CMD_FW_UPDATE frame followed by DATA frames containing the firmware
binary. It uses the local `sup` module (same protocol as the MCU) to
build and parse frames.

Usage:
    python main.py --port COM9 --baud 57600 --file blinky.bin --trace

The script expects the bootloader on the MCU to reply with ACK frames
for command and each data chunk. It will retry frames on timeout up to
`MAX_RETRIES` times and prints transfer progress.
"""

from __future__ import annotations

import argparse
import sys
import time
import logging
from typing import Optional

try:
    import serial
except Exception:  # pragma: no cover - runtime environment may not have pyserial
    serial = None

import sup


MAX_RETRIES = 3
ACK_TIMEOUT = 2.0  # seconds to wait for ACK/NACK for each frame


def open_serial(port: str, baud: int, timeout: float = 0.01):
    if serial is None:
        raise RuntimeError("pyserial is required. Install with: pip install pyserial")
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def read_frame_with_parser(
    ser, parser: sup.SupParser, timeout: float
) -> Optional[sup.SupFrame]:
    """Read bytes from serial and feed parser until a frame is returned or timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if b:
            logging.debug("RX raw byte: %02x", b[0])
            frame = parser.handle_rx_byte(b[0])
            if frame:
                # Log full parsed frame including numeric id and payload size
                try:
                    rx_id_val = frame.frame_id.value
                    rx_id_name = frame.frame_id.name
                except Exception:
                    rx_id_val = getattr(frame.frame_id, "value", 0)
                    rx_id_name = str(frame.frame_id)
                logging.debug(
                    "RX parsed: id=%s (0x%02x) payload_size=%d payload=%s",
                    rx_id_name,
                    rx_id_val,
                    len(frame.payload),
                    frame.payload.hex(" "),
                )
                return frame
        else:
            # no data, small sleep to avoid busy loop
            time.sleep(0.001)
    return None


def send_frame_and_wait_ack(
    ser,
    frame_bytes: bytes,
    expected_ack_for: Optional[sup.SupId],
    parser: sup.SupParser,
) -> bool:
    """Send a SUP frame and wait for an ACK (optionally checking payload).

    Returns True on ACK, False on NACK or timeout.
    """
    for attempt in range(1, MAX_RETRIES + 1):
        # Log raw frame being sent
        logging.debug("TX frame (%d bytes): %s", len(frame_bytes), frame_bytes.hex(" "))
        # If it's a well-formed SUP frame, additionally log parsed fields (id/payload)
        try:
            if (
                len(frame_bytes) >= 6
                and frame_bytes[0] == sup.SOF
                and frame_bytes[-1] == sup.EOF
            ):
                tx_id = frame_bytes[1]
                tx_size = frame_bytes[2]
                tx_payload = frame_bytes[3 : 3 + tx_size]
                try:
                    tx_name = sup.SupId(tx_id).name
                except Exception:
                    tx_name = f"0x{tx_id:02x}"
                logging.debug(
                    "TX parsed: id=%s (0x%02x) payload_size=%d payload=%s",
                    tx_name,
                    tx_id,
                    tx_size,
                    tx_payload.hex(" "),
                )
        except Exception:
            logging.exception("Failed to parse TX frame for logging")
        ser.write(frame_bytes)
        ser.flush()
        frame = read_frame_with_parser(ser, parser, ACK_TIMEOUT)
        if frame is None:
            print(
                f"No response (timeout) after sending frame, attempt {attempt}/{MAX_RETRIES}"
            )
            continue

        if frame.frame_id == sup.SupId.ACK:
            # ACK may include the original ID in payload (1 byte) or be empty (final ACK)
            if expected_ack_for is None:
                return True
            if len(frame.payload) == 0:
                return True
            if frame.payload[0] == expected_ack_for.value:
                return True
            print(f"Received ACK for different id: {frame.payload}")
            return False

        if frame.frame_id == sup.SupId.NACK:
            print("Received NACK from target")
            return False

        # Unexpected frame, ignore and continue waiting (but break attempts only on timeout)
        print(f"Received unexpected frame: {frame.frame_id} payload={frame.payload}")
    return False


def upload_firmware(port: str, baud: int, firmware: bytes) -> bool:
    total = len(firmware)
    print(f"Opening serial port {port} at {baud} bps...")
    ser = open_serial(port, baud)
    parser = sup.SupParser()

    try:
        # 1) Send CMD_FW_UPDATE frame (no payload)
        cmd_frame = sup.create_frame(sup.SupId.CMD_FW_UPDATE, b"")
        print("Sending CMD_FW_UPDATE")
        if not send_frame_and_wait_ack(ser, cmd_frame, sup.SupId.CMD_FW_UPDATE, parser):
            print("Target did not acknowledge firmware update command")
            return False

        # 2) Send the firmware size in a DATA frame (2 bytes, little-endian)
        size_payload = bytes([total & 0xFF, (total >> 8) & 0xFF])
        size_data_frame = sup.create_data_frame(size_payload)
        print(f"Sending firmware size in DATA frame (size={total})")
        if not send_frame_and_wait_ack(ser, size_data_frame, None, parser):
            # Expect ACK for the DATA frame; we pass None because the ACK payload
            # may be empty or may echo the DATA id depending on implementation.
            print("Target did not acknowledge firmware size DATA frame")
            return False

        # 2) Send data frames in chunks
        chunk_size = sup.MAX_PAYLOAD_SIZE
        sent = 0
        chunk_index = 0
        while sent < total:
            chunk = firmware[sent : sent + chunk_size]
            data_frame = sup.create_data_frame(bytes(chunk))
            # For each data chunk, expect ACK acknowledging SUP_ID_DATA
            ok = send_frame_and_wait_ack(ser, data_frame, sup.SupId.DATA, parser)
            if not ok:
                print(f"Failed to transfer chunk {chunk_index} (offset {sent})")
                return False
            sent += len(chunk)
            chunk_index += 1
            pct = (sent * 100) / total
            print(f"Progress: {sent}/{total} bytes ({pct:.1f}%)")

        # 3) Optionally wait for final ACK with no payload
        final = read_frame_with_parser(ser, parser, ACK_TIMEOUT)
        if final is not None and final.frame_id == sup.SupId.ACK:
            print("Received final ACK from target")
        else:
            print(
                "No final ACK received (this may be acceptable depending on bootloader)."
            )

        return True
    finally:
        ser.close()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="SUP firmware uploader for the AVR bootloader"
    )
    p.add_argument(
        "--port", required=True, help="Serial port (e.g. COM3 or /dev/ttyUSB0)"
    )
    p.add_argument("--baud", type=int, default=57600, help="Baud rate (default: 57600)")
    p.add_argument(
        "--trace",
        action="store_true",
        help="Enable verbose trace logging of frames and bytes",
    )
    p.add_argument(
        "--file",
        required=False,
        help="Path to firmware binary file to upload (e.g. blinky.bin). If omitted, the loader will try to use 'blinky.bin' located next to this script.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    # Configure logging
    logging.basicConfig(
        level=logging.DEBUG if getattr(args, "trace", False) else logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
    )
    try:
        # Determine firmware file path: CLI overrides default next-to-script blinky.bin
        import os

        firmware_path = None
        if getattr(args, "file", None):
            # Try the path the user provided first (may be relative to cwd)
            candidate = args.file
            if os.path.exists(candidate):
                firmware_path = os.path.abspath(candidate)
            else:
                # Fallback: try resolving relative to the script directory
                alt = os.path.join(os.path.dirname(__file__), args.file)
                if os.path.exists(alt):
                    firmware_path = os.path.abspath(alt)
                else:
                    # Also try joining with cwd explicitly for clarity
                    alt2 = os.path.join(os.getcwd(), args.file)
                    if os.path.exists(alt2):
                        firmware_path = os.path.abspath(alt2)

        if firmware_path is None:
            # If user didn't supply file or lookups above failed, try default blinky.bin next to script
            default_path = os.path.join(os.path.dirname(__file__), "blinky.bin")
            if os.path.exists(default_path):
                firmware_path = os.path.abspath(default_path)

        if firmware_path is None:
            print(
                "No firmware file provided and default 'blinky.bin' not found next to loader.py. Use --file <path>."
            )
            sys.exit(4)

        if getattr(args, "trace", False):
            logging.debug("Using firmware file: %s", firmware_path)

        # Load firmware from determined path
        try:
            with open(firmware_path, "rb") as f:
                firmware_bytes = f.read()
        except Exception as e:
            print(f"Failed to read firmware file '{firmware_path}': {e}")
            sys.exit(4)

        ok = upload_firmware(args.port, args.baud, firmware_bytes)
        if ok:
            print("Firmware upload completed successfully.")
            sys.exit(0)
        else:
            print("Firmware upload failed.")
            sys.exit(2)
    except KeyboardInterrupt:
        print("Interrupted by user")
        sys.exit(1)
    except Exception as exc:
        print(f"Error: {exc}")
        sys.exit(3)


if __name__ == "__main__":
    main()
