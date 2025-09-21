import serial
import time

# SUP Protocol Constants
SUP_SOF = 0xA1
SUP_EOF = 0xE9
SUP_MAX_PAYLOAD_SIZE = 128

# SUP Frame IDs
SUP_ID_ACK = 0x01  # Acknowledgment for received frame
SUP_ID_NACK = 0x02  # Negative Acknowledgment for received frame
SUP_ID_DATA = 0x03  # Binary data chunk transfer
SUP_ID_CMD_FW_UPDATE = 0x11  # Firmware Update Command


# SUP Protocol Functions
def calculate_checksum(payload_size, id, payload):
    """Calculates the checksum for a SUP frame."""
    checksum = payload_size + id
    for byte in payload:
        checksum += byte
    return checksum & 0xFF  # Ensure checksum is a single byte


def create_sup_frame(id, payload=b""):
    """Creates a complete SUP frame as a bytearray."""
    payload_size = len(payload)
    if payload_size > SUP_MAX_PAYLOAD_SIZE:
        raise ValueError(
            f"Payload size ({payload_size}) exceeds max limit ({SUP_MAX_PAYLOAD_SIZE})"
        )

    checksum = calculate_checksum(payload_size, id, payload)

    frame = bytearray()
    frame.append(SUP_SOF)
    frame.append(id)
    frame.append(payload_size)
    frame.extend(payload)
    frame.append(checksum)
    frame.append(SUP_EOF)

    return frame


def parse_sup_frames(buffer: bytearray):
    """Parse a byte buffer and return a list of full SUP frames (each as bytearray).

    A SUP frame has the structure:
      SOF(1) ID(1) SIZE(1) PAYLOAD(SIZE) CHECKSUM(1) EOF(1)

    This function extracts complete frames and skips incomplete or corrupted segments.
    """
    frames = []
    i = 0
    buf_len = len(buffer)
    while i < buf_len:
        # find next SOF
        if buffer[i] != SUP_SOF:
            i += 1
            continue

        # need at least SOF + ID + SIZE + CHECKSUM + EOF => 5 bytes (with 0 payload)
        if i + 5 > buf_len:
            # incomplete header/short tail -- wait for more bytes
            break

        # read size
        payload_size = buffer[i + 2]
        total_len = payload_size + 5  # SOF, ID, SIZE, PAYLOAD, CHK, EOF => payload+5

        # check if full frame is present
        if i + total_len > buf_len:
            # incomplete frame
            break

        # check EOF marker
        if buffer[i + total_len - 1] != SUP_EOF:
            # malformed frame: skip this SOF and continue searching
            i += 1
            continue

        # extract full frame
        frame = bytearray(buffer[i : i + total_len])
        frames.append(frame)

        # advance index past this frame
        i += total_len

    return frames


def main():
    """Main function to demonstrate communication with the AVR."""
    # Configure the serial port
    ser = serial.Serial(
        port="COM9",  # Change to your COM port (e.g., '/dev/ttyACM0' on Linux)
        baudrate=57600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1,
    )

    print(f"Connected to {ser.name}")

    try:
        # Example 1: Send a simple message to the AVR
        demo_id = SUP_ID_DATA
        demo_payload = bytearray([0x44, 0x55, 0x66, 0x77])

        tx_frame = create_sup_frame(demo_id, demo_payload)
        print(f"Sending frame:\r\n  {[f'{b:02X}' for b in tx_frame]}")
        ser.write(tx_frame)

        # Wait for a response from the AVR
        time.sleep(0.5)

        rx_buffer = bytearray()
        while ser.in_waiting > 0:
            rx_buffer.extend(ser.read(ser.in_waiting))
            time.sleep(0.1)  # Give some time for more data to arrive

        if rx_buffer:
            # split into SUP frames (there may be multiple concatenated frames)
            parsed = parse_sup_frames(rx_buffer)
            if parsed:
                print("Received frames:")
                for f in parsed:
                    print(f"  {[f'{b:02X}' for b in f]}")
            else:
                # fallback: print raw buffer if no valid frames found
                print(f"Received bytes (raw): {[f'{b:02X}' for b in rx_buffer]}")
        else:
            print("No response from AVR.")

    except serial.SerialException as e:
        print(f"Serial communication error: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")


if __name__ == "__main__":
    main()
