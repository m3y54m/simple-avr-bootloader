"""
Demonstration of the SUP (Simple UART Protocol) Python module.

This script simulates sending and receiving SUP frames to showcase the
functionality of the `sup` module. It uses an in-memory buffer (`io.BytesIO`)
to act as a virtual serial port.
"""

import io
import sup

def main():
    """Runs the demonstration."""
    print("--- SUP Python Module Demonstration ---")

    # Use an in-memory buffer to simulate a serial port
    virtual_serial_port = io.BytesIO()

    # 1. Create a DATA frame
    print("\n1. Creating a DATA frame with payload 'Hello SUP!'...")
    payload_to_send = b"Hello SUP!"
    data_frame_bytes = sup.create_data_frame(payload_to_send)
    print(f"   - Generated frame (hex): {data_frame_bytes.hex(' ')}")
    
    # "Send" the frame by writing it to our virtual port
    virtual_serial_port.write(data_frame_bytes)
    print("   - Frame has been 'sent' to the virtual serial port.")

    # 2. Parse the received data
    print("\n2. Simulating reception and parsing...")
    
    # Move the buffer's cursor to the beginning to read what we wrote
    virtual_serial_port.seek(0)
    
    parser = sup.SupParser()
    received_frame = None

    # Read byte-by-byte, feeding each one to the parser
    while (byte := virtual_serial_port.read(1)):
        print(f"   - RX byte: {byte.hex()} -> feeding to parser...")
        parsed_frame = parser.handle_rx_byte(byte[0])
        if parsed_frame:
            received_frame = parsed_frame
            print("   - Complete frame parsed!")
            break
    
    if received_frame:
        print("\n--- Successfully Parsed Frame ---")
        print(f"   Frame ID: {received_frame.frame_id.name} (0x{received_frame.frame_id.value:02x})")
        print(f"   Payload: {received_frame.payload}")
        print(f"   Payload (decoded): '{received_frame.payload.decode('utf-8')}'")
        print("---------------------------------")
    else:
        print("\n--- Failed to parse frame ---")
        print(f"   Last parser error: {parser.get_last_error()}")

    # 3. Demonstrate creating an ACK frame
    if received_frame and received_frame.frame_id == sup.SupId.DATA:
        print("\n3. Creating an ACK in response...")
        ack_frame = sup.create_ack_frame(received_frame.frame_id)
        print(f"   - Generated ACK frame (hex): {ack_frame.hex(' ')}")

    # 4. Demonstrate error handling (bad checksum)
    print("\n4. Simulating a corrupted frame (bad checksum)...")
    
    # Manually create a frame with a checksum that is wrong by 1
    bad_frame_bytes = bytearray(data_frame_bytes)
    bad_frame_bytes[-2] = (bad_frame_bytes[-2] + 1) & 0xFF # Corrupt the checksum
    
    virtual_serial_port = io.BytesIO(bad_frame_bytes)
    parser_for_error = sup.SupParser()

    while (byte := virtual_serial_port.read(1)):
        print(f"   - RX byte: {byte.hex()} -> feeding to parser...")
        parser_for_error.handle_rx_byte(byte[0])

    print("\n--- Parsing Result for Corrupted Frame ---")
    print(f"   Last parser error: {parser_for_error.get_last_error().name}")
    print("------------------------------------------")


if __name__ == "__main__":
    main()
