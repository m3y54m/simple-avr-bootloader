# For this program you should instal pySerial library
# pip install pyserial

import serial
import os
import time

BINARY_FILE_NAME = "program.bin"
# Get the path of the Python source file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
# Construct the relative path to the binary file
FILE_PATH = os.path.join(CURRENT_DIR, BINARY_FILE_NAME)
FILE_MAX_SIZE = 65535

# Serial port configuration
# NOTE: Change this to the COM port you are using (in Linux should be sth like /dev/tty*)
PORT_NAME = "COM3"
BAUD_RATE = 9600

# Start-Of-Frame Byte
FRAME_SOF = b"\xAA"
# End-Of-Frame Byte
FRAME_EOF = b"\xBB"
# Message IDs
MSG_ID_FILE_SIZE = 1
MSG_ID_FILE_CHUNK = 2


def calculate_checksum(payload_size, message_id, payload_data):
    checksum = payload_size
    checksum ^= message_id
    for byte in payload_data:
        checksum ^= byte
    return checksum


def send_frame(serial_port_handle, message_id, payload_data):
    """
    **************************************************************************************
    *                                 UART Frame Format                                  *
    **************************************************************************************

        0           1              2             3 .. (N + 2)          (N + 3)   (N + 4)
    +--------+--------------+--------------+-------------------------+----------+--------+
    |  SOF   | Payload Size |  Message-ID  | Payload (Variable size) | Checksum |   EOF  |
    +--------+--------------+--------------+-------------------------+----------+--------+
    |  0xAA  |      N       |  Message-ID  |      Payload Bytes      | Checksum |  0xBB  |
    +--------+--------------+--------------+-------------------------+----------+--------+

    Payload bytes are in Little-Endian format

    *************************************************************************************
    """
    payload_size = len(payload_data)
    checksum = calculate_checksum(payload_size, message_id, payload_data)
    frame = (
        FRAME_SOF
        + bytes([payload_size])
        + bytes([message_id])
        + payload_data
        + bytes([checksum])
        + FRAME_EOF
    )

    # Send the frame through serial port
    serial_port_handle.write(frame)


def send_file_data(serial_port_handle, FILE_PATH):
    try:
        with open(FILE_PATH, "rb") as file:     
                 
            while True:
                # Read 128 bytes from the file
                chunk = file.read(128)
                
                # Break the loop if no more data is read
                if not chunk:
                    break
                
                # time.sleep(0.1)
                # Send the chunk through the serial connection
                send_frame(serial_port_handle, MSG_ID_FILE_CHUNK, chunk)
                
                # Wait for transmission to complete
                ser.flush()
                
            print("File sent successfully.")
    except IOError:
        print("Error opening file.")


def int_to_bytes(number):
    if (number > FILE_MAX_SIZE or number < 0):
        raise ValueError("File size must be between 0 and 65535. Program stopped.")
    else:
        # Convert the number to two bytes
        byte_1 = (number >> 8) & 0xFF
        byte_0 = number & 0xFF
        # Send LSB (byte_0) first (Little-Endian)
        return bytes([byte_0, byte_1])


def send_file_size(serial_port_handle, FILE_PATH):
    try:
        # Get the size of the binary file
        file_size_in_bytes = os.path.getsize(FILE_PATH)
        send_frame(
            serial_port_handle, MSG_ID_FILE_SIZE, int_to_bytes(file_size_in_bytes)
        )
        print(f"The size of the binary file is {file_size_in_bytes} bytes.")
    except IOError:
        print("Error opening file.")


# Open the serial port
with serial.Serial(
    PORT_NAME, BAUD_RATE, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE
) as ser:
    
    # If you are using Atmega328 on an Arduino board, starting serial port connection
    # causes DTR line to get the MCU to reset!
    # If you want to disable Arduino auto reset read this article

    # External reset is required to enter the firmware update mode of the bootloader

    # Send the binary file size through serial port
    send_file_size(ser, FILE_PATH)

    # Send the binary file data through serial port
    send_file_data(ser, FILE_PATH)
