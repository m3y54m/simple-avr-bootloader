"""
Simple UART Protocol (SUP) - A Python Implementation

This module provides a Python implementation of the Simple UART Protocol (SUP)
for creating and parsing data frames. It is a translation of the original C
library.

Protocol Frame Format:
+-----+----+------+-------------+----------+-----+
| SOF | ID | SIZE |   PAYLOAD   | CHECKSUM | EOF |
+-----+----+------+-------------+----------+-----+
| 1B  | 1B |  1B  | 0..N bytes  |    1B    | 1B  |
+-----+----+------+-------------+----------+-----+

- SOF: Start-Of-Frame (0xA1)
- ID: Frame identifier
- SIZE: Number of payload bytes
- PAYLOAD: Variable-length data
- CHECKSUM: 8-bit sum (modulo 256) of ID, SIZE, and PAYLOAD
- EOF: End-Of-Frame (0xE9)
"""

import enum
from typing import Optional, NamedTuple

# --- Protocol Constants ---
SOF = 0xA1
EOF = 0xE9
MAX_PAYLOAD_SIZE = 128


class SupId(enum.IntEnum):
    """SUP Frame Identifiers"""
    ACK = 0x01
    NACK = 0x02
    DATA = 0x03
    CMD_FW_UPDATE = 0x11


class SupParsingResult(enum.Enum):
    """Result of a frame parsing attempt"""
    PENDING = 0
    SUCCESS = 1
    ERROR_SIZE = 2
    ERROR_CHECKSUM = 3
    ERROR_EOF = 4


class SupParsingState(enum.Enum):
    """Internal states for the frame parsing state machine"""
    WAIT_SOF = 0
    WAIT_ID = 1
    WAIT_PAYLOAD_SIZE = 2
    GET_PAYLOAD = 3
    WAIT_CHECKSUM = 4
    WAIT_EOF = 5


class SupFrame(NamedTuple):
    """Represents a successfully parsed SUP frame."""
    frame_id: SupId
    payload: bytes


class SupParser:
    """
    Parses incoming bytes according to the SUP protocol using a state machine.

    This class processes one byte at a time and tracks the state of frame
    reception. When a full, valid frame is received, it is returned.
    """

    def __init__(self):
        self._last_error: Optional[SupParsingResult] = None
        self._reset_state()

    def _reset_state(self) -> None:
        """Resets the parser state to its initial condition."""
        self._state = SupParsingState.WAIT_SOF
        self._frame_id: int = 0
        self._payload_size: int = 0
        self._payload: bytearray = bytearray()
        self._payload_index: int = 0
        self._calculated_checksum: int = 0

    def handle_rx_byte(self, byte: int) -> Optional[SupFrame]:
        """
        Processes a single received byte.

        Args:
            byte: The integer value of the byte received.

        Returns:
            A SupFrame object if a complete and valid frame has been parsed,
            otherwise None.
        """
        if self._state == SupParsingState.WAIT_SOF:
            if byte == SOF:
                self._reset_state()
                self._state = SupParsingState.WAIT_ID
        
        elif self._state == SupParsingState.WAIT_ID:
            self._frame_id = byte
            self._calculated_checksum = byte
            self._state = SupParsingState.WAIT_PAYLOAD_SIZE
        
        elif self._state == SupParsingState.WAIT_PAYLOAD_SIZE:
            if byte > MAX_PAYLOAD_SIZE:
                self._last_error = SupParsingResult.ERROR_SIZE
                self._reset_state()
            else:
                self._payload_size = byte
                self._calculated_checksum = (self._calculated_checksum + byte) & 0xFF
                if self._payload_size == 0:
                    self._state = SupParsingState.WAIT_CHECKSUM
                else:
                    self._state = SupParsingState.GET_PAYLOAD
        
        elif self._state == SupParsingState.GET_PAYLOAD:
            self._payload.append(byte)
            self._calculated_checksum = (self._calculated_checksum + byte) & 0xFF
            if len(self._payload) >= self._payload_size:
                self._state = SupParsingState.WAIT_CHECKSUM
        
        elif self._state == SupParsingState.WAIT_CHECKSUM:
            if byte == self._calculated_checksum:
                self._state = SupParsingState.WAIT_EOF
            else:
                self._last_error = SupParsingResult.ERROR_CHECKSUM
                self._reset_state()
        
        elif self._state == SupParsingState.WAIT_EOF:
            self._state = SupParsingState.WAIT_SOF # Reset for next frame
            if byte == EOF:
                self._last_error = SupParsingResult.SUCCESS
                try:
                    frame_id_enum = SupId(self._frame_id)
                    frame = SupFrame(frame_id=frame_id_enum, payload=bytes(self._payload))
                    self._reset_state()
                    return frame
                except ValueError:
                    # Invalid frame ID, but protocol structure was correct
                    self._reset_state()
            else:
                self._last_error = SupParsingResult.ERROR_EOF
                self._reset_state()

        return None

    def get_last_error(self) -> Optional[SupParsingResult]:
        """Returns the result of the last completed parsing attempt."""
        return self._last_error


def _calculate_checksum(frame_id: int, payload: bytes) -> int:
    """Calculates the checksum for a given frame ID and payload."""
    checksum = (frame_id + len(payload)) & 0xFF
    for byte in payload:
        checksum = (checksum + byte) & 0xFF
    return checksum


def create_frame(frame_id: SupId, payload: bytes = b'') -> bytes:
    """
    Constructs a complete SUP frame.

    Args:
        frame_id: The identifier for the frame.
        payload: The data payload for the frame. Max 128 bytes.

    Returns:
        A bytes object representing the full, ready-to-send SUP frame.

    Raises:
        ValueError: If the payload size exceeds MAX_PAYLOAD_SIZE.
    """
    payload_size = len(payload)
    if payload_size > MAX_PAYLOAD_SIZE:
        raise ValueError(f"Payload size {payload_size} exceeds maximum of {MAX_PAYLOAD_SIZE}")

    checksum = _calculate_checksum(frame_id.value, payload)
    
    frame = bytearray()
    frame.append(SOF)
    frame.append(frame_id.value)
    frame.append(payload_size)
    frame.extend(payload)
    frame.append(checksum)
    frame.append(EOF)
    
    return bytes(frame)


def create_ack_frame(ack_for_id: SupId) -> bytes:
    """Creates an ACK frame for a given original frame ID."""
    return create_frame(SupId.ACK, payload=bytes([ack_for_id.value]))


def create_nack_frame(nack_for_id: SupId) -> bytes:
    """Creates a NACK frame for a given original frame ID."""
    return create_frame(SupId.NACK, payload=bytes([nack_for_id.value]))


def create_data_frame(data: bytes) -> bytes:
    """Creates a DATA frame with the given payload."""
    return create_frame(SupId.DATA, payload=data)
