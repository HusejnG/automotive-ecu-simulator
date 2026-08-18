"""
signal_codec.py

Independent Python implementation of the CAN signal decoding used by the
C++ side (bit-protocol-parser). Deliberately *not* a binding to the C++
code: a diagnostic tool is normally a separate program from the ECU
software it talks to, often written by a different team in a different
language. Having two independent implementations agree is a stronger
check that the frame format is specified correctly than calling the same
code twice would be.

Bit numbering matches the C++ side: MSB-first within each byte and across
byte boundaries (Motorola convention).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SignalSpec:
    """Mirrors SignalSpec in the C++ signal_codec.h."""
    name: str
    bit_length: int      # 1-64
    is_signed: bool
    scale: float         # physical units per raw LSB
    offset: float        # physical value when raw == 0


class BitReader:
    """Reads MSB-first bit fields out of a bytes object."""

    def __init__(self, data: bytes):
        self._data = data
        self._bit_pos = 0

    def read_bits(self, bit_count: int) -> int:
        value = 0
        for _ in range(bit_count):
            byte_index = self._bit_pos // 8
            bit_in_byte = 7 - (self._bit_pos % 8)

            bit = 0
            if byte_index < len(self._data):
                bit = (self._data[byte_index] >> bit_in_byte) & 1
            value = (value << 1) | bit
            self._bit_pos += 1
        return value

    def read_signed(self, bit_count: int) -> int:
        """Reads bit_count bits and sign-extends as two's complement."""
        raw = self.read_bits(bit_count)
        if bit_count == 0 or bit_count >= 64:
            return raw
        sign_bit = 1 << (bit_count - 1)
        if raw & sign_bit:
            return raw - (1 << bit_count)
        return raw

    @property
    def bit_position(self) -> int:
        return self._bit_pos


def decode_signal(reader: BitReader, spec: SignalSpec) -> float:
    raw = reader.read_signed(spec.bit_length) if spec.is_signed \
        else reader.read_bits(spec.bit_length)
    return raw * spec.scale + spec.offset


def decode_frame(specs: list[SignalSpec], data: bytes) -> dict[str, float]:
    """Decodes an ordered set of signals out of one frame payload."""
    reader = BitReader(data)
    return {spec.name: decode_signal(reader, spec) for spec in specs}


def parse_hex(hex_string: str) -> bytes:
    """'94 70 00 00 50 41' -> b'\\x94\\x70\\x00\\x00\\x50\\x41'"""
    return bytes(int(token, 16) for token in hex_string.split())


# The BMS status frame layout, matching bms_node.h. If the C++ side ever
# changes this table, the integration tests will catch the mismatch --
# which is the point of keeping a second copy rather than sharing one.
BMS_STATUS_SIGNALS = [
    SignalSpec("pack_voltage_v",  16, False, 0.01,   0.0),
    SignalSpec("pack_current_a",  16, True,  0.1,    0.0),
    SignalSpec("state_of_charge",  8, False, 0.5,    0.0),
    SignalSpec("temperature_c",    8, False, 1.0,  -40.0),
]

# UDS data identifiers exposed by uds_server.h, and how to decode each.
UDS_DIDS = {
    0x1001: SignalSpec("pack_voltage_v",  16, False, 0.01,   0.0),
    0x1002: SignalSpec("pack_current_a",  16, True,  0.1,    0.0),
    0x1003: SignalSpec("state_of_charge",  8, False, 0.5,    0.0),
    0x1004: SignalSpec("temperature_c",    8, False, 1.0,  -40.0),
}

DIAGNOSTIC_SESSIONS = {
    0x01: "Default",
    0x02: "Programming",
    0x03: "Extended",
}
