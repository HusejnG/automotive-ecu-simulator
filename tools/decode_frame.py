#!/usr/bin/env python3
"""
decode_frame.py

Decodes a raw CAN or UDS frame into readable values -- the small utility
you end up wanting constantly when staring at raw bus traffic.

Usage:
    python3 decode_frame.py --bms "94 70 00 00 50 41"
    python3 decode_frame.py --uds "62 10 01 9c 40"
    python3 decode_frame.py --uds "7f 22 12"

Examples of what it handles:
    BMS status frame  -> all four signals with units
    UDS positive resp -> service, DID, decoded value
    UDS negative resp -> service and the reason it was rejected
"""

import argparse
import sys

from signal_codec import (
    BMS_STATUS_SIGNALS,
    DIAGNOSTIC_SESSIONS,
    UDS_DIDS,
    BitReader,
    decode_frame,
    decode_signal,
    parse_hex,
)

UNITS = {
    "pack_voltage_v": "V",
    "pack_current_a": "A",
    "state_of_charge": "%",
    "temperature_c": "°C",
}

NRC_NAMES = {
    0x11: "ServiceNotSupported",
    0x12: "SubFunctionNotSupported",
    0x24: "RequestSequenceError",
    0x35: "InvalidKey",
}

SERVICE_NAMES = {
    0x10: "DiagnosticSessionControl",
    0x22: "ReadDataByIdentifier",
    0x27: "SecurityAccess",
}


def show_bms(hex_string: str) -> int:
    data = parse_hex(hex_string)
    if len(data) < 6:
        print(f"Warning: BMS status frames are 6 bytes, got {len(data)} — "
              "trailing signals will read as zero.", file=sys.stderr)

    values = decode_frame(BMS_STATUS_SIGNALS, data)
    print("BMS status frame:")
    for spec in BMS_STATUS_SIGNALS:
        unit = UNITS.get(spec.name, "")
        print(f"  {spec.name:<18} {values[spec.name]:>8.2f} {unit}")
    return 0


def show_uds(hex_string: str) -> int:
    data = parse_hex(hex_string)
    if not data:
        print("Empty frame.", file=sys.stderr)
        return 1

    sid = data[0]

    # Negative response: 0x7F, original SID, NRC
    if sid == 0x7F:
        if len(data) < 3:
            print("Malformed negative response (expected 3 bytes).", file=sys.stderr)
            return 1
        service = SERVICE_NAMES.get(data[1], f"0x{data[1]:02X}")
        reason = NRC_NAMES.get(data[2], f"unknown NRC 0x{data[2]:02X}")
        print(f"UDS negative response:")
        print(f"  service   {service}")
        print(f"  rejected  {reason}")
        return 0

    # Positive responses have bit 6 set: response SID = request SID + 0x40
    request_sid = sid - 0x40
    service = SERVICE_NAMES.get(request_sid)
    if service is None:
        print(f"Unrecognized UDS frame (first byte 0x{sid:02X}).", file=sys.stderr)
        return 1

    print(f"UDS positive response: {service}")

    if request_sid == 0x10 and len(data) >= 2:
        session = DIAGNOSTIC_SESSIONS.get(data[1], f"unknown (0x{data[1]:02X})")
        print(f"  active session   {session}")

    elif request_sid == 0x22 and len(data) >= 3:
        did = (data[1] << 8) | data[2]
        print(f"  data identifier  0x{did:04X}")
        if did in UDS_DIDS:
            spec = UDS_DIDS[did]
            value = decode_signal(BitReader(data[3:]), spec)
            unit = UNITS.get(spec.name, "")
            print(f"  {spec.name:<16} {value:.2f} {unit}")
        elif did == 0xF186 and len(data) >= 4:
            session = DIAGNOSTIC_SESSIONS.get(data[3], f"unknown (0x{data[3]:02X})")
            print(f"  active session   {session}")
        else:
            payload = " ".join(f"{b:02x}" for b in data[3:])
            print(f"  raw payload      {payload}")

    elif request_sid == 0x27 and len(data) >= 2:
        sub = data[1]
        if sub % 2 == 1 and len(data) >= 4:
            seed = (data[2] << 8) | data[3]
            print(f"  seed             0x{seed:04X}")
        else:
            print(f"  key accepted (sub-function 0x{sub:02X})")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Decode a raw CAN/UDS frame into readable values.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--bms", metavar="HEX",
                       help='BMS status frame, e.g. "94 70 00 00 50 41"')
    group.add_argument("--uds", metavar="HEX",
                       help='UDS response frame, e.g. "62 10 01 9c 40"')
    args = parser.parse_args()

    try:
        if args.bms:
            return show_bms(args.bms)
        return show_uds(args.uds)
    except ValueError as exc:
        print(f"Could not parse hex input: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
