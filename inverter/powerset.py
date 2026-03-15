#!/usr/bin/env python3
"""
gtn_send.py

Create and optionally send a GTN-style 8-byte packet:

  [0] = 0x24
  [1] = 0x56
  [2] = 0x00
  [3] = 0x21
  [4] = P_hi
  [5] = P_lo
  [6] = 0x80
  [7] = CHK  where CHK = (0xFF - sum(bytes[1..6])) & 0xFF

Usage examples:
  # dry-run: print the packet for power=1234 W
  python3 gtn_send.py 1234 --dry-run

  # send the packet once on /dev/ttyUSB0 @ 4800 baud
  python3 gtn_send.py 1234 --port /dev/ttyUSB0 --baud 4800

  # send the packet 10 times with 250 ms spacing (useful for testing)
  python3 gtn_send.py 1234 --repeat 10 --delay 0.25 --port /dev/ttyUSB0

Install dependency:
  pip3 install pyserial
"""
from __future__ import annotations
import argparse
import sys
import time
from typing import Sequence

def calc_chk(b: Sequence[int]) -> int:
    """Compute checksum as used by GTN projects:
       CHK = (0xFF - sum(bytes[1..6])) & 0xFF
       b should be an iterable of bytes; we assume caller supplies bytes[1..6].
    """
    s = sum(b) & 0xFF
    return (0xFF - s) & 0xFF

def build_frame(power_w: int) -> bytes:
    """Build an 8-byte GTN-style frame for the given power in watts.
       power_w is clamped to 0..0xFFFF (unsigned 16-bit).
    """
    if power_w < 0:
        raise ValueError('power must be non-negative')
    if power_w > 0xFFFF:
        # clamp and warn
        print(f'Warning: power {power_w} > 65535, clamping to 65535', file=sys.stderr)
        power_w = 0xFFFF
    p_hi = (power_w >> 8) & 0xFF
    p_lo = power_w & 0xFF
    frame = bytearray(8)
    frame[0] = 0x24
    frame[1] = 0x56
    frame[2] = 0x00
    frame[3] = 0x21
    frame[4] = p_hi
    frame[5] = p_lo
    frame[6] = 0x80
    frame[7] = calc_chk(frame[1:7])
    return bytes(frame)

def hexdump(b: bytes) -> str:
    return ' '.join(f'{x:02x}' for x in b)

def send_frame(port: str, baud: int, frame: bytes, use_rs485: bool = False, timeout: float = 1.0):
    """Open serial port and send frame. Optionally enable RS-485 mode if available."""
    try:
        import serial
    except Exception as e:
        raise RuntimeError('pyserial is required to send: pip3 install pyserial') from e

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = timeout

    try:
        ser.open()
    except Exception as e:
        raise RuntimeError(f'Failed to open {port}: {e}') from e

    # Try to enable RS-485 settings if requested and supported
    if use_rs485:
        try:
            # RS485Settings() exists in pyserial >= 3.0
            ser.rs485_mode = serial.rs485.RS485Settings()
        except Exception as e:
            # Non-fatal: continue without hardware DE/RE toggle
            print(f'Warning: cannot enable rs485_mode: {e}', file=sys.stderr)

    try:
        # write entire frame in one go
        n = ser.write(frame)
        ser.flush()
        if n != len(frame):
            raise RuntimeError(f'only wrote {n}/{len(frame)} bytes')
    finally:
        ser.close()

def main():
    p = argparse.ArgumentParser(description='Construct and optionally send GTN-style 8-byte packets.')
    p.add_argument('power', type=int, help='power in watts (integer)')
    p.add_argument('--port', '-p', default=None, help='serial port to send (e.g. /dev/ttyUSB0). If omitted, prints only (dry-run).')
    p.add_argument('--baud', '-b', type=int, default=4800, help='baud rate when sending (default: 4800)')
    p.add_argument('--dry-run', action='store_true', help='only print packet hex; do not open serial port')
    p.add_argument('--repeat', '-r', type=int, default=1, help='how many times to send the packet (default: 1)')
    p.add_argument('--delay', '-d', type=float, default=0.0, help='delay in seconds between repeated sends (default: 0.0)')
    p.add_argument('--rs485', action='store_true', help='attempt to enable pyserial RS-485 mode (hardware DE/RE).')
    args = p.parse_args()

    try:
        frame = build_frame(args.power)
    except ValueError as e:
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(2)

    print(f'Frame for power={args.power} W: {hexdump(frame)}')
    if args.dry_run or not args.port:
        print('Dry run; not sending.')
        sys.exit(0)

    for i in range(args.repeat):
        try:
            send_frame(args.port, args.baud, frame, use_rs485=args.rs485)
            print(f'Sent ({i+1}/{args.repeat}) to {args.port} @ {args.baud}')
        except Exception as e:
            print(f'Failed to send on iteration {i+1}: {e}', file=sys.stderr)
            sys.exit(3)
        if i != args.repeat - 1 and args.delay > 0:
            time.sleep(args.delay)

if __name__ == '__main__':
    main()

