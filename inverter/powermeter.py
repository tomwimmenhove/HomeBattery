#!/usr/bin/env python3
"""
gtn_capture.py -- read GTN RS485 frames and print power (+ optional current).

Usage:
  pip3 install pyserial
  python3 gtn_capture.py --port /dev/ttyUSB0 --baud 4800 --voltage 230 --outfile capture.csv

Notes:
- Many community reports show 8-byte frames sent ~4Hz with structure:
  [0]=0x24, [1]=0x56, [2]=0x00, [3]=0x21, [4]=P_hi, [5]=P_lo, [6]=0x80, [7]=CHK
  CHK = (0xFF - sum(bytes[1..6])) & 0xFF
  (The code below accepts any frame that starts with 0x24 and validates checksum.)
"""
import argparse, time, sys
from datetime import datetime
import serial

def calc_chk(frame6):
    """frame6 is bytes[1..6] as integers"""
    s = sum(frame6) & 0xFF
    return (0xFF - s) & 0xFF

def hexdump(b):
    return ' '.join(f'{x:02x}' for x in b)

def parse_frame(frame):
    """Given 8-byte frame (bytes-like), validate and extract fields.
       Returns (ok, info_dict)"""
    if len(frame) != 8:
        return False, {'err': 'len!=8'}
    if frame[0] != 0x24:
        return False, {'err': f'bad header 0x{frame[0]:02x}'}
    expected = calc_chk(frame[1:7])
    if expected != frame[7]:
        return False, {'err': f'bad chk got 0x{frame[7]:02x} exp 0x{expected:02x}'}
    # Extract power (bytes indices 4 and 5)
    power = (frame[4] << 8) | frame[5]
    return True, {'power_w': power, 'raw': bytes(frame)}

def main():
    p = argparse.ArgumentParser(description='Capture GTN RS485 frames and print power/current.')
    p.add_argument('--port', '-p', default='/dev/ttyUSB0', help='serial port')
    p.add_argument('--baud', '-b', type=int, default=4800, help='baud rate (default: 4800)')
    p.add_argument('--voltage', '-v', type=float, default=None,
                   help='nominal voltage to compute current = P/V (optional)')
    p.add_argument('--outfile', '-o', default=None, help='append CSV: timestamp,power,current,rawhex')
    p.add_argument('--timeout', type=float, default=0.02, help='serial read timeout (s)')
    args = p.parse_args()

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baud,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE, timeout=args.timeout)
    except Exception as e:
        print(f'Failed to open {args.port}: {e}', file=sys.stderr)
        sys.exit(2)

    print(f'Opened {args.port} @ {args.baud} baud. Waiting for frames...')
    buf = bytearray()
    try:
        while True:
            chunk = ser.read(256)  # read up to 256 bytes
            if chunk:
                buf.extend(chunk)
                # scan buffer for possible frames; keep scanning until not enough bytes
                i = 0
                while i <= len(buf) - 8:
                    if buf[i] != 0x24:
                        i += 1
                        continue
                    candidate = bytes(buf[i:i+8])
                    ok, info = parse_frame(candidate)
                    if ok:
                        ts = datetime.now().isoformat(sep=' ', timespec='milliseconds')
                        power_w = info['power_w']
                        current_a = None
                        if args.voltage and args.voltage > 0:
                            # avoid divide-by-zero; compute float current
                            current_a = power_w / args.voltage
                        rawhex = hexdump(candidate)
                        # Print a compact single-line readout
                        if current_a is None:
                            print(f'[{ts}] Power: {power_w} W   raw: {rawhex}')
                        else:
                            print(f'[{ts}] Power: {power_w} W   Current: {current_a:.3f} A (V={args.voltage})   raw: {rawhex}')
                        # optional CSV append
                        if args.outfile:
                            with open(args.outfile, 'a', encoding='utf-8') as f:
                                if current_a is None:
                                    f.write(f'{ts},{power_w},, "{rawhex}"\n')
                                else:
                                    f.write(f'{ts},{power_w},{current_a:.6f},"{rawhex}"\n')
                        # consume the 8 bytes from buffer and continue scanning from start
                        del buf[0:i+8]
                        i = 0
                    else:
                        # not a valid frame at this offset -> maybe false header or bad chk;
                        # to avoid infinite loop, if header matched but checksum bad, drop this header byte
                        # (some noise on bus can produce 0x24). Skip this header and continue scanning.
                        # Optionally print debug every N fails (not here to remain quiet).
                        i += 1
                # keep buffer trimmed to last 16 bytes to avoid runaway growth
                if len(buf) > 64:
                    # keep tail where a frame could start
                    buf = buf[-32:]
            else:
                # no bytes received in this timeout slice; loop again
                continue
    except KeyboardInterrupt:
        print('\nInterrupted by user.')
    finally:
        ser.close()

if __name__ == '__main__':
    main()
