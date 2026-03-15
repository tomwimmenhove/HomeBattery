#!/usr/bin/env python3
"""
gtn_send_stream.py

Continuously send GTN-style 8-byte frames (indefinitely). Power values
are read from stdin (one value per line). Stdin is read non-blocking;
if no new value is available the last value is reused.

Usage examples:
  # run, reading power values from stdin (e.g. echo "1234" > fifo)
  python3 gtn_send_stream.py --port /dev/ttyUSB0 --baud 4800

  # run with 4 Hz sends (default interval=0.25s) and RS-485 mode
  python3 gtn_send_stream.py --port /dev/ttyUSB0 --rs485 --interval 0.25

One-liner to install dependency and run (example):
  pip3 install pyserial && python3 gtn_send_stream.py --port /dev/ttyUSB0
"""
from __future__ import annotations
import argparse
import sys
import time
import os
import fcntl
from datetime import datetime

def calc_chk(b):
    s = sum(b) & 0xFF
    return (0xFF - s) & 0xFF

def build_frame(power_w: int) -> bytes:
    if power_w < 0:
        raise ValueError("power must be >= 0")
    if power_w > 0xFFFF:
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

def open_serial(port: str, baud: int, timeout: float = 1.0, try_rs485: bool = False):
    try:
        import serial
    except Exception as e:
        raise RuntimeError("pyserial required: pip3 install pyserial") from e

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = timeout

    ser.open()
    if try_rs485:
        # best-effort: may raise on unsupported platforms
        try:
            ser.rs485_mode = serial.rs485.RS485Settings()
        except Exception as e:
            # non-fatal; inform user
            print(f'Warning: cannot enable rs485_mode: {e}', file=sys.stderr)
    return ser

def set_stdin_nonblocking():
    fd = sys.stdin.fileno()
    old_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)
    return old_flags

def restore_stdin_flags(old_flags):
    fd = sys.stdin.fileno()
    fcntl.fcntl(fd, fcntl.F_SETFL, old_flags)

def try_parse_power(s: str):
    s = s.strip()
    if not s:
        return None
    try:
        if '.' in s:
            v = float(s)
            return int(round(v))
        else:
            return int(s, 0)  # allow hex (0x...), decimal
    except Exception:
        return None

def read_available_lines_from_stdin(buf_bytes: bytearray, eof_flag_ref):
    """Read any available bytes from stdin (non-blocking) and return a list of complete lines.
       buffer holds leftover partial line bytes (utf-8). eof_flag_ref is a [bool] mutable
       that will be set True on EOF (b'').
    """
    fd = sys.stdin.fileno()
    lines = []
    try:
        data = os.read(fd, 4096)
        if data == b'':
            eof_flag_ref[0] = True
            return lines
        buf_bytes.extend(data)
    except BlockingIOError:
        # no data available
        return lines
    except OSError as e:
        # unexpected read error: mark EOF and return
        eof_flag_ref[0] = True
        return lines

    # process buffer into UTF-8 string; handle partial UTF-8 safely
    try:
        s = buf_bytes.decode('utf-8')
    except UnicodeDecodeError:
        # if incomplete multi-byte sequence at end, try to decode what we can
        # find the longest prefix that decodes
        for cut in range(len(buf_bytes)-1, max(-1, len(buf_bytes)-8), -1):
            try:
                s = buf_bytes[:cut].decode('utf-8')
                remainder = buf_bytes[cut:]
                buf_bytes.clear()
                buf_bytes.extend(remainder)
                break
            except Exception:
                continue
        else:
            # give up; clear buffer to avoid infinite loop (unlikely)
            s = ""
            buf_bytes.clear()
    else:
        buf_bytes.clear()

    # split into lines; keep last partial line (if any) in buffer_bytes
    parts = s.splitlines(keepends=True)
    for part in parts:
        if part.endswith('\n') or part.endswith('\r'):
            lines.append(part.rstrip('\r\n'))
        else:
            # partial line: push back to buffer_bytes for next round
            buf_bytes.extend(part.encode('utf-8'))
    return lines

def main():
    p = argparse.ArgumentParser(description='Continuously send GTN-style frames; accept power from stdin (non-blocking).')
    p.add_argument('--port', '-p', default=None, help='serial port to send (e.g. /dev/ttyUSB0). If omitted, frames are printed but not sent.')
    p.add_argument('--baud', '-b', type=int, default=4800, help='baud rate (default: 4800)')
    p.add_argument('--interval', '-i', type=float, default=0.25, help='seconds between sends (default 0.25 -> 4Hz)')
    p.add_argument('--rs485', action='store_true', help='try to enable pyserial RS-485 mode (DE/RE toggling)')
    p.add_argument('--initial', type=int, default=0, help='initial power value (watts) used until stdin provides a value (default 0)')
    args = p.parse_args()

    ser = None
    if args.port:
        try:
            ser = open_serial(args.port, args.baud, timeout=1.0, try_rs485=args.rs485)
        except Exception as e:
            print(f'Failed to open serial port {args.port}: {e}', file=sys.stderr)
            sys.exit(2)
        print(f'Opened {args.port} @ {args.baud} baud. Sending every {args.interval}s')
    else:
        print(f'No --port given: will only print frames (dry run). Sending every {args.interval}s')

    # prepare non-blocking stdin
    old_stdin_flags = None
    stdin_eof = [False]
    stdin_buf = bytearray()
    try:
        old_stdin_flags = set_stdin_nonblocking()
    except Exception as e:
        print(f'Warning: could not set stdin non-blocking: {e}', file=sys.stderr)

    current_power = int(args.initial)
    if current_power < 0:
        current_power = 0
    last_print_ts = 0.0

    try:
        while True:
            # read any available stdin lines and update current_power (keep last valid)
            if not stdin_eof[0]:
                lines = read_available_lines_from_stdin(stdin_buf, stdin_eof)
                for line in lines:
                    parsed = try_parse_power(line)
                    if parsed is None:
                        print(f'Ignored bad stdin input: {line!r}', file=sys.stderr)
                    else:
                        if parsed < 0:
                            print(f'Clamping negative power {parsed} to 0', file=sys.stderr)
                            parsed = 0
                        if parsed > 0xFFFF:
                            print(f'Clamping power {parsed} to 65535', file=sys.stderr)
                            parsed = 0xFFFF
                        current_power = int(parsed)
                        ts = datetime.now().isoformat(sep=' ', timespec='milliseconds')
                        print(f'[{ts}] stdin -> power = {current_power} W')

            # build and send frame
            frame = build_frame(current_power)
            ts = datetime.now().isoformat(sep=' ', timespec='milliseconds')
            if ser:
                try:
                    n = ser.write(frame)
                    ser.flush()
                    if n != len(frame):
                        print(f'Warning: wrote {n}/{len(frame)} bytes', file=sys.stderr)
                except Exception as e:
                    print(f'Error writing to serial: {e}', file=sys.stderr)
            # always print a short status line (not too spammy)
            now = time.monotonic()
            if now - last_print_ts >= max(1.0, args.interval):
                print(f'[{ts}] Sent power={current_power} W  raw={hexdump(frame)}')
                sys.stdout.flush()
                last_print_ts = now

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\nInterrupted by user. Exiting.')
    finally:
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        if old_stdin_flags is not None:
            try:
                restore_stdin_flags(old_stdin_flags)
            except Exception:
                pass

if __name__ == '__main__':
    main()
