#!/usr/bin/env python3
"""
charger_manual.py

Read Modbus-RTU holding (0x03) or input (0x04) registers
using manual RTS toggling for transmit/receive.

Added:
- --type {holding,input}
"""

from __future__ import annotations
import argparse
import serial
import sys
import time
from typing import Tuple, Optional
import math
import json

# ---------- CRC16 (Modbus) ----------
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 0x0001):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

# ---------- Build request ----------
def build_read_request(slave: int, start_addr: int, count: int, func: int) -> bytes:
    pdu = bytes([
        slave & 0xFF,
        func & 0xFF,
        (start_addr >> 8) & 0xFF,
        start_addr & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF
    ])
    crc = crc16_modbus(pdu)
    return pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

# ---------- Build write single register (0x06) ----------
def build_write_single_register_request(slave: int, addr: int, value: int) -> bytes:
    func = 0x06
    pdu = bytes([
        slave & 0xFF,
        func,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF
    ])
    crc = crc16_modbus(pdu)
    return pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

# ---------- Parse response ----------
def parse_read_response(resp: bytes, expected_slave: int, expected_count: int, expected_func: int) -> Tuple[bool, Optional[str], Optional[list]]:
    if len(resp) < 5:
        return False, f"Response too short ({len(resp)} bytes).", None

    rx_crc = resp[-2] | (resp[-1] << 8)
    body = resp[:-2]
    calc_crc = crc16_modbus(body)
    if rx_crc != calc_crc:
        return False, f"CRC mismatch: rx=0x{rx_crc:04X} calc=0x{calc_crc:04X}", None

    slave = body[0]
    func = body[1]

    if slave != (expected_slave & 0xFF):
        return False, f"Response slave ID {slave} != expected {expected_slave}", None

    if func & 0x80:
        if len(body) >= 3:
            excode = body[2]
            return False, f"Modbus exception: func 0x{func:02X}, code {excode}", None
        return False, "Malformed exception response", None

    if func != expected_func:
        return False, f"Unexpected function code 0x{func:02X}", None

    bytecount = body[2]
    if bytecount != expected_count * 2:
        return False, f"Unexpected byte count {bytecount}", None

    data = body[3:]
    if len(data) != bytecount:
        return False, "Data length mismatch", None

    regs = []
    for i in range(0, len(data), 2):
        regs.append((data[i] << 8) | data[i+1])

    return True, None, regs

# ---------- Linear helpers ----------
def linear11_to_float(word: int) -> float:
    tmp = word & 0xFFFF
    mant = tmp & 0x07FF
    if mant & 0x0400:
        mant -= 0x0800
    exp = (tmp >> 11) & 0x1F
    if exp & 0x10:
        exp -= 0x20
    if exp >= 0:
        return float(mant) * float(1 << exp)
    else:
        return float(mant) / float(1 << (-exp))

def linear16_to_float(word: int) -> float:
#    w = word & 0xFFFF
#    if w & 0x8000:
#        w -= 0x10000
    return float(word / 512)

# ---------- Float -> Linear11 ----------
def float_to_linear11(value: float) -> int:
    """
    Encode float into Linear11 format.
    Returns 16-bit integer.
    """

    if math.isnan(value) or math.isinf(value):
        raise ValueError("Cannot encode NaN or Inf in Linear11")

    if value == 0.0:
        return 0

    # Decompose value = m * 2^exp with |m| in [0.5,1)
    m, exp = math.frexp(value)

    SHIFT = 10  # mantissa fractional bits
    mant = int(round(m * (1 << SHIFT)))

    # Handle rounding overflow (1024 case)
    if mant > 1023:
        mant >>= 1
        exp += 1
    if mant < -1024:
        mant >>= 1
        exp += 1

    exp_field = exp - SHIFT

    if exp_field < -16 or exp_field > 15:
        raise ValueError("Value out of Linear11 exponent range")

    # Pack fields (two's complement)
    mant &= 0x07FF
    exp_field &= 0x1F

    return (exp_field << 11) | mant


# ---------- Float -> Linear16 ----------
def float_to_linear16(value: float) -> int:
    """
    Encode float into Linear16 (signed 16-bit integer).
    """

    scaled = value * 512
    i = int(round(scaled))

    if i < -32768 or i > 32767:
        raise ValueError("Value out of Linear16 range")

    return i & 0xFFFF


# ---------- Serial send/receive ----------
def send_request_and_read_response(
    ser: serial.Serial,
    request: bytes,
    expected_response_len: int,
    rts_delay: float = 0.002,
) -> bytes:

    if not ser.is_open:
        raise RuntimeError("Serial port not open")

    ser.setRTS(True)
    ser.write(request)
    ser.flush()
    time.sleep(rts_delay)
    ser.setRTS(False)

    resp = ser.read(expected_response_len)
    if len(resp) == 0:
        raise RuntimeError("Timeout waiting for response")

    if len(resp) < expected_response_len:
        resp += ser.read(expected_response_len - len(resp))

    return resp

def modbus_write_register(
    ser: serial.Serial,
    slave: int,
    addr: int,
    value: int,
    timeout: float,
    rts_delay: float,
):
    req = build_write_single_register_request(slave, addr, value)

    expected_len = 8  # echo response is always 8 bytes

    prev_timeout = ser.timeout
    ser.timeout = timeout
    try:
        raw = send_request_and_read_response(ser, req, expected_len, rts_delay)
    finally:
        ser.timeout = prev_timeout

    # Validate echo
    if len(raw) != 8:
        raise RuntimeError("Invalid write response length")

    if raw != req:
        raise RuntimeError("Write verification failed (echo mismatch)")

# ---------- High-level read ----------
def modbus_read_registers(
    ser: serial.Serial,
    slave: int,
    start: int,
    count: int,
    timeout: float,
    rts_delay: float,
    func: int,
) -> list:

    req = build_read_request(slave, start, count, func)
    expected_len = 1 + 1 + 1 + 2*count + 2

    prev_timeout = ser.timeout
    ser.timeout = timeout
    try:
        raw = send_request_and_read_response(ser, req, expected_len, rts_delay)
    finally:
        ser.timeout = prev_timeout

    ok, err, regs = parse_read_response(raw, slave, count, func)
    if not ok:
        raise RuntimeError(err)

    return regs

# ---------- CLI ----------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    p.add_argument("unit", nargs="?", type=int, default=1)
    p.add_argument("--start", type=int, default=0)
    p.add_argument("--count", type=int, default=14)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--parity", choices=["N","E","O"], default="E")
    p.add_argument("--timeout", type=float, default=1.0)
    p.add_argument("--rts-delay", type=float, default=0.002)
    p.add_argument("--type", choices=["holding", "input"], default="holding",
                   help="Register type: holding (0x03) or input (0x04)")
    p.add_argument("--write-linear11", type=float, help="Write float as Linear11 to --start register")
    p.add_argument("--write-linear16", type=float, help="Write float as Linear16 to --start register")

    args = p.parse_args()

    parity_map = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN, "O": serial.PARITY_ODD}

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=parity_map[args.parity],
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        )
    except Exception as e:
        print(f"ERROR: cannot open serial port: {e}", file=sys.stderr)
        sys.exit(2)

    # ----- Write mode -----
    if args.write_linear11 is not None:
        encoded = float_to_linear11(args.write_linear11)
        print(f"Writing Linear11 value {args.write_linear11} -> 0x{encoded:04X} to register {args.start}")
        modbus_write_register(
            ser=ser,
            slave=args.unit,
            addr=args.start,
            value=encoded,
            timeout=args.timeout,
            rts_delay=args.rts_delay,
        )
        print("Write successful.")
        ser.close()
        return
    
    if args.write_linear16 is not None:
        encoded = float_to_linear16(args.write_linear16)
        print(f"Writing Linear16 value {args.write_linear16} -> 0x{encoded:04X} to register {args.start}")
        modbus_write_register(
            ser=ser,
            slave=args.unit,
            addr=args.start,
            value=encoded,
            timeout=args.timeout,
            rts_delay=args.rts_delay,
        )
        print("Write successful.")
        ser.close()
        return
    
    func = 0x03 if args.type == "holding" else 0x04

    try:
        regs = modbus_read_registers(
            ser=ser,
            slave=args.unit,
            start=args.start,
            count=args.count,
            timeout=args.timeout,
            rts_delay=args.rts_delay,
            func=func,
        )
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        ser.close()
        sys.exit(3)

    result = {
        "port": args.port,
        "unit": args.unit,
        "register_type": args.type,
        "start": args.start,
        "count": len(regs),
        "registers": []
    }
    
    for i, val in enumerate(regs):
        addr = args.start + i
        signed = val if val < 0x8000 else val - 0x10000
        lin11 = linear11_to_float(val)
        lin16 = linear16_to_float(val)
    
        result["registers"].append({
            "address": addr,
            "unsigned": val,
            "signed": signed,
            "hex": f"0x{val:04X}",
            "linear11": lin11,
            "linear16": lin16
        })
    
    print(json.dumps(result, indent=2))

#    print(f"\nRead {len(regs)} {args.type} registers starting at {args.start}")
#    print("Addr  Unsigned  Signed   Hex     Linear11        Linear16")
#    print("----  --------  ------  ------  --------------  --------------")

#    for i, val in enumerate(regs):
#        addr = args.start + i
#        signed = val if val < 0x8000 else val - 0x10000
#        lin11 = linear11_to_float(val)
#        lin16 = linear16_to_float(val)
#        print(f"{addr:4d}  {val:8d}  {signed:6d}  0x{val:04X}  {lin11:14.6g}  {lin16:14.6g}")

    ser.close()

if __name__ == "__main__":
    main()
