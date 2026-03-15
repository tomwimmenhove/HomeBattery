"""
charger_api.py

FastAPI web service wrapper for the original `modbus_controller.py` script.

Endpoints implemented (examples):
 - GET  /api/health                       -> basic service health
 - GET  /api/measurements                 -> read input registers (func 0x04) and return parsed measurements
 - GET  /api/status                       -> read input registers (func 0x04) and parse status bits
 - GET  /api/config                       -> read holding registers (func 0x03) and return parsed configuration
 - POST /api/read                         -> read arbitrary registers (body: unit,start,count,register_type)
 - POST /api/write                        -> write single register (supports linear11, linear16 or raw 16-bit)

Usage: install dependencies: pip install fastapi uvicorn pyserial
Run: uvicorn charger_api:app --host 0.0.0.0 --port 8000

The service exposes the same encoding/decoding helpers (linear11/linear16) and uses manual RTS toggling like the original script.

"""
from __future__ import annotations
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
import serial
import threading
import time
from typing import List, Optional, Tuple, Dict
import math
from ttyfinder import find_tty_for_vidpid

app = FastAPI(title="Modbus RTU Charger API")

# ---------- Settings / defaults ----------
vidpid = '1a86:7523'
DEFAULT_PORT = find_tty_for_vidpid(vidpid)

##DEFAULT_PORT = "/dev/ttyUSB1"
DEFAULT_BAUD = 115200
DEFAULT_PARITY = "E"
DEFAULT_TIMEOUT = 1.0
DEFAULT_RTS_DELAY = 0.002

parity_map = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN, "O": serial.PARITY_ODD}

# ---------- Low-level helpers (copied + adapted) ----------

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


def parse_read_response(resp: bytes, expected_slave: int, expected_count: int, expected_func: int) -> Tuple[bool, Optional[str], Optional[List[int]]]:
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
    return float(word / 512)


def float_to_linear11(value: float) -> int:
    if math.isnan(value) or math.isinf(value):
        raise ValueError("Cannot encode NaN or Inf in Linear11")
    if value == 0.0:
        return 0
    m, exp = math.frexp(value)
    SHIFT = 10
    mant = int(round(m * (1 << SHIFT)))
    if mant > 1023:
        mant >>= 1
        exp += 1
    if mant < -1024:
        mant >>= 1
        exp += 1
    exp_field = exp - SHIFT
    if exp_field < -16 or exp_field > 15:
        raise ValueError("Value out of Linear11 exponent range")
    mant &= 0x07FF
    exp_field &= 0x1F
    return (exp_field << 11) | mant


def float_to_linear16(value: float) -> int:
    scaled = value * 512
    i = int(round(scaled))
    if i < -32768 or i > 32767:
        raise ValueError("Value out of Linear16 range")
    return i & 0xFFFF

# ---------- Serial manager (thread-safe) ----------

class SerialManager:
    """Manages a single serial port instance and serialises access with a lock.

    The manager opens the port lazily on first use. Every request may override
    the effective serial parameters (port, baud, parity, timeout, rts_delay).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._ser: Optional[serial.Serial] = None
        self._current_params = None

    def _open(self, port: str, baud: int, parity: str, timeout: float):
        if self._ser is not None:
            if self._current_params == (port, baud, parity, timeout):
                return
            # close and reopen with new params
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        try:
            self._ser = serial.Serial(port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                                      parity=parity_map.get(parity, serial.PARITY_EVEN),
                                      stopbits=serial.STOPBITS_ONE, timeout=timeout)
            self._current_params = (port, baud, parity, timeout)
        except Exception as e:
            raise RuntimeError(f"Cannot open serial port {port}: {e}")

    def send_request_and_read_response(self, port: str, baud: int, parity: str, timeout: float,
                                       rts_delay: float, request: bytes, expected_len: int) -> bytes:
        with self._lock:
            self._open(port, baud, parity, timeout)
            ser = self._ser
            if ser is None:
                raise RuntimeError("Serial port not open")

            ser.timeout = timeout
            ser.setRTS(True)
            ser.write(request)
            ser.flush()
            time.sleep(rts_delay)
            ser.setRTS(False)

            resp = ser.read(expected_len)
            if len(resp) == 0:
                raise RuntimeError("Timeout waiting for response")
            if len(resp) < expected_len:
                resp += ser.read(expected_len - len(resp))
            return resp

    def close(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                self._current_params = None

serial_manager = SerialManager()

# ---------- Higher-level modbus calls ----------

def modbus_read_registers(port: str, baud: int, parity: str, timeout: float, rts_delay: float,
                          slave: int, start: int, count: int, func: int) -> List[int]:
    req = build_read_request(slave, start, count, func)
    expected_len = 1 + 1 + 1 + 2*count + 2
    raw = serial_manager.send_request_and_read_response(port, baud, parity, timeout, rts_delay, req, expected_len)
    ok, err, regs = parse_read_response(raw, slave, count, func)
    if not ok:
        raise RuntimeError(err)
    return regs

def modbus_write_register(port: str, baud: int, parity: str, timeout: float, rts_delay: float,
                          slave: int, addr: int, value: int):
    req = build_write_single_register_request(slave, addr, value)
    expected_len = 8
    raw = serial_manager.send_request_and_read_response(port, baud, parity, timeout, rts_delay, req, expected_len)
    if len(raw) != 8:
        raise RuntimeError("Invalid write response length")
    if raw != req:
        raise RuntimeError("Write verification failed (echo mismatch)")

def modbus_read_coils(port: str, baud: int, parity: str, timeout: float, rts_delay: float,
                      slave: int, start: int, count: int) -> List[bool]:
    func = 0x01  # read coils
    req = build_read_request(slave, start, count, func)
    byte_count = (count + 7) // 8
    expected_len = 1 + 1 + 1 + byte_count + 2
    raw = serial_manager.send_request_and_read_response(port, baud, parity, timeout, rts_delay, req, expected_len)
    ok, err, coils = parse_coil_response(raw, slave, count, func)
    if not ok:
        raise RuntimeError(err)
    return coils

def modbus_write_coil(port: str, baud: int, parity: str, timeout: float, rts_delay: float,
                      slave: int, addr: int, value: bool):
    # Modbus "Write Single Coil" uses function 0x05 with value 0xFF00 for ON, 0x0000 for OFF
    val16 = 0xFF00 if value else 0x0000
    # If you already have build_write_single_coil_request use it. If not, a small builder:
    try:
        req = build_write_single_coil_request(slave, addr, val16)
    except NameError:
        # fallback builder (modbus RTU frame w/o CRC appended)
        # build_write_single_coil_request should return bytes including CRC
        frame = bytearray()
        frame.append(slave & 0xFF)
        frame.append(0x05)
        frame += (addr & 0xFFFF).to_bytes(2, byteorder="big")
        frame += (val16 & 0xFFFF).to_bytes(2, byteorder="big")
        # append CRC (little-endian)
        c = crc16_modbus(bytes(frame))
        frame += c.to_bytes(2, byteorder="little")
        req = bytes(frame)

    expected_len = 8
    raw = serial_manager.send_request_and_read_response(port, baud, parity, timeout, rts_delay, req, expected_len)
    if len(raw) != expected_len:
        raise RuntimeError("Invalid write response length")
    if raw != req:
        raise RuntimeError("Write verification failed (echo mismatch)")

# ---------- Pydantic models for requests ----------

class ReadRequest(BaseModel):
    unit: int = Field(1, ge=1, le=247)
    start: int = Field(0, ge=0)
    count: int = Field(1, ge=1, le=125)
    register_type: str = Field("holding", regex="^(holding|input)$")
    port: Optional[str] = DEFAULT_PORT
    baud: Optional[int] = DEFAULT_BAUD
    parity: Optional[str] = DEFAULT_PARITY
    timeout: Optional[float] = DEFAULT_TIMEOUT
    rts_delay: Optional[float] = DEFAULT_RTS_DELAY

class WriteRequest(BaseModel):
    unit: int = Field(1, ge=1, le=247)
    addr: int = Field(..., ge=0)
    value: float
    value_format: str = Field("linear11", regex="^(linear11|linear16|raw)$")
    port: Optional[str] = DEFAULT_PORT
    baud: Optional[int] = DEFAULT_BAUD
    parity: Optional[str] = DEFAULT_PARITY
    timeout: Optional[float] = DEFAULT_TIMEOUT
    rts_delay: Optional[float] = DEFAULT_RTS_DELAY

class CoilReadRequest(BaseModel):
    unit: int = Field(1, ge=1, le=247)
    start: int = Field(1, ge=1)
    count: int = Field(1, ge=1, le=2000)  # modbus allows up to 2000 coils per read
    port: Optional[str] = DEFAULT_PORT
    baud: Optional[int] = DEFAULT_BAUD
    parity: Optional[str] = DEFAULT_PARITY
    timeout: Optional[float] = DEFAULT_TIMEOUT
    rts_delay: Optional[float] = DEFAULT_RTS_DELAY

class CoilWriteRequest(BaseModel):
    unit: int = Field(1, ge=1, le=247)
    addr: int = Field(..., ge=1)
    value: bool
    port: Optional[str] = DEFAULT_PORT
    baud: Optional[int] = DEFAULT_BAUD
    parity: Optional[str] = DEFAULT_PARITY
    timeout: Optional[float] = DEFAULT_TIMEOUT
    rts_delay: Optional[float] = DEFAULT_RTS_DELAY

# ---------- Parsing helpers ----------

def parse_coil_response(raw: bytes, slave: int, count: int, func: int):
    """Return (ok: bool, err: str|None, coils: List[bool]|None)"""
    # minimal length: slave(1) + func(1) + bytecount(1) + crc(2) = 5
    if raw is None or len(raw) < 5:
        return False, "Short/empty response", None
    if raw[0] != slave:
        return False, f"Slave mismatch (expected {slave}, got {raw[0]})", None
    if raw[1] != func:
        return False, f"Function code mismatch (expected 0x{func:02X}, got 0x{raw[1]:02X})", None
    byte_count = raw[2]
    expected_len = 1 + 1 + 1 + byte_count + 2
    if len(raw) != expected_len:
        return False, f"Invalid response length (expected {expected_len}, got {len(raw)})", None
    # CRC check (last two bytes are CRC low, CRC high)
    resp_crc = int.from_bytes(raw[-2:], byteorder="little")
    if crc16_modbus(raw[:-2]) != resp_crc:
        return False, "CRC check failed", None
    payload = raw[3:3+byte_count]
    coils: List[bool] = []
    for i in range(count):
        b = payload[i // 8]
        bit = (b >> (i % 8)) & 0x01
        coils.append(bool(bit))
    return True, None, coils


def parse_measurements_from_input_regs(regs: List[int]) -> Dict[str, float]:
    temp1 = linear11_to_float(regs[0])
    temp2 = linear11_to_float(regs[1])
    temp3 = linear11_to_float(regs[2])
    vOut = linear16_to_float(regs[3])
    iOut = linear11_to_float(regs[4])
    pOut = vOut * iOut
    vIn = linear11_to_float(regs[5])
    return {"temperature1": temp1, "temperature2": temp2, "temperature3": temp3,
            "vout": vOut, "iout": iOut, "pout": pOut, "vin": vIn}


def parse_status_from_input_regs(regs: List[int]) -> Dict[str, bool]:
    # regs index mapping copied from original script
    statusWord = regs[6]
    statusVOut = regs[7]
    statusIOut = regs[8]
    statusInput = regs[9]
    statusTemp = regs[10]
    statusCml = regs[11]

    status_dict = {
       "faultVout": bool(statusWord & (1 << 15)),
       "faultIout": bool(statusWord & (1 << 14)),
       "faultInput": bool(statusWord & (1 << 13)),
       "powerGoodDeasserted": bool(statusWord & (1 << 11)),
       "unitOff": bool(statusWord & (1 << 6)),
       "voutOv": bool(statusWord & (1 << 5)),
       "ioutOc": bool(statusWord & (1 << 4)),
       "vinUv": bool(statusWord & (1 << 3)),
       "temperature": bool(statusWord & (1 << 2)),
       "cml": bool(statusWord & (1 << 1)),

       "voutOvFault": bool(statusVOut & (1 << 7)),
       "voutOvWarning": bool(statusVOut & (1 << 6)),
       "voutUvWarning": bool(statusVOut & (1 << 5)),
       "voutUvFault": bool(statusVOut & (1 << 4)),
       "tonMaxFault": bool(statusVOut & (1 << 2)),

       "ioutOcFault": bool(statusIOut & (1 << 7)),
       "ioutOcWarning": bool(statusIOut & (1 << 5)),
       "poutOpWarning": bool(statusIOut & (1 << 0)),

       "vinOvFault": bool(statusInput & (1 << 7)),
       "vinOvWarning": bool(statusInput & (1 << 6)),
       "vinUvWarning": bool(statusInput & (1 << 5)),
       "vinUvFault": bool(statusInput & (1 << 4)),
       "noInputOrUnitOff": bool(statusInput & (1 << 3)),

       "otFault": bool(statusTemp & (1 << 7)),
       "otWarning": bool(statusTemp & (1 << 6)),

       "invalidOrUnsupportedCommand": bool(statusCml & (1 << 7)),
       "invalidOrUnsupportedData": bool(statusCml & (1 << 6)),
       "packetError": bool(statusCml & (1 << 5)),
       "memoryFault": bool(statusCml & (1 << 4)),
    }
    return status_dict

# ---------- API endpoints ----------

@app.get("/api/health")
def health():
    return {"status": "ok"}

@app.get("/api/measurements")
def get_measurements(port: Optional[str] = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                     parity: str = DEFAULT_PARITY, timeout: float = DEFAULT_TIMEOUT,
                     rts_delay: float = DEFAULT_RTS_DELAY, unit: int = 1):
    try:
        regs = modbus_read_registers(port, baud, parity, timeout, rts_delay, unit, 0, 13, 0x04)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))
    measurements = parse_measurements_from_input_regs(regs)
    return {"unit": unit, "port": port, "measurements": measurements}

@app.get("/api/status")
def get_status(port: Optional[str] = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
               parity: str = DEFAULT_PARITY, timeout: float = DEFAULT_TIMEOUT,
               rts_delay: float = DEFAULT_RTS_DELAY, unit: int = 1):
    try:
        regs = modbus_read_registers(port, baud, parity, timeout, rts_delay, unit, 0, 13, 0x04)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))
    status = parse_status_from_input_regs(regs)
    return {"unit": unit, "port": port, "status": status}

@app.get("/api/config")
def get_config(port: Optional[str] = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
               parity: str = DEFAULT_PARITY, timeout: float = DEFAULT_TIMEOUT,
               rts_delay: float = DEFAULT_RTS_DELAY, unit: int = 1):
    try:
        regs = modbus_read_registers(port, baud, parity, timeout, rts_delay, unit, 0, 9, 0x03)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))

    stepVoltage = linear16_to_float(regs[0])
    minVoltage = linear16_to_float(regs[1])
    setVoltageLow = linear16_to_float(regs[2])
    setVoltageHigh = linear16_to_float(regs[3])
    setCurrentLow = linear11_to_float(regs[4])
    setCurrentHigh = linear11_to_float(regs[5])
    setPowerLow = regs[6]
    setPowerHigh = regs[7]
    tripVoltage = linear16_to_float(regs[8])

    return {"unit": unit, "port": port, "config": {
        "stepVoltage": stepVoltage,
        "minVoltage": minVoltage,
        "setVoltageLow": setVoltageLow,
        "setVoltageHigh": setVoltageHigh,
        "setCurrentLow": setCurrentLow,
        "setCurrentHigh": setCurrentHigh,
        "setPowerLow": setPowerLow,
        "setPowerHigh": setPowerHigh,
        "tripVoltage": tripVoltage,
    }}

@app.post("/api/read")
def post_read(req: ReadRequest):
    func = 0x03 if req.register_type == "holding" else 0x04
    try:
        regs = modbus_read_registers(req.port, req.baud, req.parity, req.timeout, req.rts_delay, req.unit, req.start, req.count, func)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))
    return {"unit": req.unit, "port": req.port, "start": req.start, "count": req.count, "registers": regs}

@app.post("/api/write")
def post_write(req: WriteRequest):
    try:
        if req.value_format == "linear11":
            encoded = float_to_linear11(req.value)
        elif req.value_format == "linear16":
            encoded = float_to_linear16(req.value)
        else:
            # raw 16-bit value, allow integer-form floats that are actually integers
            encoded = int(req.value) & 0xFFFF
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Encoding error: {e}")

    try:
        modbus_write_register(req.port, req.baud, req.parity, req.timeout, req.rts_delay, req.unit, req.addr, encoded)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))

    return {"unit": req.unit, "port": req.port, "addr": req.addr, "value_encoded": f"0x{encoded:04X}"}

@app.post("/api/coils/read")
def post_read_coils(req: CoilReadRequest):
    try:
        coils = modbus_read_coils(req.port, req.baud, req.parity, req.timeout, req.rts_delay, req.unit, req.start, req.count)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))
    return {"unit": req.unit, "port": req.port, "start": req.start, "count": req.count, "coils": coils}

@app.post("/api/coils/write")
def post_write_coil(req: CoilWriteRequest):
    try:
        modbus_write_coil(req.port, req.baud, req.parity, req.timeout, req.rts_delay, req.unit, req.addr, req.value)
    except Exception as e:
        raise HTTPException(status_code=502, detail=str(e))
    return {"unit": req.unit, "port": req.port, "addr": req.addr, "value": req.value}

@app.on_event("shutdown")
def shutdown_event():
    serial_manager.close()
