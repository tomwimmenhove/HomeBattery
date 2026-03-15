#include <Wire.h>
#include <ModbusSerial.h>

#define SERIAL_BAUD 115200
#define ADJUST_INTERVAL_MS 1000

// PMBus command codes (standard)
const uint8_t CMD_VOUT_MODE     	= 0x20;
const uint8_t CMD_VOUT_COMMAND  	= 0x21;
const uint8_t CMD_READ_VOUT     	= 0x8B;
const uint8_t CMD_READ_IOUT     	= 0x8C;
const uint8_t CMD_CLEAR_FAULTS		= 0x03;
const uint8_t CMD_READ_TEMPERATURE1	= 0x8d;
const uint8_t CMD_READ_TEMPERATURE2	= 0x8e;
const uint8_t CMD_READ_TEMPERATURE3	= 0x8f;
const uint8_t CMD_READ_VIN		= 0x88;

const uint8_t CMD_STATUS_WORD		= 0x79;
const uint8_t CMD_STATUS_VOUT		= 0x7a;
const uint8_t CMD_STATUS_IOUT		= 0x7b;
const uint8_t CMD_STATUS_INPUT		= 0x7c;
const uint8_t CMD_STATUS_TEMP		= 0x7d;
const uint8_t CMD_STATUS_CML		= 0x7e;

// Allowed PMBus 7-bit addresses for LCC600 (datasheet): 0x58..0x5F
const uint8_t ADDR_MIN = 0x58;
const uint8_t ADDR_MAX = 0x5F;

unsigned long lastAdjust = 0;
unsigned long lastReport = 0;

// These are repeated for individual PSUs at 0x100, 0x200, ...
#define MB_IREG_TEMPERATURE1 0x0000
#define MB_IREG_TEMPERATURE2 0x0001
#define MB_IREG_TEMPERATURE3 0x0002
#define MB_IREG_V_OUT 0x0003
#define MB_IREG_I_OUT 0x0004
#define MB_IREG_V_IN 0x0005
#define MB_IREG_STATUS_WORD 0x0006
#define MB_IREG_STATUS_V_OUT 0x0007
#define MB_IREG_STATUS_I_OUT 0x0008
#define MB_IREG_STATUS_INPUT 0x0009
#define MB_IREG_STATUS_TEMP 0x000a
#define MB_IREG_STATUS_CML 0x000b
#define MB_IREG_TRIPPED 0x000c

#define MB_IREG_NUM_PSU 0x000f

#define MB_HREG_STEP_VOLTAGE 0x0000
#define MB_HREG_MIN_VOLTAGE 0x0001
#define MB_HREG_SET_VOLTAGE_LOW 0x0002
#define MB_HREG_SET_VOLTAGE_HIGH 0x0003
#define MB_HREG_SET_CURRENT_LOW 0x0004
#define MB_HREG_SET_CURRENT_HIGH 0x0005
#define MB_HREG_SET_POWER_LOW 0x0006
#define MB_HREG_SET_POWER_HIGH 0x0007
#define MB_HREG_OVER_VOLTAGE_TRIP 0x0008

#define MB_COIL_POWER 1
#define MB_COIL_CLEAR_FAULTS 2

/*
String makeNMEA(String talker, String type, String data) {
	String s = "$" + talker + type + "," + data;
	uint8_t chk = 0;
	for (uint8_t i = 1; i < s.length(); i++) chk ^= s[i];
	if (chk < 16) s += "*0" + String(chk, HEX);
	else s += "*" + String(chk, HEX);
	s += "\r\n";
	return s;
}*/

class Linear16 {
public:
	Linear16() : _word(0) {}
	Linear16(float value) { setAsFloat(value); }
	Linear16(uint16_t value) { setAsWord(value); }

	uint16_t getAsWord() const { return _word; }
	void setAsWord(uint16_t w) { _word = w; }

	float getAsFloat() const {
		int16_t mant = (int16_t)_word; // reinterpret two's complement bits
		return (float)mant / 512.0f;
	}

	// returns false when value is out of representable range
	bool setAsFloat(float value) {
		// manual rounding: mant = round(value * 512)
		float scaled = value * 512.0f;
		// avoid roundf dependency
		int32_t mant_i = (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);

		if (mant_i < -32768 || mant_i > 32767) return false;

		int16_t mant = (int16_t)mant_i;
		_word = (uint16_t)mant; // two's-complement bit pattern stored in uint16_t
		return true;
	}

private:
	uint16_t _word;
};


// Linear11: 11-bit two's-complement mantissa (bits 0..10), 5-bit two's-complement exponent (bits 11..15).
// value = mantissa * 2^exponent
class Linear11 {
public:
	Linear11() : _word(0) {}
	Linear11(float value) { setAsFloat(value); }
	Linear11(uint16_t value) { setAsWord(value); }

	uint16_t getAsWord() const { return _word; }
	void setAsWord(uint16_t w) { _word = w; }

	float getAsFloat() const
       	{
		int16_t tmp = (int16_t)_word;                // keep bit pattern
		int16_t mantissa = tmp & 0x07FF;            // lower 11 bits
		if (mantissa & 0x0400) mantissa |= 0xF800;  // sign-extend 11->16 bits

		int8_t exp = (tmp >> 11) & 0x1F;            // top 5 bits
		if (exp & 0x10) exp |= 0xE0;                // sign-extend 5->8 bits

		// compute 2^exp using integer shifts (exp in [-16..15])
		float pow2;
		if (exp >= 0) {
			pow2 = (float)(1u << exp);
		} else {
			pow2 = 1.0f / (float)(1u << (-exp));
		}

		return (float)mantissa * pow2;
	}

	bool setAsFloat(float value)
	{
		// reject NaN / Inf
		if (isnan(value) || isinf(value)) return false;

		// zero
		if (value == 0.0f) {
			_word = 0;
			return true;
		}

		// decompose value: value = m * 2^exp, with |m| in [0.5, 1)
		int exp;
		float m = frexpf(value, &exp); // m in [-1..-0.5) U (0.5..1)

		// scale fractional mantissa to an 11-bit signed integer with 10 fractional bits:
		// mant_int = round(m * 2^10)
		const int SHIFT = 10;             // number of fractional bits
		const int MANT_MAX_POS =  1023;   // max positive mantissa
		const int MANT_MIN_NEG = -1024;   // min negative mantissa (two's complement fits -1024..1023)

		float scaled = m * (1u << SHIFT);   // 1024.0f
		int32_t mant = (int32_t)roundf(scaled);

		// if rounding produced a positive overflow (== 1024) we must renormalize:
		// divide mantissa by two and increase exponent so value is preserved.
		if (mant > MANT_MAX_POS) { // this only happens when mant == 1024
			mant >>= 1;            // 1024 -> 512
			exp += 1;
		}

		// compute the exponent field that will be stored (exp_field = exp - SHIFT)
		int32_t exp_field = exp - SHIFT;

		// exponent must fit into signed 5-bit range [-16..15]
		if (exp_field < -16) {
			// underflow: value too small to represent — store zero (alternative: return false)
			_word = 0;
			return true;
		}
		if (exp_field > 15) {
			// overflow: magnitude too large
			return false;
		}

		// pack: top 5 bits = exp_field (two's complement 5-bit), low 11 bits = mantissa (two's complement 11-bit)
		uint16_t ebits = (uint16_t)(exp_field & 0x1F);
		uint16_t mbits = (uint16_t)((uint32_t)mant & 0x07FFu);

		_word = (uint16_t)((ebits << 11) | mbits);
		return true;
	}

private:
	uint16_t _word;
};

struct PsuLimits
{
	Linear16 stepVoltage;
	Linear16 minVoltage;

	Linear16 setVoltageLow;
	Linear16 setVoltageHigh;

	Linear11 setCurrentLow;
	Linear11 setCurrentHigh;

	uint16_t setPowerLow;
	uint16_t setPowerHigh;

	Linear16 overVoltageTrip;
};

PsuLimits limits =
{
	.stepVoltage = Linear16(0.01f),
	.minVoltage = Linear16(24.0f),

	.setVoltageLow = Linear16(28.75f),
	.setVoltageHigh = Linear16(28.8f),

	.setCurrentLow = Linear11(24.0f),
	.setCurrentHigh = Linear11(25.0f),

	.setPowerLow = 590,
	.setPowerHigh = 600,

	.overVoltageTrip = Linear16(29.2f),
};

class PMBus
{
public:
	// ----- PMBus / I2C primitives -----
	static uint8_t readByte(uint8_t addr, uint8_t command)
	{
		uint8_t out;
		readByte(addr, command, out);
		return out;
	}

	static bool readByte(uint8_t addr, uint8_t command, uint8_t &out)
	{
		Wire.beginTransmission(addr);
		Wire.write(command);
		if (Wire.endTransmission(false) != 0) return false;
		Wire.requestFrom((int)addr, 1);
		if (Wire.available() < 1) return false;
		out = Wire.read();
	
		delay(1);
	
		return true;
	}
	
	static uint16_t readWord(uint8_t addr, uint8_t command)
	{
		uint16_t out;
		readWord(addr, command, out);
		return out;
	}

	static bool readWord(uint8_t addr, uint8_t command, uint16_t &out)
	{
		Wire.beginTransmission(addr);
		Wire.write(command);
		if (Wire.endTransmission(false) != 0) return false;
		uint8_t toRequest = 2;
		Wire.requestFrom((int)addr, (int)toRequest);
		if (Wire.available() < 2) return false;
		uint8_t lsb = Wire.read();
		uint8_t msb = Wire.read();
		out = ((uint16_t)msb << 8) | lsb;
	
		delay(1);
	
		return true;
	}
	
	static bool writeByte(uint8_t addr, uint8_t command, uint8_t data)
	{
		Wire.beginTransmission(addr);
		Wire.write(command);
		Wire.write(data);
		bool success = Wire.endTransmission() == 0;
	
		delay(1);
	
		return success;
	}
	
	static bool writeWord(uint8_t addr, uint8_t command, uint16_t data)
	{
		Wire.beginTransmission(addr);
		Wire.write(command);
		Wire.write((uint8_t)(data & 0xFF));
		Wire.write((uint8_t)((data >> 8) & 0xFF));
		bool success = Wire.endTransmission() == 0;
		
		delay(1);
		
		return success;
	}
	
	// read/write floats
	static float readLinear11WordAsFloat(uint8_t addr, uint8_t cmd)
	{
		uint16_t w;
		if (!readWord(addr, cmd, w)) return NAN;
		return linear11ToFloat(w);
	}
	
	static float readLinear16WordAsFloat(uint8_t addr, uint8_t cmd)
	{
		uint16_t w;
		if (!readWord(addr, cmd, w)) return NAN;	
		return linear16ToFloat(w);
	}
	
	static bool writeFloatAsLinear16(uint8_t addr, uint8_t cmd, float value)
	{
		uint16_t w;
		if (!floatToLinear16Word(value, w)) return false;
		return writeWord(addr, cmd, w);
	}

private:	
	// decode/encode
	static float linear11ToFloat(uint16_t raw)
	{
		return Linear11(raw).getAsFloat();
/*		int16_t tmp = (int16_t)raw;
		int16_t mantissa = tmp & 0x07FF;
		if (mantissa & 0x0400) mantissa |= 0xF800;
		int8_t exp = (tmp >> 11) & 0x1F;
		if (exp & 0x10) exp |= 0xE0;
		float val = (float)mantissa * pow(2.0, (float)exp);
		return val;*/
	}
	
	static float linear16ToFloat(uint16_t raw)
	{
		return Linear16(raw).getAsFloat();
/*		int16_t mant = (int16_t)word; // interpret bits as signed
		return (float)mant / 512.0f;*/
	}
	
	static bool floatToLinear16Word(float value, uint16_t &wordOut)
	{
		Linear16 linear;

		if (!linear.setAsFloat(value))
		{
			return false;
		}

		wordOut = linear.getAsWord();

		return true;
/*		float mant_f = roundf(value * 512.0f);
	
		// Linear16 mantissa range is signed 16-bit
		if (mant_f < -32768.0f || mant_f > 32767.0f) return false;
	
		int16_t mant = (int16_t)mant_f;
		wordOut = (uint16_t)mant; // two's-complement bit pattern
		return true;*/
	}
};

struct PsuStatus
{
	Linear11 temperature1;
	Linear11 temperature2;
	Linear11 temperature3;

	Linear16 vOut;
	Linear11 iOut;
	Linear11 vIn;

	uint16_t statusWord;
	uint8_t statusVOut;
	uint8_t statusIOut;
	uint8_t statusInput;
	uint8_t statusTemp;
	uint8_t statusCml;

	bool tripped;

	void acc(const struct PsuStatus& other)
	{
		if (other.temperature1.getAsFloat() > temperature1.getAsFloat())
		{
			temperature1 = other.temperature1;
		}

		if (other.temperature2.getAsFloat() > temperature2.getAsFloat())
		{
			temperature2 = other.temperature2;
		}

		if (other.temperature3.getAsFloat() > temperature3.getAsFloat())
		{
			temperature3 = other.temperature3;
		}

		vOut.setAsFloat(vOut.getAsFloat() + other.vOut.getAsFloat());
		iOut.setAsFloat(iOut.getAsFloat() + other.iOut.getAsFloat());
		vIn.setAsFloat(vIn.getAsFloat() + other.vIn.getAsFloat());

		statusWord |= other.statusWord;
		statusVOut |= other.statusVOut;
		statusIOut |= other.statusIOut;
		statusInput |= other.statusInput;
		statusTemp |= other.statusTemp;
		statusCml |= other.statusCml;
		tripped |= other.tripped;
	}

	void avg(int n)
	{
		vOut.setAsFloat(vOut.getAsFloat() / n);
		vIn.setAsFloat(vIn.getAsFloat() / n);
	}
};

class Psu
{
public:
	Psu() { }

	Psu(uint8_t addr)
	{
		_addr = addr;
	}

	void setAddr(uint8_t addr) { _addr = addr; }
	uint8_t getAddr() { return _addr; }

	static bool detect(uint8_t addr)
	{
		Wire.beginTransmission(addr);
		uint8_t err = Wire.endTransmission();

		return err == 0;
	}

	float getVOut() { return PMBus::readLinear16WordAsFloat(_addr, CMD_VOUT_COMMAND); }
	
	float getSetpoint() { return _setpoint; }
	void setSetpoint(float setpoint) { _setpoint = setpoint; }

	void useInstantaniousVoltage() { _setpoint = PMBus::readLinear16WordAsFloat(_addr, CMD_READ_VOUT); }

	void resetSetpoint() { _setpoint = 0.0f; };

	const struct PsuStatus& getStatus() { return _status; }

	void update(const PsuLimits& limits)
	{
		updateStatus(limits);

		if (_status.tripped)
		{
			return;
		}

		float pOut = _status.vOut.getAsFloat() * _status.iOut.getAsFloat();
		float oldSetpoint = _setpoint;

		if (_status.vOut.getAsFloat() > limits.setVoltageHigh.getAsFloat() ||
		    _status.iOut.getAsFloat() > limits.setCurrentHigh.getAsFloat() ||
		    pOut > limits.setPowerHigh)
		{
			_setpoint -= limits.stepVoltage.getAsFloat();
		}
		else if (_status.vOut.getAsFloat() < limits.setVoltageLow.getAsFloat() &&
			 _status.iOut.getAsFloat() < limits.setCurrentLow.getAsFloat() &&
			 pOut < limits.setPowerLow)
		{
			_setpoint += limits.stepVoltage.getAsFloat();
		}

		// If the current setpoint is below the current output voltage,
		// while the target voltage is above it, we can safely increase
		// the setpoint to the current output voltage.
//		if (_setpoint < _status.vOut.getAsFloat() &&
//		    limits.setVoltageLow.getAsFloat() > _status.vOut.getAsFloat())
//		{
//			_setpoint = _status.vOut.getAsFloat();
//		}

		// Clip setpoint
		if (_setpoint < limits.minVoltage.getAsFloat())
		{
			_setpoint = limits.minVoltage.getAsFloat();
		}

		if (_setpoint > limits.setVoltageHigh.getAsFloat())
		{
			_setpoint = limits.setVoltageHigh.getAsFloat();
		}

		if (oldSetpoint != _setpoint)
		{
			setVOut(_setpoint);
		}
	}

	bool enablePower(bool enable) { return PMBus::writeByte(_addr, 0x01, enable ? 0x80 : 0x40); }

	bool enableWrites(bool enable) { return PMBus::writeByte(_addr, 0x10, enable ? 0x00 : 0x80); }

	bool clearFaults()
	{
		Wire.beginTransmission(_addr);
		Wire.write(CMD_CLEAR_FAULTS);
		bool success = Wire.endTransmission();

		delay(100); // XXX: Too long? I dunno...

		return success;
	}

private:
	uint8_t _addr;
	struct PsuStatus _status;
	float _setpoint = 0.0f;

	bool setVOut(float voltage) { return PMBus::writeFloatAsLinear16(_addr, CMD_VOUT_COMMAND, voltage); }

	updateStatus(const PsuLimits& limits)
	{
		_status.vOut = PMBus::readLinear16WordAsFloat(_addr, CMD_READ_VOUT);

		if (_status.vOut.getAsFloat() >= limits.overVoltageTrip.getAsFloat())
		{
			enablePower(false);
			_status.tripped = true;
		}

		PMBus::readWord(_addr, CMD_STATUS_WORD, _status.statusWord);
		PMBus::readByte(_addr, CMD_STATUS_VOUT, _status.statusVOut);
		PMBus::readByte(_addr, CMD_STATUS_IOUT, _status.statusIOut);
		PMBus::readByte(_addr, CMD_STATUS_INPUT, _status.statusInput);
		PMBus::readByte(_addr, CMD_STATUS_TEMP, _status.statusTemp);
		PMBus::readByte(_addr, CMD_STATUS_CML, _status.statusCml);

		_status.iOut.setAsWord(PMBus::readWord(_addr, CMD_READ_IOUT));
		_status.temperature1.setAsWord(PMBus::readWord(_addr, CMD_READ_TEMPERATURE1));
		_status.temperature2.setAsWord(PMBus::readWord(_addr, CMD_READ_TEMPERATURE2));
		_status.temperature3.setAsWord(PMBus::readWord(_addr, CMD_READ_TEMPERATURE3));

		_status.vIn.setAsWord(PMBus::readWord(_addr, CMD_READ_VIN));
	}
};

Psu psus[ADDR_MAX - ADDR_MIN];
uint8_t detectedCount = 0;

bool power = false;

ModbusSerial mb(Serial, 11);

void addIregs(uint16_t offset, PsuStatus& status)
{
	mb.addIreg(offset + MB_IREG_TEMPERATURE1, status.temperature1.getAsWord());
	mb.addIreg(offset + MB_IREG_TEMPERATURE2, status.temperature2.getAsWord());
	mb.addIreg(offset + MB_IREG_TEMPERATURE3, status.temperature3.getAsWord());
	mb.addIreg(offset + MB_IREG_V_OUT, status.vOut.getAsWord());
	mb.addIreg(offset + MB_IREG_I_OUT, status.iOut.getAsWord());
	mb.addIreg(offset + MB_IREG_V_IN,  status.vIn.getAsWord());
	mb.addIreg(offset + MB_IREG_STATUS_WORD, status.statusWord);
	mb.addIreg(offset + MB_IREG_STATUS_V_OUT, status.statusVOut);
	mb.addIreg(offset + MB_IREG_STATUS_I_OUT, status.statusIOut);
	mb.addIreg(offset + MB_IREG_STATUS_INPUT, status.statusInput);
	mb.addIreg(offset + MB_IREG_STATUS_TEMP, status.statusTemp);
	mb.addIreg(offset + MB_IREG_STATUS_CML, status.statusCml);
	mb.addIreg(offset + MB_IREG_TRIPPED, (uint8_t)status.tripped);
}

void setIregs(uint16_t offset, PsuStatus& status)
{
	mb.setIreg(offset + MB_IREG_TEMPERATURE1, status.temperature1.getAsWord());
	mb.setIreg(offset + MB_IREG_TEMPERATURE2, status.temperature2.getAsWord());
	mb.setIreg(offset + MB_IREG_TEMPERATURE3, status.temperature3.getAsWord());
	mb.setIreg(offset + MB_IREG_V_OUT, status.vOut.getAsWord());
	mb.setIreg(offset + MB_IREG_I_OUT, status.iOut.getAsWord());
	mb.setIreg(offset + MB_IREG_V_IN, status.vIn.getAsWord());
	mb.setIreg(offset + MB_IREG_STATUS_WORD, status.statusWord);
	mb.setIreg(offset + MB_IREG_STATUS_V_OUT, status.statusVOut);
	mb.setIreg(offset + MB_IREG_STATUS_I_OUT, status.statusIOut);
	mb.setIreg(offset + MB_IREG_STATUS_INPUT, status.statusInput);
	mb.setIreg(offset + MB_IREG_STATUS_TEMP, status.statusTemp);
	mb.setIreg(offset + MB_IREG_STATUS_CML, status.statusCml);
	mb.setIreg(offset + MB_IREG_TRIPPED, (uint8_t)status.tripped);
}

PsuStatus statusAcc;

void setup()
{
	Serial.begin(SERIAL_BAUD, MB_PARITY_EVEN);
	Wire.begin();
	//while (!Serial) { }

	mb.config(SERIAL_BAUD);
	mb.setAdditionalServerData("PSU"); // for Report Server ID function (0x11)

	mb.addCoil(MB_COIL_POWER, power);
	mb.addCoil(MB_COIL_CLEAR_FAULTS, false);
	detectPSUs();
	enablePower(false);
	charge();

	mb.addIreg(MB_IREG_NUM_PSU, detectedCount);

	mb.addHreg(MB_HREG_STEP_VOLTAGE, limits.stepVoltage.getAsWord());
	mb.addHreg(MB_HREG_MIN_VOLTAGE, limits.minVoltage.getAsWord());
	mb.addHreg(MB_HREG_SET_VOLTAGE_LOW, limits.setVoltageLow.getAsWord());
	mb.addHreg(MB_HREG_SET_VOLTAGE_HIGH, limits.setVoltageHigh.getAsWord());
	mb.addHreg(MB_HREG_SET_CURRENT_LOW, limits.setCurrentLow.getAsWord());
	mb.addHreg(MB_HREG_SET_CURRENT_HIGH, limits.setCurrentHigh.getAsWord());
	mb.addHreg(MB_HREG_SET_POWER_LOW, limits.setPowerLow);
	mb.addHreg(MB_HREG_SET_POWER_HIGH, limits.setPowerHigh);
	mb.addHreg(MB_HREG_OVER_VOLTAGE_TRIP, limits.overVoltageTrip.getAsWord());

	memset(&statusAcc, 0, sizeof(PsuStatus));

	for (uint8_t i=0;i<detectedCount;i++)
	{
		PsuStatus status = psus[i].getStatus();

		statusAcc.acc(status);

		uint16_t offset = (i + 1) * 0x100;
		addIregs(offset, status);
	}

	statusAcc.avg(detectedCount);
	addIregs(0, statusAcc);
}

void loop()
{
	mb.task();

	if (mb.Coil(MB_COIL_POWER) != power)
       	{
		power = mb.Coil(MB_COIL_POWER);
		enablePower(power);
		if (!power)
		{
			for (uint8_t i=0;i<detectedCount;i++)
			{
				psus[i].resetSetpoint();
			}
		}
	}

	if (mb.Coil(MB_COIL_CLEAR_FAULTS))
       	{
		clearFaults();
		mb.setCoil(MB_COIL_CLEAR_FAULTS, 0);
	}

	limits.stepVoltage.setAsWord(mb.Hreg(MB_HREG_STEP_VOLTAGE));
	limits.minVoltage.setAsWord(mb.Hreg(MB_HREG_MIN_VOLTAGE));
	limits.setVoltageLow.setAsWord(mb.Hreg(MB_HREG_SET_VOLTAGE_LOW));
	limits.setVoltageHigh.setAsWord(mb.Hreg(MB_HREG_SET_VOLTAGE_HIGH));
	limits.setCurrentLow.setAsWord(mb.Hreg(MB_HREG_SET_CURRENT_LOW));
	limits.setCurrentHigh.setAsWord(mb.Hreg(MB_HREG_SET_CURRENT_HIGH));
	limits.setPowerLow = mb.Hreg(MB_HREG_SET_POWER_LOW);
	limits.setPowerHigh = mb.Hreg(MB_HREG_SET_POWER_HIGH);
	limits.overVoltageTrip.setAsWord(mb.Hreg(MB_HREG_OVER_VOLTAGE_TRIP));

	memset(&statusAcc, 0, sizeof(PsuStatus));

	for (uint8_t i=0;i<detectedCount;i++)
	{
		PsuStatus status = psus[i].getStatus();

		statusAcc.acc(status);

		uint16_t offset = (i + 1) * 0x100;
		setIregs(offset, status);
	}

	statusAcc.avg(detectedCount);
	setIregs(0, statusAcc);

	unsigned long now = millis();
	if (now - lastAdjust >= ADJUST_INTERVAL_MS) {
		lastAdjust = now;
		charge();
	}
}

// ----- Auto-detect PSUs on the I2C bus -----
// Scans 7-bit addresses 0x58..0x5F (LCC600 address range per datasheet)
void detectPSUs()
{
	detectedCount = 0;
	for (uint8_t addr = ADDR_MIN; addr <= ADDR_MAX; ++addr)
	{
		//Wire.beginTransmission(addr);
		//uint8_t err = Wire.endTransmission();
		//if (err == 0)
		if (Psu::detect(addr))
		{
			psus[detectedCount].setAddr(addr);
			psus[detectedCount].enableWrites(true);
			detectedCount++;
		}
	}
}

void enablePower(bool enable)
{
	for (uint8_t i=0;i<detectedCount;i++) {
		psus[i].enablePower(enable);
	}
}

void clearFaults()
{
	for (uint8_t i=0;i<detectedCount;i++) {
		psus[i].clearFaults();
	}
}

void charge()
{
	for (uint8_t i=0;i<detectedCount;i++)
       	{
		psus[i].update(limits);
	}
}

void setMinVoltage()
{
	for (uint8_t i=0;i<detectedCount;i++)
       	{
		psus[i].useInstantaniousVoltage();
	}
}

