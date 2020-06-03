#include "ina219.h"



nBlock_INA219::nBlock_INA219(PinName pinSDA, PinName pinSCL, int address, float shuntResistor):
	_i2c(pinSDA, pinSCL) {
		
	_i2c.frequency(400000);
		
	// Store the I2C address
	_address = address;
	// Store the shunt resistor value (zero not allowed, defaults to 0.1 ohm)
    _Rshunt = (shuntResistor != 0.0)? shuntResistor : 0.1;
	// Calculates the current LSB
	float max_current = 0.32 / shuntResistor; // assuming 320mV maximum shunt voltage
	_currentLSB = max_current / 32768;        // Maximum current over the signed 16 bit range
	// Calculates the power LSB
	_powerLSB = 20.0 * _currentLSB;           // Power LSB always 20 * Current LSB (per datasheet)
	
	// Reset the flag, we are not requesting a read yet
	_flagReadRequested = 0;
	
	// Configure the sensor
	char buf[3]; // Used in I2C write transactions: [reg. number, MSB, LSB]
	
	// Calibrates the device for a shunt voltage range of +- 320 mV
	// optimized over the 16-bit register range
	//
	// Maximum Shunt Voltage Register = 320mV * 100 = 32000
	//
	// Calibration = (Signed 16-bit end of scale * 4096) / Maximum Shunt Voltage Register
	//             = (32768 * 4096) / 32000
	//             = 4194.304 = 4194
	// Calibration value must be shifted one bit to the left as per datasheet
	i2cWrite(__INA219_CALIBRATION, (4194 << 1));
	
	
	
	// Make sure we have the default config:
	// 32V VBus input range
	// +- 320mV VShunt range
	// 12bit ADC 532 us
	// Continuous reading shunt and bus voltages

	// Configures the device. This also starts a first measurement
	i2cWrite(__INA219_CONFIG, __INA219_DEFAULT_CONFIG);
	
}

void nBlock_INA219::triggerInput(uint32_t inputNumber, uint32_t value) {
	// inputNumber is ignored as we only have one input
	// value is ignored as any value is considered a trigger for reading

	// If we received ANY input, means we should read the sensor
	_flagReadRequested = 1;

}

void nBlock_INA219::endFrame(void) {
	// If a read has been requested
	if (_flagReadRequested) {
		// Clear the flag
		_flagReadRequested = 0;
		
		// === Perform the I2C reads
		// VBus register
		uint16_t vbus_reg = i2cRead(__INA219_V_BUS) & 0xFFFF;
		int flag_OVF = vbus_reg & 0x01;
		int flag_CNVR = (vbus_reg & 0x02) >> 1;
		float vbus_base = (vbus_reg >> 3) & 0xFFFF; // casts to float to be multiplied later
		
		// Current register
		float current = i2cRead(__INA219_CURRENT) & 0xFFFF; // current is *signed* int
		
		// Power register
		float power = i2cRead(__INA219_POWER) & 0xFFFF; // power is unsigned

        output[0] = (vbus_base * 0.004)*1000;
		available[0] = 1;
		
        output[1] = (current * _currentLSB)*1000; 
		available[1] = 1;
		
        output[2] = (power * _powerLSB)*1000; 
		available[2] = 1;
		
	}
    return;
}


void nBlock_INA219::i2cWrite(uint8_t regAddress, uint16_t value) {
	char buf[3]; // [reg. number, MSB, LSB]
	
	buf[0] = regAddress;
	buf[1] = (value >> 8) & 0xFF;
	buf[2] = (value     ) & 0xFF;
	
	_i2c.write(_address, buf, 3);
}

uint16_t nBlock_INA219::i2cRead(uint8_t regAddress) {
	char buf[2]; // [reg. number, unused] or [MSB, LSB]

	// Sets the register address
	buf[0] = regAddress;
	buf[1] = 0xFF;// dummy
	
	int res = _i2c.write(_address, buf, 1);
	
	// Clears the buffer and reads the value
	buf[0] = 0;
	buf[1] = 0;
	res = _i2c.read(_address, buf, 2);
	
	// Variable to be returned
	// Cast to uint16_t is important otherwise the value disappears on overflow
	uint16_t result = (((uint16_t)buf[0]) << 8) | buf[1];
	
	return result;
	
}
