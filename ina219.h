#ifndef __NB_INA219
#define __NB_INA219

#include "mbed.h"
#include "nworkbench.h"

// INA219 constants

#define __INA219_DEFAULT_CONFIG   0x399F

#define __INA219_CONFIG       0x00
#define __INA219_V_SHUNT      0x01
#define __INA219_V_BUS        0x02
#define __INA219_POWER        0x03
#define __INA219_CURRENT      0x04
#define __INA219_CALIBRATION  0x05


/**
 *  \brief Driver to configure and read data from INA219 current/power
 *  sensors. It has a trigger input and 3 outputs: bus voltage,
 *  current and power.
 *  The trigger is used to synchronise the output. It is not related
 *  to INA219's triggered mechanisms. This library keeps the INA219
 *  in continuous mode.
 */
class nBlock_INA219: public nBlockSimpleNode<3> {
public:
    /**
     *  \brief Constructor is invoked at node creation, and arguments
     *  are configuration parameters assigned in the IDE
     *  
     *  \param [in] pinSDA Pin connected to the sensor's SDA line
     *  \param [in] pinSCL Pin connected to the sensor's SCL line
     *  \param [in] pinSCL Pin connected to the sensor's SCL line
     *  \param [in] shuntResistor Resistance of the shunt resistor, in ohms
     *  
     */
    nBlock_INA219(PinName pinSDA, PinName pinSCL, int address, float shuntResistor);
	
    /**
     *  \brief Called whenever a data packet is sent to this node in one
     *  of the inputs. In this case, sets a flag indicating a read must be 
     *  performed in the end of this frame, so data is available in the next.
     *  Does not engage in bus communication yet.
     *  
     *  \param [in] inputNumber Number identifying the input. This node has only one.
     *  \param [in] value The value sent to this node. In this node, any value triggers.
     */
    void triggerInput(uint32_t inputNumber, uint32_t value);
	
    /**
     *  \brief Called in the end of every frame.
     *  In this case, reads the sensor if a read was requested and prepares
     *  data in the output for the next frame.
     */
    void endFrame(void);
	
	/**
	 *  \brief Writes a 16-bit value into a register over I2C
	 *  
	 *  \param [in] regAddress Register address (range 0x00 - 0x05)
	 *  \param [in] value Value to be written
	 */
	void i2cWrite(uint8_t regAddress, uint16_t value);
	
	/**
	 *  \brief Reads a 16-bit register over I2C
	 *  
	 *  \param [in] regAddress Register address (range 0x00 - 0x05)
	 *  \return Register value as 16-bit integer
	 *  
	 */
	uint16_t i2cRead(uint8_t regAddress);
	
	
private:
	// I2C Object
	I2C _i2c;
	// I2C Address
	int _address;
	// Shunt resistance
    float _Rshunt;
	// Current LSB used to convert raw reading to amperes
	float _currentLSB;
	// Power LSB used to convert raw reading to watts
	float _powerLSB;
	// Flag indicating a read was requested
	int _flagReadRequested;
	
};

#endif
