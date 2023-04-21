#ifndef RoboClaw_h
#define RoboClaw_h

#include <stdarg.h>

#include <inttypes.h>
#include <Stream.h>
#include <HardwareSerial.h>
#ifdef __AVR__
	#include <SoftwareSerial.h>
#endif

/******************************************************************************
* Definitions
******************************************************************************/

#define _RC_VERSION 10 // software version of this library
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#define _SS_VERSION 16

class RoboClaw : public Stream
{
	uint16_t crc;
	uint32_t timeout;
	
	HardwareSerial *hserial;
#ifdef __AVR__
	SoftwareSerial *sserial;
#endif
	
	enum {
		ERROR_NONE			= 0x000000,
		ERROR_ESTOP			= 0x000001,	//Error: E-Stop active
		ERROR_TEMP			= 0x000002,	//Error: Temperature Sensor 1 >=100c
		ERROR_TEMP2			= 0x000004,	//Error: Temperature Sensor 2 >=100C (available only on some models)
		ERROR_MBATHIGH		= 0x000008,	//Error: Main Battery Over Voltage
		ERROR_LBATHIGH		= 0x000010,	//Error: Logic Battery High Voltage
		ERROR_LBATLOW		= 0x000020,	//Error: Logic Battery Low Voltage
		ERROR_FAULTM1		= 0x000040,	//Error: Motor 1 Driver Fault (only on some models)
		ERROR_FAULTM2		= 0x000080,	//Error: Motor 2 Driver Fault (only on some models)
		ERROR_SPEED1		= 0x000100,	//Error: Motor 1 Speed Error Limit
		ERROR_SPEED2		= 0x000200,	//Error: Motor 2 Speed Error Limit
		ERROR_POS1			= 0x000400,	//Error: Motor 1 Position Error Limit
		ERROR_POS2			= 0x000800,	//Error: MOtor2 Position Error Limit
		WARN_OVERCURRENTM1	= 0x010000, //Warning: Motor 1 Current Limited
		WARN_OVERCURRENTM2	= 0x020000, //Warning: Motor 2 CUrrent Limited
		WARN_MBATHIGH		= 0x040000, //Warning: Main Battery Voltage High
		WARN_MBATLOW		= 0x080000, //Warning: Main Battery Low Voltage
		WARN_TEMP			= 0x100000, //Warning: Temperaure Sensor 1 >=85C
		WARN_TEMP2			= 0x200000, //Warning: Temperature Sensor 2 >=85C (available only on some models)
		WARN_S4				= 0x400000, //Warning: Motor 1 Home/Limit Signal
		WARN_S5				= 0x800000, //Warning: Motor 2 Home/Limit Signal
	};
	
	enum {M1FORWARD = 0,
			M1BACKWARD = 1,
			SETMINMB = 2,
			SETMAXMB = 3,
			M2FORWARD = 4,
			M2BACKWARD = 5,
			M17BIT = 6,
			M27BIT = 7,
			MIXEDFORWARD = 8,
			MIXEDBACKWARD = 9,
			MIXEDRIGHT = 10,
			MIXEDLEFT = 11,
			MIXEDFB = 12,
			MIXEDLR = 13,
			GETM1ENC = 16,
			GETM2ENC = 17,
			GETM1SPEED = 18,
			GETM2SPEED = 19,
			RESETENC = 20,
			GETVERSION = 21,
			SETM1ENCCOUNT = 22,
			SETM2ENCCOUNT = 23,
			GETMBATT = 24,
			GETLBATT = 25,
			SETMINLB = 26,
			SETMAXLB = 27,
			SETM1PID = 28,
			SETM2PID = 29,
			GETM1ISPEED = 30,
			GETM2ISPEED = 31,
			M1DUTY = 32,
			M2DUTY = 33,
			MIXEDDUTY = 34,
			M1SPEED = 35,
			M2SPEED = 36,
			MIXEDSPEED = 37,
			M1SPEEDACCEL = 38,
			M2SPEEDACCEL = 39,
			MIXEDSPEEDACCEL = 40,
			M1SPEEDDIST = 41,
			M2SPEEDDIST = 42,
			MIXEDSPEEDDIST = 43,
			M1SPEEDACCELDIST = 44,
			M2SPEEDACCELDIST = 45,
			MIXEDSPEEDACCELDIST = 46,
			GETBUFFERS = 47,
			GETPWMS = 48,
			GETCURRENTS = 49,
			MIXEDSPEED2ACCEL = 50,
			MIXEDSPEED2ACCELDIST = 51,
			M1DUTYACCEL = 52,
			M2DUTYACCEL = 53,
			MIXEDDUTYACCEL = 54,
			READM1PID = 55,
			READM2PID = 56,
			SETMAINVOLTAGES = 57,
			SETLOGICVOLTAGES = 58,
			GETMINMAXMAINVOLTAGES = 59,
			GETMINMAXLOGICVOLTAGES = 60,
			SETM1POSPID = 61,
			SETM2POSPID = 62,
			READM1POSPID = 63,
			READM2POSPID = 64,
			M1SPEEDACCELDECCELPOS = 65,
			M2SPEEDACCELDECCELPOS = 66,
			MIXEDSPEEDACCELDECCELPOS = 67,
			SETM1DEFAULTACCEL = 68,
			SETM2DEFAULTACCEL = 69,
			SETPINFUNCTIONS = 74,
			GETPINFUNCTIONS = 75,
			SETDEADBAND	= 76,
			GETDEADBAND	= 77,
			GETENCODERS = 78,
			GETISPEEDS = 79,
			RESTOREDEFAULTS = 80,
			GETTEMP = 82,
			GETTEMP2 = 83,	//Only valid on some models
			GETERROR = 90,
			GETENCODERMODE = 91,
			SETM1ENCODERMODE = 92,
			SETM2ENCODERMODE = 93,
			WRITENVM = 94,
			READNVM = 95,	//Reloads values from Flash into Ram
			SETCONFIG = 98,
			GETCONFIG = 99,
			SETM1MAXCURRENT = 133,
			SETM2MAXCURRENT = 134,
			GETM1MAXCURRENT = 135,
			GETM2MAXCURRENT = 136,
			SETPWMMODE = 148,
			GETPWMMODE = 149,
			FLAGBOOTLOADER = 255};	//Only available via USB communications
public:
	// public methods
	RoboClaw(HardwareSerial *hserial,uint32_t tout);
#ifdef __AVR__
	RoboClaw(SoftwareSerial *sserial,uint32_t tout);
#endif
	
	~RoboClaw();

	/**
	 * Drive motor 1 forward. Valid data range is 0 - 127. A value of 127 = full speed forward, 64 = about half speed forward and 0 = full stop.
	 *
	 * Send: [Address, 0, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool ForwardM1(uint8_t address, uint8_t speed);

	/**
	 *
	 * Drive motor 1 backwards. Valid data range is 0 - 127. A value of 127 full speed backwards, 64 = about half speed backward and 0 = full stop.
	 *
	 * Send: [Address, 1, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool BackwardM1(uint8_t address, uint8_t speed);

	/**
	 * Command 57 Preferred (SetMainVoltages)
	 *
	 * Sets main battery (B- / B+) minimum voltage level. If the battery voltages drops below the set voltage level RoboClaw will stop driving the motors. The voltage is set in .2 volt increments. A value of 0 sets the minimum value allowed which is 6V. The valid data range is 0 - 140 (6V - 34V).
	 *
	 * The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value.
	 *
	 * Examples of valid values are 6V = 0, 8V = 10 and 11V = 25.
	 *
	 * Send: [Address, 2, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetMinVoltageMainBattery(uint8_t address, uint8_t voltage);

	/**
	 * Command 57 Preferred (SetMainVoltages)
	 *
	 * Sets main battery (B- / B+) maximum voltage level. The valid data range is 30 - 175 (6V - 34V). During regenerative breaking a back voltage is applied to charge the battery. When using a power supply, by setting the maximum voltage level, RoboClaw will, before exceeding it, go into hard braking mode until the voltage drops below the maximum value set. This will prevent overvoltage conditions when using power supplies.
	 *
	 * The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
	 *
	 * Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.
	 *
	 * Send: [Address, 3, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage);

	/**
	 * Drive motor 2 forward. Valid data range is 0 - 127. A value of 127 full speed forward, 64 = about half speed forward and 0 = full stop.
	 *
	 * Send: [Address, 4, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool ForwardM2(uint8_t address, uint8_t speed);

	/**
	 * Drive motor 2 backwards. Valid data range is 0 - 127. A value of 127 full speed backwards, 64 = about half speed backward and 0 = full stop.
	 *
	 * Send: [Address, 5, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool BackwardM2(uint8_t address, uint8_t speed);

	/**
	 * Drive motor 1 forward or reverse. Valid data range is 0 - 127. A value of 0 = full speed reverse, 64 = stop and 127 = full speed forward.
	 *
	 * Send: [Address, 6, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool ForwardBackwardM1(uint8_t address, uint8_t speed);

	/**
	 * Drive motor 2 forward or reverse. Valid data range is 0 - 127. A value of 0 = full speed reverse, 64 = stop and 127 = full speed forward.
	 *
	 * Send: [Address, 7, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool ForwardBackwardM2(uint8_t address, uint8_t speed);

	/**
	 * Drive forward in mix mode. Valid data range is 0 - 127. A value of 0 = full stop and 127 = full forward.
	 *
	 * Send: [Address, 8, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool ForwardMixed(uint8_t address, uint8_t speed);

	/**
	 * Drive backwards in mix mode. Valid data range is 0 - 127. A value of 0 = full stop and 127 = full reverse.
	 *
	 * Send: [Address, 9, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool BackwardMixed(uint8_t address, uint8_t speed);

	/**
	 * Turn right in mix mode. Valid data range is 0 - 127. A value of 0 = stop turn and 127 = full speed turn.
	 * 
	 * Send: [Address, 10, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	*/
	bool TurnRightMixed(uint8_t address, uint8_t speed);

	/**
	 * Turn left in mix mode. Valid data range is 0 - 127. A value of 0 = stop turn and 127 = full speed turn.
	 *
	 * Send: [Address, 11, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool TurnLeftMixed(uint8_t address, uint8_t speed);

	/**
	 * Drive forward or backwards. Valid data range is 0 - 127. A value of 0 = full backward, 64 = stop and 127 = full forward.
	 *
	 * Send: [Address, 12, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool ForwardBackwardMixed(uint8_t address, uint8_t speed);

	/**
	 * Turn left or right. Valid data range is 0 - 127. A value of 0 = full left, 0 = stop turn and 127 = full right.
	 *
	 * Send: [Address, 13, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool LeftRightMixed(uint8_t address, uint8_t speed);

	/**
	 * Read M1 encoder count/position.
	 *
	 * Send: [Address, 16]
	 * Receive: [Enc1(4 bytes), Status, CRC(2 bytes)]
	 *
	 * Quadrature encoders have a range of 0 to 4,294,967,295. Absolute encoder values are converted from an analog voltage into a value from 0 to 2047 for the full 2v range.
	 *
	 * The status byte tracks counter underflow, direction and overflow. The byte value represents:
	 * Bit0 - Counter Underflow (1= Underflow Occurred, Clear After Reading)
	 * Bit1 - Direction (0 = Forward, 1 = Backwards)
	 * Bit2 - Counter Overflow (1= Underflow Occurred, Clear After Reading)
	 * Bit3 - Reserved
	 * Bit4 - Reserved
	 * Bit5 - Reserved
	 * Bit6 - Reserved
	 * Bit7 - Reserved
	 */
	uint32_t ReadEncM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);

	/**
	 * Read M2 encoder count/position.
	 *
	 * Send: [Address, 17]
	 * Receive: [EncCnt(4 bytes), Status, CRC(2 bytes)]
	 *
	 * Quadrature encoders have a range of 0 to 4,294,967,295. Absolute encoder values are converted from an analog voltage into a value from 0 to 2047 for the full 2v range.
	 *
	 * The Status byte tracks counter underflow, direction and overflow. The byte value represents:
	 * Bit0 - Counter Underflow (1= Underflow Occurred, Cleared After Reading)
	 * Bit1 - Direction (0 = Forward, 1 = Backwards)
	 * Bit2 - Counter Overflow (1= Underflow Occurred, Cleared After Reading)
	 * Bit3 - Reserved
	 * Bit4 - Reserved
	 * Bit5 - Reserved
	 * Bit6 - Reserved
	 * Bit7 - Reserved
	 */
	uint32_t ReadEncM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);

	/**
	 * Set the value of the Encoder 1 register. Useful when homing motor 1. This command applies to quadrature encoders only.
	 *
	 * Send: [Address, 22, Value(4 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetEncM1(uint8_t address, int32_t val);

	/**
	 * Set the value of the Encoder 2 register. Useful when homing motor 2. This command applies to quadrature encoders only.
	 *
	 * Send: [Address, 23, Value(4 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetEncM2(uint8_t address, int32_t val);

	/**
	 * Read M1 counter speed. Returned value is in pulses per second. RoboClaw keeps track of how many pulses received per second for both encoder channels.
	 *
	 * Send: [Address, 18]
	 * Receive: [Speed(4 bytes), Status, CRC(2 bytes)]
	 *
	 * Status indicates the direction (0 – forward, 1 - backward).
	 */
	uint32_t ReadSpeedM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);

	/**
	 * Read M2 counter speed. Returned value is in pulses per second. RoboClaw keeps track of how many pulses received per second for both encoder channels.
	 *
	 * Send: [Address, 19]
	 * Receive: [Speed(4 bytes), Status, CRC(2 bytes)]
	 *
	 * Status indicates the direction (0 – forward, 1 - backward).
	 */
	uint32_t ReadSpeedM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);

	/**
	 * Will reset both quadrature decoder counters to zero. This command applies to quadrature encoders only.
	 *
	 * Send: [Address, 20, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool ResetEncoders(uint8_t address);

	/**
	 * Read RoboClaw firmware version. Returns up to 48 bytes (depending on the Roboclaw model) and is terminated by a line feed character and a null character.
	 *
	 * Send: [Address, 21]
	 * Receive: [“RoboClaw 10.2A v4.1.11”,10,0, CRC(2 bytes)]
	 */
	bool ReadVersion(uint8_t address,char *version);

	/**
	 * Read the main battery voltage level connected to B+ and B- terminals. The voltage is returned in 10ths of a volt(eg 300 = 30v).
	 *
	 * Send: [Address, 24]
	 * Receive: [Value(2 bytes), CRC(2 bytes)]
	 */
	uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL);

	/**
	 * Read a logic battery voltage level connected to LB+ and LB- terminals. The voltage is returned in 10ths of a volt(eg 50 = 5v).
	 *
	 * Send: [Address, 25]
	 * Receive: [Value.Byte1, Value.Byte0, CRC(2 bytes)]
	 */
	uint16_t ReadLogicBatteryVoltage(uint8_t address,bool *valid=NULL);

	/**
	 * Note: This command is included for backwards compatibility. We recommend you use command 58 instead (SetLogicVoltages).
	 *
	 * Sets logic input (LB- / LB+) minimum voltage level. RoboClaw will shut down with an error if the voltage is below this level. The voltage is set in .2 volt increments. A value of 0 sets the minimum value allowed which is 6V. The valid data range is 0 - 140 (6V - 34V).
	 *
	 * The formula for calculating the voltage is: (Desired Volts - 6) x 5 = Value.
	 *
	 * Examples of valid values are 6V = 0, 8V = 10 and 11V = 25.
	 *
	 * Send: [Address, 26, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage);

	/**
	 * Note: This command is included for backwards compatibility. We recommend you use command 58 instead (SetLogicVoltages).
	 * 
	 * Sets logic input (LB- / LB+) maximum voltage level. The valid data range is 30 - 175 (6V - 34V). RoboClaw will shutdown with an error if the voltage is above this level.
	 * 
	 * The formula for calculating the voltage is: Desired Volts x 5.12 = Value.
	 * 
	 * Examples of valid values are 12V = 62, 16V = 82 and 24V = 123.
	 * 
	 * Send: [Address, 27, Value, CRC(2 bytes)]
	 * Receive: [0xFF]
	  */
	bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage);

	/**
	 * Several motor and quadrature combinations can be used with RoboClaw. In some cases the default PID values will need to be tuned for the systems being driven. This gives greater flexibility in what motor and encoder combinations can be used. The RoboClaw PID system consist of four constants starting with QPPS, P = Proportional, I  = Integral and D = Derivative.
	 *
	 * The defaults values are:
	 * QPPS = 44000
	 * P = 0x00010000
	 * I = 0x00008000
	 * D = 0x00004000
	 *
	 * QPPS is the speed of the encoder when the motor is at 100% power. P, I, D are the default values used after a reset. Command syntax:
	 *
	 * Send: [Address, 28, D(4 bytes), P(4 bytes), I(4 bytes), QPPS(4 byte), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM1VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);

	/**
	 * Several motor and quadrature combinations can be used with RoboClaw. In some cases the default PID values will need to be tuned for the systems being driven. This gives greater flexibility in what motor and encoder combinations can be used. The RoboClaw PID system consist of four constants starting with QPPS, P = Proportional, I = Integral and D = Derivative.
	 *
	 * The defaults values are:
	 * QPPS = 44000
	 * P = 0x00010000
	 * I = 0x00008000
	 * D = 0x00004000
	 *
	 * QPPS is the speed of the encoder when the motor is at 100% power. P, I, D are the default values used after a reset. Command syntax:
	 *
	 * Send: [Address, 29, D(4 bytes), P(4 bytes), I(4 bytes), QPPS(4 byte), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM2VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);

	/**
	 * Read the pulses counted in that last 300th of a second. This is an unfiltered version of command 18 (ReadSpeedM1). Command 30 (this) can be used to make a independent PID routine. Value returned is in encoder counts per second.
	 *
	 * Send: [Address, 30]
	 * Receive: [Speed(4 bytes), Status, CRC(2 bytes)]
	 *
	 * The Status byte is direction (0 – forward, 1 - backward).
	 */
	uint32_t ReadISpeedM1(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);

	/**
	 * Read the pulses counted in that last 300th of a second. This is an unfiltered version of command 19 (ReadSpeedM2). Command 31 (this) can be used to make a independent PID routine. Value returned is in encoder counts per second.
	 *
	 * Send: [Address, 31]
	 * Receive: [Speed(4 bytes), Status, CRC(2 bytes)]
	 *
	 * The Status byte is direction (0 – forward, 1 - backward).
	 */
	uint32_t ReadISpeedM2(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);

	/**
	 * Drive M1 using a duty cycle value. The duty cycle is used to control the speed of the motor without a quadrature encoder.
	 * 
	 * Send: [Address, 32, Duty(2 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 * 
	 * The duty value is signed and the range is -32767 to +32767 (eg. +-100% duty). 
	 */
	bool DutyM1(uint8_t address, uint16_t duty);

	/**
	 * Drive M2 using a duty cycle value. The duty cycle is used to control the speed of the motor without a quadrature encoder. The command syntax:
	 * 
	 * Send: [Address, 33, Duty(2 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 * 
	 * The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty).
	 */
	bool DutyM2(uint8_t address, uint16_t duty);

	/**
	 * Drive both M1 and M2 using a duty cycle value. The duty cycle is used to control the speed of the motor without a quadrature encoder. The command syntax:
	 *
	 * Send: [Address, 34, DutyM1(2 Bytes), DutyM2(2 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty).
	 */
	bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);

	/**
	 * Drive M1 using a speed value. The sign indicates which direction the motor will turn. This command is used to drive the motor by quad pulses per second. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate as fast as possible until the defined rate is reached.
	 *
	 * Send: [Address, 35, Speed(4 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SpeedM1(uint8_t address, uint32_t speed);

	/**
	 * Drive M2 with a speed value. The sign indicates which direction the motor will turn. This command is used to drive the motor by quad pulses per second. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent, the motor will begin to accelerate as fast as possible until the rate defined is reached.
	 *
	 * Send: [Address, 36, Speed(4 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SpeedM2(uint8_t address, uint32_t speed);

	/**
	 * Drive M1 and M2 in the same command using a signed speed value. The sign indicates which direction the motor will turn. This command is used to drive both motors by quad pulses per second. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate as fast as possible until the rate defined is reached.
	 *
	 * Send: [Address, 37, SpeedM1(4 Bytes), SpeedM2(4 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2);

	/**
	 * Drive M1 with a signed speed and acceleration value. The sign indicates which direction the motor will run. The acceleration values are not signed. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.
	 *
	 * Send: [Address, 38, Accel(4 Bytes), Speed(4 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second.
	 * Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
	 */
	bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);

	/**
	 * Drive M2 with a signed speed and acceleration value. The sign indicates which direction the motor will run. The acceleration value is not signed. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.
	 *
	 * Send: [Address, 39, Accel(4 Bytes), Speed(4 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second.
	 * Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
	 */
	bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);

	/**
	 * Drive M1 and M2 in the same command using one value for acceleration and two signed speed values for each motor. The sign indicates which direction the motor will run. The acceleration value is not signed. The motors are sync during acceleration. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.
	 *
	 * Send: [Address, 40, Accel(4 Bytes), SpeedM1(4 Bytes), SpeedM2(4 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second.
	 * Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
	 */
	bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2);

	/**
	 * Drive M1 with a signed speed and distance value. The sign indicates which direction the motor will run. The distance value is not signed. This command is buffered. This command is used to control the top speed and total distance traveled by the motor. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.
	 *
	 * Send: [Address, 41, Speed(4 Bytes), Distance(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
	 */
	bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);

	/**
	 * Drive M2 with a speed and distance value. The sign indicates which direction the motor will run. The distance value is not signed. This command is buffered. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.
	 *
	 * Send: [Address, 42, Speed(4 Bytes), Distance(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
	 */
	bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);

	/**
	 * Drive M1 and M2 with a speed and distance value. The sign indicates which direction the motor will run. The distance value is not signed. This command is buffered. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.
	 *
	 * Send: [Address, 43, SpeedM1(4 Bytes), DistanceM1(4 Bytes), SpeedM2(4 Bytes), DistanceM2(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
	 */
	bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);

	/**
	 * Drive M1 with a speed, acceleration and distance value. The sign indicates which direction the motor will run. The acceleration and distance values are not signed. This command is used to control the motors top speed, total distanced traveled and at what incremental acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.
	 *
	 * Send: [Address, 44, Accel(4 bytes), Speed(4 Bytes), Distance(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
	 */
	bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);

	/**
	 * Drive M2 with a speed, acceleration and distance value. The sign indicates which direction the motor will run. The acceleration and distance values are not signed. This command is used to control the motors top speed, total distanced traveled and at what incremental acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.
	 *
	 * Send: [Address, 45, Accel(4 bytes), Speed(4 Bytes), Distance(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
	 */
	bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);

	/**
	 * Drive M1 and M2 with a speed, acceleration and distance value. The sign indicates which direction the motor will run. The acceleration and distance values are not signed. This command is used to control both motors top speed, total distanced traveled and at what incremental acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.
	 *
	 * Send: [Address, 46, Accel(4 Bytes), SpeedM1(4 Bytes), DistanceM1(4 Bytes), SpeedM2(4 bytes), DistanceM2(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
	 */
	bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);

	/**
	 * Read both motor M1 and M2 buffer lengths. This command can be used to determine how many commands are waiting to execute.
	 *
	 * Send: [Address, 47]
	 * Receive: [BufferM1, BufferM2, CRC(2 bytes)]
	 *
	 * The return values represent how many commands per buffer are waiting to be executed. The maximum buffer size per motor is 64 commands(0x3F). A return value of 0x80(128) indicates the buffer is empty. A return value of 0 indiciates the last command sent is executing. A value of 0x80 indicates the last command buffered has finished.
	 */
	bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2);

	/**
	 * Read the current PWM output values for the motor channels. The values returned are +/-32767. The duty cycle percent is calculated by dividing the Value by 327.67.
	 *
	 * Send: [Address, 48]
	 * Receive: [M1 PWM(2 bytes), M2 PWM(2 bytes), CRC(2 bytes)]
	 */
	bool ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2);

	/**
	 * Read the current draw from each motor in 10mA increments. The Amps value is calculated by dividing the value by 100.
	 *
	 * Send: [Address, 49]
	 * Receive: [M1 Current(2 bytes), M2 Currrent(2 bytes), CRC(2 bytes)]
	 */
	bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2);

	/**
	 * Drive M1 and M2 in the same command using one value for acceleration and two signed speed values for each motor. The sign indicates which direction the motor will run. The acceleration value is not signed. The motors are sync during acceleration. This command is used to drive the motor by quad pulses per second and using an acceleration value for ramping. Different quadrature encoders will have different rates at which they generate the incoming pulses. The values used will differ from one encoder to another. Once a value is sent the motor will begin to accelerate incrementally until the rate defined is reached.
	 *
	 * Send: [Address, 50, AccelM1(4 Bytes), SpeedM1(4 Bytes), AccelM2(4 Bytes), SpeedM2(4 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The acceleration is measured in speed increase per second. An acceleration value of 12,000 QPPS with a speed of 12,000 QPPS would accelerate a motor from 0 to 12,000 QPPS in 1 second.
	 * Another example would be an acceleration value of 24,000 QPPS and a speed value of 12,000 QPPS would accelerate the motor to 12,000 QPPS in 0.5 seconds.
	 */
	bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);

	/**
	 * Drive M1 and M2 with a speed, acceleration and distance value. The sign indicates which direction the motor will run. The acceleration and distance values are not signed. This command is used to control both motors top speed, total distanced traveled and at what incremental acceleration value to use until the top speed is reached. Each motor channel M1 and M2 have separate buffers. This command will execute immediately if no other command for that channel is executing, otherwise the command will be buffered in the order it was sent. Any buffered or executing command can be stopped when a new command is issued by setting the Buffer argument. All values used are in quad pulses per second.
	 *
	 * Send: [Address, 51, AccelM1(4 Bytes), SpeedM1(4 Bytes), DistanceM1(4 Bytes), AccelM2(4 Bytes), SpeedM2(4 bytes), DistanceM2(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered and executed in the order sent. If a value of 1 is used the current running command is stopped, any other commands in the buffer are deleted and the new command is executed.
	 */
	bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag=0);

	/**
	 * Drive M1 with a signed duty and acceleration value. The sign indicates which direction the motor will run. The acceleration values are not signed. This command is used to drive the motor by PWM and using an acceleration value for ramping. Accel is the rate per second at which the duty changes from the current duty to the specified duty.
	 * 
	 * Send: [Address, 52, Duty(2 bytes), Accel(2 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 * 
	 * The duty value is signed and the range is -32768 to +32767(eg. +-100% duty). The accel value range is 0 to 655359(eg maximum acceleration rate is -100% to 100% in 100ms).
	*/
	bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel);

	/**
	 * Drive M2 with a signed duty and acceleration value. The sign indicates which direction the motor will run. The acceleration values are not signed. This command is used to drive the motor by PWM and using an acceleration value for ramping. Accel is the rate at which the duty changes from the current duty to the specified duty.
	 * Send: [Address, 53, Duty(2 bytes), Accel(2 Bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty). The accel value range is 0 to 655359 (eg maximum acceleration rate is -100% to 100% in 100ms).
	 */
	bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel);

	/**
	 * Drive M1 and M2 in the same command using acceleration and duty values for each motor. The sign indicates which direction the motor will run. The acceleration value is not signed. This command is used to drive the motor by PWM using an acceleration value for ramping. The command syntax:
	 *
	 * Send: [Address, CMD, DutyM1(2 bytes), AccelM1(4 Bytes), DutyM2(2 bytes), AccelM1(4 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty). The accel value range is 0 to 655359 (eg maximum acceleration rate is -100% to 100% in 100ms).
	 */
	bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);

	/**
	 * Read the PID and QPPS Settings.
	 *
	 * Send: [Address, 55]
	 * Receive: [P(4 bytes), I(4 bytes), D(4 bytes), QPPS(4 byte), CRC(2 bytes)]
	 */
	bool ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);

	/**
	 * Read the PID and QPPS Settings.
	 *
	 * Send: [Address, 56]
	 * Receive: [P(4 bytes), I(4 bytes), D(4 bytes), QPPS(4 byte), CRC(2 bytes)]
	 */
	bool ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);

	/**
	 * Set the Main Battery Voltage cutoffs, Min and Max. Min and Max voltages are in 10th of a volt increments. Multiply the voltage to set by 10.
	 *
	 * Send: [Address, 57, Min(2 bytes), Max(2bytes, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetMainVoltages(uint8_t address,uint16_t min,uint16_t max);

	/**
	 * Set the Logic Battery Voltages cutoffs, Min and Max. Min and Max voltages are in 10th of a volt increments. Multiply the voltage to set by 10.
	 *
	 * Send: [Address, 58, Min(2 bytes), Max(2bytes, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max);

	/**
	 * Read the Main Battery Voltage Settings. The voltage is calculated by dividing the value by 10
	 *
	 * Send: [Address, 59]
	 * Receive: [Min(2 bytes), Max(2 bytes), CRC(2 bytes)]
	 */
	bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max);

	/**
	 * Read the Logic Battery Voltage Settings. The voltage is calculated by dividing the value by 10
	 *
	 * Send: [Address, 60]
	 * Receive: [Min(2 bytes), Max(2 bytes), CRC(2 bytes)]
	 */
	bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max);

	/**
	 * The RoboClaw Position PID system consist of seven constants starting with P = Proportional, I = Integral and D = Derivative, MaxI = Maximum Integral windup, Deadzone in encoder counts, MinPos = Minimum Position and MaxPos = Maximum Position. The defaults values are all zero.
	 *
	 * Send: [Address, 61, D(4 bytes), P(4 bytes), I(4 bytes), MaxI(4 bytes), Deadzone(4 bytes), MinPos(4 bytes), MaxPos(4 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * Position constants are used only with the Position commands, 65,66 and 67 or when encoders are enabled in RC/Analog modes.
	 */
	bool SetM1PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);

	/**
	 * The RoboClaw Position PID system consist of seven constants starting with P = Proportional, I = Integral and D= Derivative, MaxI = Maximum Integral windup, Deadzone in encoder counts, MinPos = Minimum Position and MaxPos = Maximum Position. The defaults values are all zero.
	 *
	 * Send: [Address, 62, D(4 bytes), P(4 bytes), I(4 bytes), MaxI(4 bytes), Deadzone(4 bytes), MinPos(4 bytes), MaxPos(4 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * Position constants are used only with the Position commands, 65,66 and 67 or when encoders are enabled in RC/Analog modes.
	 */
	bool SetM2PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);

	/**
	 * Read the Position PID Settings.
	 *
	 * Send: [Address, 63]
	 * Receive: [P(4 bytes), I(4 bytes), D(4 bytes), MaxI(4 byte), Deadzone(4 byte), MinPos(4 byte), MaxPos(4 byte), CRC(2 bytes)]
	 */
	bool ReadM1PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);

	/**
	 * Read the Position PID Settings.
	 *
	 * Send: [Address, 64]
	 * Receive: [P(4 bytes), I(4 bytes), D(4 bytes), MaxI(4 byte), Deadzone(4 byte), MinPos(4 byte), MaxPos(4 byte), CRC(2 bytes)]
	 */
	bool ReadM2PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
	
	/**
	 * Move M1 position from the current position to the specified new position and hold the new position. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the speed in quadrature pulses the motor will run at after acceleration and before decceleration.
	 *
	 * Send: [Address, 65, Accel(4 bytes), Speed(4 Bytes), Deccel(4 bytes), Position(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);

	/**
	 * Move M2 position from the current position to the specified new position and hold the new position. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the speed in quadrature pulses the motor will run at after acceleration and before decceleration.
	 *
	 * Send: [Address, 66, Accel(4 bytes), Speed(4 Bytes), Deccel(4 bytes), Position(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);

	/**
	 * Move M1 & M2 positions from their current positions to the specified new positions and hold the new positions. Accel sets the acceleration value and deccel the decceleration value. QSpeed sets the speed in quadrature pulses the motor will run at after acceleration and before decceleration.
	 *
	 * Send: [Address, 67, AccelM1(4 bytes), SpeedM1(4 Bytes), DeccelM1(4 bytes),
	 * 				PositionM1(4 Bytes), AccelM2(4 bytes), SpeedM2(4 Bytes), DeccelM2(4 bytes), PositionM2(4 Bytes), Buffer, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag);

	/**
	 * Set the default acceleration for M1 when using duty cycle commands (Cmds 32 (DutyM1), 33 (DutyM2) and 34 (DutyM1M2)) or when using Standard Serial, RC and Analog PWM modes.
	 *
	 * Send: [Address, 68, Accel(4 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM1DefaultAccel(uint8_t address, uint32_t accel);

	/**
	 * Set the default acceleration for M2 when using duty cycle commands (Cmds 32 (DutyM1), 33 (DutyM2) and 34 (DutyM1M2)) or when using Standard Serial, RC and Analog PWM modes.
	 *
	 * Send: [Address, 69, Accel(4 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM2DefaultAccel(uint8_t address, uint32_t accel);

	/**
	 * Set modes for S3,S4 and S5.
	 *
	 * Send: [Address, 74, S3mode, S4mode, S5mode, CRC(2 bytes)]
	 * Receive: [0xFF]
	 *
	 * See page 71 of https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf for more information.
	 */
	bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
	
	/**
	 * Read mode settings for S3,S4 and S5. See command 74 for mode descriptions
	 *
	 * Send: [Address, 75]
	 * Receive: [S3mode, S4mode, S5mode, CRC(2 bytes)]
	 */
	bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode);

	/**
	 * Set RC/Analog mode control deadband percentage in 10ths of a percent. Default value is 25(2.5%). Minimum value is 0(no DeadBand), Maximum value is 250(25%).
	 *
	 * Send: [Address, 76, Reverse, Forward, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max);

	/**
	 * Read DeadBand settings in 10ths of a percent.
	 *
	 * Send: [Address, 77]
	 * Receive: [Reverse, SForward, CRC(2 bytes)]
	 */
	bool GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max);

	/**
	 * Read M1 and M2 encoder counters. Quadrature encoders have a range of 0 to 4,294,967,295. Absolute encoder values are converted from an analog voltage into a value from 0 to 2047 for the full 2V analog range.
	 *
	 * Send: [Address, 78]
	 * Receive: [Enc1(4 bytes), Enc2(4 bytes), CRC(2 bytes)]
	 */
	bool ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2);

	/**
	 * Read M1 and M2 instantaneous speeds. Returns the speed in encoder counts per second for the last 300th of a second for both encoder channels.
	 *
	 * Send: [Address, 79]
	 * Receive: [ISpeed1(4 bytes), ISpeed2(4 bytes), CRC(2 bytes)]
	 */
	bool ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2);

	/**
	 * Reset Settings to factory defaults.
	 *
	 * Send: [Address, 80]
	 * Receive: [0xFF]
	 */
	bool RestoreDefaults(uint8_t address);

	/**
	 * Read the board temperature. Value returned is in 10ths of degrees.
	 *
	 * Send: [Address, 82]
	 * Receive: [Temperature(2 bytes), CRC(2 bytes)]
	 */
	bool ReadTemp(uint8_t address, uint16_t &temp);

	/**
	 * Read the second board temperature (only on supported units). Value returned is in 10ths of degrees.
	 *
	 * Send: [Address, 83]
	 * Receive: [Temperature(2 bytes), CRC(2 bytes)]
	 */
	bool ReadTemp2(uint8_t address, uint16_t &temp);

	/**
	 * Read the current unit status.
	 *
	 * Send: [Address, 90]
	 * Receive: [Status, CRC(2 bytes)]
	 *
	 * See page 73 of https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf for more information.
	 */
	uint32_t ReadError(uint8_t address,bool *valid=NULL);

	/**
	 * Read the encoder mode for both motors.
	 *
	 * Send: [Address, 91]
	 * Receive: [Enc1Mode, Enc2Mode, CRC(2 bytes)]
	 *
	 * Encoder Mode bits:
	 * Bit 7 Enable/Disable RC/Analog Encoder support
	 * Bit 6 Reverse Encoder Relative Direction
	 * Bit 5 Reverse Motor Relative Direction
	 * Bit 4-1 N/A
	 * Bit 0 Quadrature(0)/Absolute(1)
	 */
	bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode);

	/**
	 * Set the Encoder Mode for motor 1. See command 91.
	 *
	 * Send: [Address, 92, Mode, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM1EncoderMode(uint8_t address,uint8_t mode);

	/**
	 * Set the Encoder Mode for motor 2. See command 91.
	 *
	 * Send: [Address, 93, Mode, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM2EncoderMode(uint8_t address,uint8_t mode);

	/**
	 * Writes all settings to non-volatile memory. Values will be loaded after each power up.
	 *
	 * Send: [Address, 94]
	 * Receive: [0xFF]
	 */
	bool WriteNVM(uint8_t address);

	/**
	 * Read all settings from non-volatile memory.
	 *
	 * Send: [Address, 95]
	 * Receive: [Enc1Mode, Enc2Mode, CRC(2 bytes)]
	 */
	bool ReadNVM(uint8_t address);

	/**
	 * Set config bits for standard settings.
	 *
	 * Send: [Address, 98, Config(2 bytes), CRC(2 bytes)]
	 * Receive: [0xFF]
	 * 
	 * See page 75 of https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf for more information.
	 */
	bool SetConfig(uint8_t address, uint16_t config);

	/**
	 * Read config bits for standard settings See Command 98.
	 *
	 * Send: [Address, 99]
	 * Receive: [Config(2 bytes), CRC(2 bytes)]
	 */
	bool GetConfig(uint8_t address, uint16_t &config);

	/**
	 * Set Motor 1 Maximum Current Limit. Current value is in 10ma units. To calculate multiply current limit by 100.
	 *
	 * Send: [Address, 134, MaxCurrent(4 bytes), 0, 0, 0, 0, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM1MaxCurrent(uint8_t address,uint32_t max);

	/**
	 * Set Motor 2 Maximum Current Limit. Current value is in 10ma units. To calculate multiply current limit by 100.
	 *
	 * Send: [Address, 134, MaxCurrent(4 bytes), 0, 0, 0, 0, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetM2MaxCurrent(uint8_t address,uint32_t max);

	/**
	 * Read Motor 1 Maximum Current Limit. Current value is in 10ma units. To calculate divide value by 100. MinCurrent is always 0.
	 * 
	 * Send: [Address, 135]
	 * Receive: [MaxCurrent(4 bytes), MinCurrent(4 bytes), CRC(2 bytes)]
	 */
	bool ReadM1MaxCurrent(uint8_t address,uint32_t &max);

	/**
	 * Read Motor 2 Maximum Current Limit. Current value is in 10ma units. To calculate divide value by 100. MinCurrent is always 0.
	 *
	 * Send: [Address, 136]
	 * Receive: [MaxCurrent(4 bytes), MinCurrent(4 bytes), CRC(2 bytes)]
	 */
	bool ReadM2MaxCurrent(uint8_t address,uint32_t &max);

	/**
	 * Set PWM Drive mode. Locked Antiphase(0) or Sign Magnitude(1).
	 *
	 * Send: [Address, 148, Mode, CRC(2 bytes)]
	 * Receive: [0xFF]
	 */
	bool SetPWMMode(uint8_t address, uint8_t mode);

	/**
	 * Read PWM Drive mode. See Command 148 (SetPWMMode).
	 *
	 * Send: [Address, 149]
	 * Receive: [PWMMode, CRC(2 bytes)]
	 */
	bool GetPWMMode(uint8_t address, uint8_t &mode);
	
	static int16_t library_version() { return _SS_VERSION; }

	virtual int available();
	void begin(long speed);
	bool isListening();
	bool overflow();
	int peek();
	virtual int read();
	int read(uint32_t timeout);
	bool listen();
	virtual size_t write(uint8_t byte);
	virtual void flush();
	void clear();

private:
	void crc_clear();
	void crc_update (uint8_t data);
	uint16_t crc_get();
	bool write_n(uint8_t byte,...);
	bool read_n(uint8_t byte,uint8_t address,uint8_t cmd,...);
	uint32_t Read4_1(uint8_t address,uint8_t cmd,uint8_t *status,bool *valid);
	uint32_t Read4(uint8_t address,uint8_t cmd,bool *valid);
	uint16_t Read2(uint8_t address,uint8_t cmd,bool *valid);
	uint8_t Read1(uint8_t address,uint8_t cmd,bool *valid);
	
};

#endif