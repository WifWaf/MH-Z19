/*************************************************** 
  Author: Jonathan Dempsey JDWifWaf@gmail.com
  
  Version: 1.4.2

  License: GPL-3.0

  This is a library for the MHZ19 CO2 Sensor

  The sensors uses UART to communicate and sends
  9 bytes in a modbus-like sequence. The sensor
  responds and the bytes are interpreted.
  
  Considerable time has gone into discovering, 
  implementing and making these commands accessible, so
  please abide the licensing and support open
  source.
 ****************************************************/

#ifndef MHZ19_H
#define MHZ19_H

#include <Arduino.h>

#ifdef ESP32
#include "esp32-hal-log.h"
#endif

#define MHZ19_ERRORS 1		   // Set to 0 to disable error prints

/* time out period for response */
#define TIMEOUT_PERIOD 500     // (ms)

/* For range mode,  */
#define DEFAULT_RANGE 2000     // MH-Z19 works best in this range

/* enum alias for error code defintions */
enum ERRORCODE
{
	RESULT_ERR_NULL = 0,
	RESULT_OK = 1,
	RESULT_ERR_TIMEOUT = 2,
	RESULT_ERR_MATCH = 3,
	RESULT_ERR_CRC = 4,
	RESULT_FILTER = 5,
	RESULT_FAILED = 6
};

/* alias for command types */
typedef enum COMMAND_TYPE
{
	RECOVER = 0,		// 0 Recovery Reset
	ABC = 1,			// 1 ABC Mode ON/OFF
	RAWCO2 = 2,			// 2 Raw CO2
	TEMPUNLIM = 3,		// 3 Temp float, CO2 Unlimited
	TEMPLIM = 4,		// 4 Temp integer, CO2 limited
	ZEROCAL = 5,		// 5 Zero Calibration
	SPANCAL = 6,		// 6 Span Calibration
	RANGE = 7,			// 7 Range
	GETRANGE = 8,		// 8 Get Range
	GETCALPPM = 9,		// 9 Get Background CO2
	GETFIRMWARE = 10,	// 10 Get Firmware Version
	GETLASTRESP = 11,	// 11 Get Last Response
	GETEMPCAL = 12 		// 12 Get Temp Calibration
} Command_Type;

class MHZ19
{
  public:
	/*###########################-Variables-##########################*/

	/* Holds last recieved errorcode from recieveResponse() */
	byte errorCode;

	/* for keeping track of the ABC run interval */
	unsigned long ABCRepeatTimer;

	/*#####################-Initiation Functions-#####################*/

	/* essential begin */
	void begin(Stream &stream);    

	/*########################-Set Functions-##########################*/

	/* Sets Range to desired value*/
	void setRange(int range = 2000);

	/* Sets Span to desired value below 10,000*/
	void setSpan(int span = 2000);

  /* Sets "filter mode" to ON or OFF & mode type (see example) */
	void setFilter(bool isON = true, bool isCleared = true);
 
	/*########################-Get Functions-##########################*/

	/* request CO2 values, 2 types of CO2 can be returned, isLimted = true (command 134) and is Limited = false (command 133) */
	int getCO2(bool isunLimited = true, bool force = true);

	/* returns the "raw" CO2 value of unknown units */
	float getCO2Raw(bool force = true);

	/* returns Raw CO2 value as a % of transmittance */ //<--- needs work to understand
	float getTransmittance(bool force = true);

	/*  returns temperature using command 134 or 135 if isFloat = true */
	float getTemperature(bool isFloat = false, bool force = true);
	
	/* Returns what appears to be an offset */ //<----- knowledgable people might have success using against the raw value
	float getTemperatureOffset(bool force = true);

	/* reads range using command 153 */
	int getRange();

	/* Returns accuracy value if available */
	byte getAccuracy(bool force = true);

	/* not yet implamented */
	byte getPWMStatus();

	/* returns MH-Z19 version using command 160, to the entered array */
	void getVersion(char rVersion[]);

	/* returns background CO2 used by sensor using command 156 */
	int getBackgroundCO2();

	/* returns temperature using command 163 (Note: this library deducts -2 when the value is used) */
	byte getTempAdjustment();

	/* returns last recorded response from device using command 162 */
	byte getLastResponse(byte bytenum);

	/*######################-Utility Functions-########################*/

	/* ensure communication is working (included in begin())*/
	void stablise();

	/* disables calibration or sets ABCPeriod */
	void autoCalibration(bool isON = true, byte ABCPeriod = 24);

	/* Calibrates "Zero" (Note: Zero refers to 400ppm for this sensor)*/
	void calibrateZero(int rangeCal = 0);

	/* requests a reset */
	void recoveryReset();

	/* use to show communication between MHZ19 and  Device */
	void printCommunication(bool isDec = true, bool isPrintComm = true);

  private:
	/*###########################-Variables-##########################*/
     
	/* pointer for Stream class to accept reference for hardware and software ports */
  	Stream* mySerial; 

	/* A flag which represents whether autocalibration ABC period was checked */
	bool ABCRepeat = false; 

	/* Flag set by setFilter() to signify is "filter mode" was made active */
	bool filterMode = false;

	/* Additional flag set by setFiler() to store which mode was selected */ 
	bool filterCleared = true;
  
	/* Holds interval for turning autocalibration off periodicaly */
	unsigned long ABCInterval = 4.32e7;

	/* Communication print options */
	bool printcomm = false;
	bool _isDec = true;

	/* holder for new commands which are to be sent */
	byte constructedCommand[9];

	/* Incoming Data Holders */
	byte responseTEMPUNLIM[9];		// Holds command 133 response values "temperature unlimited"
	byte responseTEMPLIM[9];		// Holds command 134 response values "temperature limited"
	byte responseRAW[9];			// Holds command 132 response values "CO2 Raw"
	byte responseSTAT[9];			// Holds other command response values such as range, background CO2 etc

	/*######################-Inernal Functions-########################*/

	/* Coordinates  sending, constructing and recieving commands */
	void provisioning(Command_Type commandtype, int inData = 0);

	/* Constructs commands using command array and entered values */
	void constructCommand(Command_Type commandtype, int inData = 0);

	/* generates a checksum for sending and verifying incoming data */
	byte checkSum(byte inBytes[]);

	/* Sends commands to the sensor */
	void write(byte toSend[]);

	/* Assigns response to the correct communcation arrays */
	void handleResponse(Command_Type commandtype);

	/* Call retrieveData to retrieve values from the sensor and check return code */
	byte receiveResponse(byte inBytes[9], Command_Type commandnumber);

	/* prints sending / recieving messages if enabled */
	void printstream(byte inbytes[9], bool isSent, byte pserrorCode);

	/* Cheks whether time elapse for next ABC OFF cycle has occured */
	void ABCCheck();

	/* converts integers to bytes according to /256 and %256 */
	void int2bytes(int inInt, byte *high, byte *low);

	/* converts bytes to integers according to *256 and + value */
	unsigned int bytes2int(byte high, byte low);
};
#endif
