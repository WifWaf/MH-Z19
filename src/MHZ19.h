/* ----------------------------------------------------------------
    Contact: JDWifWaf@gmail.com | Version: 2.0.1 | License: LGPLv3
   ---------------------------------------------------------------- */

#ifndef MHZ19_H
#define MHZ19_H

#include <Arduino.h>

#ifdef ESP32
#include "esp32-hal-log.h"
#define TAG_MHZ19 "MH-Z19"
#endif

#define MHZ19_LIB_ERRORS              1			// Set to 0 to disable error prints
#define MHZ19_LIB_FLASH_W_DELAY	  	  2			// Delay when writing to flash memory
#define MHZ19_LIB_FLASH_R_DELAY	  	  1			// Delay when reading from flash memory
#define MHZ19_LIB_DATA_LEN            9		    // Data protocol length
#define MHZ19_LIB_TEMP_ADJUST         40		// Value used to adjust the temeperature.
#define MHZ19_LIB_TIMEOUT_PERIOD      500		// Time out period for response (ms)
#define MHZ19_LIB_DEFAULT_RANGE       2000		// Default used when arguments not given
#define MHZ19_LIB_DEFAULT_SPAN        2000		// Default used when arguments not given
#define MHZ19_LIB_ABC_INTERVAL        4.32e7    // 12 hours in milliseconds
#define MHZ19_LIB_MAX_SPAN            10000     // Maximum allowed span
#define MHZ19_LIB_MAX_RANGE           65000		// Maximum allowed range

/* enum alias for error code defintions */
enum ERRORCODE
{
	RESULT_NULL = 0,
	RESULT_OK = 1,
	RESULT_TIMEOUT = 2,
	RESULT_MATCH = 3,
	RESULT_CRC = 4,
	RESULT_FILTER = 5
};

class MHZ19
{
  public:
	/*###########################-Variables-##########################*/

	/* Holds last recieved errorcode from recieveResponse() */
	byte errorCode;

	/*#####################-Initiation Functions-#####################*/

	/* essential begin */
	void begin(Stream &stream);    

	void wipeStorage();
	/*########################-Set Functions-##########################*/

	/* Sets Range to desired value*/
	void setRange(int range = MHZ19_LIB_DEFAULT_RANGE);

	/* Sets Span to desired value below 10,000*/
	void zeroSpan(int span = MHZ19_LIB_DEFAULT_SPAN);

    /* Sets "filter mode" to ON or OFF & mode type (see example) */
	void setFilter(bool isON = true, bool isCleared = true);
	
	bool setStorage(uint16_t address, uint8_t val);

	/*########################-Get Functions-##########################*/

	/* request CO2 values, 2 seperate commands can return CO2 values; 0x85 and 0x86 */
	int getCO2(bool isunLimited = true);

	/* returns the "raw" CO2 value of unknown units */
	unsigned int getCO2Raw();

	/* returns Raw CO2 value as a % of transmittance */		//<--- needs work to understand
	float getTransmittance();

	/*  returns temperature using command 134 or 135 if isFloat = true */
	float getTemperature();
	
	/* reads range using command 153 */
	int getRange();

	/* reads ABC-Status using command 125 / 0x7D */
	bool getABC();

	/* Returns accuracy value if available */
	byte getAccuracy();

	/* not yet implamented */
	byte getPWMStatus();

	/* returns MH-Z19 version using command 160, to the entered array */
	void getVersion(char rVersion[]);

	/* returns background CO2 used by sensor using command 156 */
	int getBackgroundCO2();

	/* returns temperature using command 163  */
	byte getTempAdjustment();

	/* returns last recorded response from device using command 162 */
	byte getLastResponse(byte bytenum);

	uint8_t getStorage(uint16_t address);

	/*######################-Utility Functions-########################*/

	/* ensure communication is working (included in begin())*/
	void verify();

	/* disables calibration or sets ABCPeriod */
	void autoCalibration(bool isON = true);

	/* Calibrates "Zero" (Note: Zero refers to 400ppm for this sensor)*/
	void calibrate();

	/*  Calibrate Backwards compatability */
	void inline calibrateZero(){ calibrate(); };
	
	/* requests a reset */
	void recoveryReset();

	/* use to show communication between MHZ19 and  Device */
	void printCommunication(bool isDec = false, bool isPrintComm = true);
	
	/* converts integers to bytes according to /256 and %256 */
	void makeByte(int inInt, byte *high, byte *low);

	/* converts bytes to integers according to *256 and + value */
	unsigned int makeInt(byte high, byte low);


  private:
	/*###########################-Variables-##########################*/
     
	/* pointer for Stream class for compatability */
  	Stream* mySerial; 

	/* Memory Pool */
	struct mempool
	{
		uint8_t cfg = 0X00 | 0x04;        // Default settings have MHZ19_FILTER_CLR_EN
		uint8_t fw_mv = 0;

		struct data
		{
			byte in[MHZ19_LIB_DATA_LEN];		    // Holds generic in data
			byte out[MHZ19_LIB_DATA_LEN];		    // Holds all out going data
		} block;
	} mem;

	/*######################-Inernal Functions-########################*/

	/* Coordinates  sending, constructing and recieving commands */
	void provisioning(byte comm, int inData = 0);

	/* Constructs commands using command array and entered values */
	void constructCommand(byte comm, int inData = 0, uint8_t extra = 0);

	/* generates a checksum for sending and verifying incoming data */
	byte getCRC(byte inBytes[]);

	/* Sends commands to the sensor */
	void write(byte toSend[]);

	/* Call retrieveData to retrieve values from the sensor and check return code */
	byte read(byte inBytes[], byte comm);

	/* Assigns response to the correct communcation arrays */
	void handleResponse(byte comm);

	/* prints sending / recieving messages if enabled */
	void printstream(byte inbytes[9], bool isSent, byte pserrorCode);

	/* Cheks whether time elapse for next ABC OFF cycle has occured */
	void ABCCheck();

	void cleanUp(uint8_t cnt);

	int filter(bool isunLimited, unsigned int CO2);

	byte getPage(uint16_t address);
};
#endif
