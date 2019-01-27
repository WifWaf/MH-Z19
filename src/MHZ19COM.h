/*************************************************** 
  Written by: Jonathan Dempsey JDWifWaf@gmail.com

  This is a library for the MHZ19 CO2 Sensor

  The sensors uses UART to communicate and sends
  9 bytes in a modbus-like sequence. The sensor
  responds and the bytes are interpreted.
  
  Considerable time has gone into discovering, 
  implementing and making these commands accessible, so
  please abide the licensing and support open
  source.
 ****************************************************/

#ifndef MHZ19COM
#define MHZ19COM

#include <Arduino.h> 

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
#endif

/* time out delay */
#define WAIT_READ_DELAY 100

/* native to the sensor */ //do not change these unless you are sure on the result
#define SCONFIG SERIAL_8N1
#define BAUDRATE 9600

enum COMERRORCODE
{
    COMRESULT_ERR_NULL = 0,
    COMRESULT_OK = 1,
    COMRESULT_ERR_TIMEOUT = 2,
    COMRESULT_ERR_MATCH = 3,
    COMRESULT_ERR_CRC = 4,
    COMRESULT_FAILED = 5,
};

void comWrite(byte inBytes[]);
byte comResponse(byte inBytes[]);

class COMMHZ19
{
  private:
    byte _CRX, _CTX, _CS;

  public:
    COMMHZ19(byte CRX, byte CTX, byte CS) : _CRX(CRX), _CTX(CTX), _CS(CS){};

#ifdef ARDUINO_AVR_UNO
    void comWrite(byte inBytes[])
    {
        /* open communications */
        SoftwareSerial mySerial(_CRX, _CTX);
        mySerial.begin(BAUDRATE);

        /* transfer to buffer */
        mySerial.write(inBytes, 9);

        /* send */
        mySerial.flush();
    }

    byte comResponse(byte inBytes[])
    {
        /* loop escape */
        byte TimeOut = 0;

        /* prepare memory array with unsigned chars of 0 */
        memset(inBytes, 0, 9);

        /* prepare errorCode */
        byte COMErrorCode = COMRESULT_ERR_NULL;

        /* open communications */
        SoftwareSerial mySerial(_CRX, _CTX);
        mySerial.begin(BAUDRATE);

        /* wait for response, allow for defined time before exit */
        while (mySerial.available() <= 0)
        {
            delay(WAIT_READ_DELAY);
            TimeOut++;
            if (TimeOut >= 50)
            {
                Serial.println("!Warning, Timed Out!");
                COMErrorCode = COMRESULT_ERR_TIMEOUT;
                return COMErrorCode;
            }
        }

        /* response recieved, read buffer */
        mySerial.readBytes(inBytes, 9);

        return COMErrorCode;
    }
#endif

#ifdef ESP32
    void comWrite(byte inBytes[])
    {
        /* open communications */
        HardwareSerial hserial(_CS);
        hserial.begin(BAUDRATE, SCONFIG, _CRX, _CTX);

        /* transfer to buffer */
        hserial.write(inBytes, 9);

        /* send */
        hserial.flush();
    }

    byte comResponse(byte inBytes[])
    {
        /* loop escape */
        byte TimeOut = 0;

        /* prepare memory array with unsigned chars of 0 */
        memset(inBytes, 0, 9);

        /* prepare errorCode */
        byte COMErrorCode = COMRESULT_ERR_NULL;

        /* open communications */
        HardwareSerial hserial(_CS);
        hserial.begin(BAUDRATE, SCONFIG, _CRX, _CTX);

        /* wait for response, allow for defined time before exit */
        while (hserial.available() <= 0)
        {
            delay(WAIT_READ_DELAY);
            TimeOut++;
            if (TimeOut >= 50)
            {
                Serial.println("!Warning, Timed Out!");
                COMErrorCode = COMRESULT_ERR_TIMEOUT;
                return COMErrorCode;
            }
        }

        /* response recieved, read buffer */
        hserial.readBytes(inBytes, 9);
        return COMErrorCode;
    }
#endif
};
#endif