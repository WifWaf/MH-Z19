/* 
   Every time communication is initiated, a code is subsequetnly 
   generated to represent the result of the attempt. This can 
   be used to monitor your program and pinpoint any difficulties
   your program might be having.

   The codes are as followed:

	RESULT_ERR_NULL = 0,             // This should not occur, and suggests a library logic error
	RESULT_OK = 1,                   // Communication was sucessfull
	RESULT_ERR_TIMEOUT = 2,          // Timed out waiting for a response
	RESULT_ERR_MATCH = 3,            // Recieved data does not match the usual syntax expected
	RESULT_ERR_CRC = 4,              // Recieved data does not match the CRC given
    RESULT_ERR_FILTER = 5,           // Filter was triggered (see FilterUsage example)
	RESULT_FAILED = 6                // Not currently used
*/

#include <Arduino.h>
#include "MHZ19.h"
#include <SoftwareSerial.h>                                //  Remove if using HardwareSerial or non-uno compatabile device

#define RX_PIN 10
#define TX_PIN 11
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // Uno example
//HardwareSerial mySerial(1);                              // ESP32 Example

unsigned long getDataTimer = 0;

void setRange(int range);                          

void setup()
{
    Serial.begin(9600);

    mySerial.begin(BAUDRATE);                                // Uno Exammple: Begin Stream with MHZ19 baudrate
    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 Example

    myMHZ19.printCommunication();                            // Error Codes are also included here if found (mainly for debugging/interest)

    myMHZ19.begin(mySerial);                                 // *Important, Pass your Stream reference

    //setRange(2000);                                        // Set Range 2000 using a function, see below (disabled as part of calibration)
}

void loop()
{

    if (millis() - getDataTimer >= 2000)               
    {
        int CO2;                                        // Buffer for CO2
        CO2 = myMHZ19.getCO2();                         // Request CO2 (as ppm)

        if(myMHZ19.errorCode == RESULT_OK)              // RESULT_OK is an alis for 1. Either can be used to confirm the response was OK.
        {
            Serial.print("CO2 Value successfully Recieved: ");
            Serial.println(CO2);
            Serial.print("Response Code: ");
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
        }

        else 
        {
            Serial.println("Failed to recieve CO2 value - Error");
            Serial.print("Response Code: ");
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
        }  

        getDataTimer = millis();                            
    }
}

void setRange(int range)
{
    Serial.println("Setting range..");

    myMHZ19.setRange(range);                                               // request new range write

    if ((myMHZ19.errorCode == RESULT_OK) && (myMHZ19.getRange() == range)) //RESULT_OK is an alias from the library,
        Serial.println("Range successfully applied");

    else
    {
        Serial.print("Failed to set Range! Error Code: ");
        Serial.println(myMHZ19.errorCode);          // Get the Error Code value
    }
}
