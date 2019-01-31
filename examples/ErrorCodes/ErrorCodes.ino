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
	RESULT_FAILED = 5                // Not currently  used
*/

#include <Arduino.h>
#include "MHZ19.h"
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial

#define RX_PIN 10                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 11                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;                                             // Constructor for MH-Z19 class
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // Constructor for Stream class *change for HardwareSerial, i.e. ESP32 ***

//HardwareSerial mySerial(1);                              // ESP32 Example 

unsigned long getDataTimer = 0;

void setRange(int range);                                  // Declerations for non-IDE platform                           

void setup()
{
    Serial.begin(9600);

    mySerial.begin(BAUDRATE);                                // Begin Stream with MHZ19 baudrate

    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 Example 

    myMHZ19.printCommunication();                            // Error Codes are also included here if found (not suitable outside of debugging)

    myMHZ19.begin(mySerial);                                 // *Imporant, Pass your Stream reference

    setRange(2000);                                          // Set Range 2000 using a function, see below
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
