/*
    Example showing how to use "filter mode".

    Filter mode is useful if you are sending data to be stored and analysed such as 
    for a graphical display. Invalid readings produced by a reset and/or power loss
    to the sensor do not return invalid CRC values, and so will still return values
    despite being abnormal/false.
    
    The down side to this command is that an additional command is sent on each request as opposed
    to a single (for nearly all applications, this is no problem).
    
    To use filter mode, you must enable it with setFilter(true) (note; it will not return a code otherwise) 
    to check the errorCode to see if the filter was triggered during the getCO2 command phase and 
    ignore the next result . 

    The result is available by reading the function as normal, so you may store the value to check 
    for yourself at a later time.

    Below demonstrates the usuage:
*/

#include <Arduino.h>
#include "MHZ19.h"                        
#include <SoftwareSerial.h>                 //  Remove if using HardwareSerial or non-uno compatabile device
 
#define RX_PIN 10 
#define TX_PIN 11 
#define BAUDRATE 9600                       // Native to the sensor (do not change)

MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);    // Uno example
//HardwareSerial mySerial(1);               // ESP32 Example 

unsigned long getDataTimer = 0;

void setup()
{
    Serial.begin(9600);  
   
    mySerial.begin(BAUDRATE);                                    // Uno Example: Begin Stream with MHZ19 baudrate
    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);      // ESP32 Example

    myMHZ19.begin(mySerial);                                     // *Important, Pass your Stream reference here  

    myMHZ19.setFilter(true);                                     // Enable "filter mode"

    myMHZ19.autoCalibration(true);                               // Turn auto calibration ON
}

void loop()
{
    if (millis() - getDataTimer >= 2000) 
    {

        Serial.println("------------------");

        /* get sensor readings as signed integer */        
        int CO2Unlimited = myMHZ19.getCO2(true, true);
       
        /* get library code set by above getCO2 function */
        byte thisCode = myMHZ19.errorCode;

        /* handle code based upon error type */
        if(thisCode != RESULT_OK)
        {
            /* was it the filter ? */
            if(thisCode == RESULT_ERR_FILTER)
            {
                Serial.println("*** Filter was triggered ***");
                Serial.print("Offending Value: ");
                Serial.println(CO2Unlimited);
            }
            /* if not, then... */
            else
            {
                Serial.print("Communication Error Found. Error Code: ");
                Serial.println(thisCode);
            }
        } 
        /* error code was result OK. Print as "normal" */ 
        else
        {
            Serial.print("CO2: ");
            Serial.print(CO2Unlimited);
            Serial.println("PPM");
        }

        getDataTimer = millis();   // Update interval
    }
}
