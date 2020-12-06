/*
    Filter mode is useful if you are displaying in a graph or stastical analyses.

    When the sensor is reset, reading the CO2 value for around 30 seconds will produce inaccurate results
    while the IR sensor warms. The filter can behave in one of two ways:
    
    Mode 1)  myMHZ19.setFilter(true, true) (default)        <-- you can simply use setFilter() here;

    Values are filtered, and returned value is set to 0. An "errorCode" is set.

    Mode 2)  myMHZ19.setFilter(true, false)

    Values are not filtered but constrained if out of variable range. You must manually use the
    errorCode to complete the "filter". 

    (note, the down side to the filter is that an additional command is sent on each request. 
    For most applications, this is no problem).

    * Uncomment / comment out one of the two examples below*
*/

#include <Arduino.h>
#include "MHZ19.h"                        
#include <SoftwareSerial.h>                 //  Remove if using HardwareSerial or non-uno compatabile device
 
#define RX_PIN 10 
#define TX_PIN 11 
#define BAUDRATE 9600                       

#define MODE 1                              // <---------------- Set to 0 change to switch code for each mode 

MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);    // Uno example

unsigned long getDataTimer = 0;

void setup()
{
    Serial.begin(9600);  
   
    mySerial.begin(BAUDRATE);                                   // Uno Example: Begin Stream with MHZ19 baudrate
    myMHZ19.begin(mySerial);                                    // Pass Serial reference  
    
#if MODE
    myMHZ19.setFilter(true, true);                           
#else
    myMHZ19.setFilter(true, false);  
#endif
}

void loop()
{
    if (millis() - getDataTimer >= 2000) 
    {

        Serial.println("------------------");

        // get sensor readings as signed integer        
        int CO2Unlimited = myMHZ19.getCO2(true);
        
#if MODE
        
        // ######### Mode 1 ############# //

        Serial.print("CO2: ");
        Serial.print(CO2Unlimited);
        Serial.println(" PPM");

        if(CO2Unlimited != 0)
        {
            /* send/store your data code */
        }
        else
        {
            /* ignore data code */
        }
#else  
        // ######### Mode 2 ############# //         

        // get library error code returned getCO2 function
        byte thisCode = myMHZ19.errorCode;
        
        // handle code based upon error type
        if(thisCode != RESULT_OK)
        {
            // was it the filter ?
            if(thisCode == RESULT_FILTER)
            {
                Serial.println("*** Filter was triggered ***");
                Serial.print("Offending Value: ");
                Serial.println(CO2Unlimited);
            }
            // if not, then...
            else
            {
                Serial.print("Communication Error Found. Error Code: ");
                Serial.println(thisCode);
            }
        } 
        // error code was result OK. Print as "normal" 
        else
        {
            Serial.print("CO2: ");
            Serial.print(CO2Unlimited);
            Serial.println(" PPM");
        }
#endif
        getDataTimer = millis();   // Update interval
    }
}
