#include <Arduino.h>
#include "MHZ19.h"                        
#include <SoftwareSerial.h>                               //  Remove if using HardwareSerial or non-uno compatabile device

#define RX_PIN 10 
#define TX_PIN 11 
#define BAUDRATE 9600                                     // Native to the sensor (do not change)

MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);                  // Uno example

unsigned long getDataTimer = 0;

void setup()
{
    Serial.begin(9600);  
   
    mySerial.begin(BAUDRATE);               //  Uno example: Begin Stream with MHZ19 baudrate
    myMHZ19.begin(mySerial);                // *Important, Pass your Stream reference here

    myMHZ19.autoCalibration(false);         // Turn auto calibration OFF
}

void loop()
{
    if (millis() - getDataTimer >= 2000) 
    {

        Serial.println("------------------");

        /* get sensor readings as signed integer */        
        int16_t CO2Unlimited = myMHZ19.getCO2(true, true);
        int16_t CO2limited = myMHZ19.getCO2(false, true);

        if(myMHZ19.errorCode != RESULT_OK)
            Serial.println("Error found in communication ");

        else
        {
            Serial.print("CO2 PPM Unlim: ");
            Serial.println(CO2Unlimited);

            Serial.print("CO2 PPM Lim: "); 
            Serial.println(CO2limited);

            /* Command 134 is limited by background CO2 and your defined range. These thresholds can provide a software alarm */

            if (CO2Unlimited - CO2limited >= 10 || CO2Unlimited - CO2limited <= -10)      // Check if CO2 reading difference is greater less than 10ppm
            {  
                Serial.print("Alert! CO2 out of range ");
                Serial.print(CO2limited);
                Serial.println(" threshold passed");   
             /* Sanity check vs Raw CO2 (has Span/Zero failed) or straight to your Alarm code */
           }
        }
        getDataTimer = millis(); // Update interval
    }
}
