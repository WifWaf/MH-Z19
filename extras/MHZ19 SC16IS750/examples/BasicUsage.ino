/* 
	To use, extract the folder extras folder MHZ19 SC16IS750 to a new folder in your libraries
	folder. You will need to remove the MHZ19 folder and rename this new folder to the same name.
*/

#include <Arduino.h>
#include "MHZ19.h"

#define SDA A4                      // Arduino Uno Pins
#define SDL A5                      // Arduino Uno Pins

#define SC16IS750_ADDRESS_BB  (0X9A)

MHZ19 myMHZ19(SDA, SDL, SC16IS750_ADDRESS_BB); // pass your I2C pins here and SC16IS750 address (see SC16IS750 header file)

unsigned long getDataTimer; 

void setup()
{
    Serial.begin(9600);             // Set baud rate         

    myMHZ19.begin();                // Does not require any arguments
    
    myMHZ19.autoCalibration(false);
} 

void loop()
{
   if (millis() - getDataTimer >= 2000)
    {
        int CO2;                                    // Buffer for CO2
        CO2 = myMHZ19.getCO2();                     // Request CO2 (as ppm)
        
        Serial.println("------------------");      
        Serial.print("CO2 (ppm): ");               
        Serial.println(CO2);                       

        float Temp;                                 // Buffer for temperature
        Temp = myMHZ19.getTemperature();            // Request Temperature (as Celsius)

        Serial.print("Temperature (C): ");          
        Serial.println(Temp);                       
        
        getDataTimer = millis(); 
    }
}
