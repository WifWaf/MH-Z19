
/* 
   Note, mV to PPM only works for a range of 2000.
   Please see ESP32's AnalgOutSoftCal.cpp for other ranges.

   The analog output is located on the brown wire on the JST     
   version. On the non-JST version it can be found on the far 
   side, beside the Rx pin.
*/

/*MHZ19 Library not Required*/

#include <Arduino.h>

#define ANALOGPIN A0                                          // ADC pin which the brown wire is attached to

unsigned long myMHZ19Timer = 0;                            

void setup()
{
    Serial.begin(9600);
    pinMode(ANALOGPIN, INPUT_PULLUP);                         // Pullup A0

    Serial.print("\nUsing Pin: ");                            // Print Raw Pin Number
    Serial.println(ANALOGPIN);                                      
}

void loop()
{
    if (millis() - myMHZ19Timer >= 2000)                      
    {
        Serial.println("-----------------");

        float ADCReading = analogRead(ANALOGPIN);              // Get analog value
        
        Serial.print("ADC Raw: ");                             // Print Raw ADC Value
        Serial.println(ADCReading);  

        Serial.print("ADC mV/ppm: ");                          // Convert accrording to 5000 mV/1023 units, 1023 = 12-bit. 
        Serial.println(ADCReading*4.8875855);                  // "Floats have only 6-7 decimal digits of precision"
                                                                
        myMHZ19Timer = millis();                              
    }
}