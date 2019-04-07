/* 
   *Note: This is specific to the range you choose. 
   *Note: Below example will not be accurate. 

   The analog output is located on the brown wire on the JST     
   version. On the non-JST version it can be found on the far 
   side, beside the Rx pin. 

   Step 1: Record CO2 ppm and analog values at frequent intervals over your MHZ19 range (i.e. 2000 is default)
   Setp 2: Generated an equation based upon the trend (I.e. y=mx+c (linear), log etc)
   setp 3: Replace analog reading with x within the equation. 
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

        float adjustedADC = analogRead(A0);                 
        Serial.print("Analog raw: ");
        Serial.println(adjustedADC);

        adjustedADC = 6.4995*adjustedADC - 590.53; // format; y=mx+c
        Serial.print("Analog CO2: ");      
        Serial.println(adjustedADC); 
                                                                
        myMHZ19Timer = millis();                              
    }
}