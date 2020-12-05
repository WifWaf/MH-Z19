/* 
  Raw CO2:
  Using the raw value competantly requires insight into the technology.
  
  However, itcan still be useful to have a rough value as a 'sanity check'.
  This is because the raw is not affect by span/range/zero/temperature.

  By plotting the Raw value vs CO2 ppm the full range (2000 usually),
  a trend can be produced (an exponetial rend is ideal for a 2000 range).
 */

#include "MHZ19.h"
#include <Arduino.h>
#include <SoftwareSerial.h>                                //  Remove if using HardwareSerial or non-uno compatabile device

#define RX_PIN 10                                          
#define TX_PIN 11                                         
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;                                             
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // Uno example

unsigned long getDataTimer = 0;

void setup()
{
  Serial.begin(9600);

  mySerial.begin(BAUDRATE);                               // Uno example: Begin Stream with MHZ19 baudrate  
  myMHZ19.begin(mySerial);                                 // *Important, Pass your Stream reference
}

void loop()
{
  if (millis() - getDataTimer >= 2000)
  {

    double adjustedCO2 = myMHZ19.getCO2Raw();

    Serial.println("----------------");
    Serial.print("Raw CO2: ");
    Serial.println(adjustedCO2);

    adjustedCO2 = 6.60435861e+15 * exp(-8.78661228e-04 * adjustedCO2);      // Exponential equation for Raw & CO2 relationship

    Serial.print("Adjusted CO2: ");
    Serial.print(adjustedCO2);
    Serial.println(" ppm");

    getDataTimer = millis();

  }
}
