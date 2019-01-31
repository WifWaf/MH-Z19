/*
 *****Testing Function*****

 Transmittance: 
  As Raw CO2 decreases with increasing CO2, a function has been created 
  named "transmittance". This uses a backwards projection to the zero 
  crossing of x to produce a 7-decimal place % value. (i.e. what % of 
  IR radiation was received of the amount sent).

  Below demonstrates transmittance, and the exponential equation from my 
 sensor under such conditions.

 ZeroCalibrte(Range):   ****Unknown Affect - Caution****
  The library uses the entered range to send an additional byte with the
  calibration command. The byte sent is matched as the closest to your 
  range, based on prior obserations in responses from the sesnor. 
 
  For example entered 2150, a decimal value of 9 is sent, which returns a
  command which can be interpreted as 2000. At the moment,it is unclear
  of the affect, or whether this byte is part of a set of bytes that must 
  be sent. However, they do not affect the alarm (set points), of course
  it could be that the alarm is disabled somewhere else and this currently
  defunct. 
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

void setup()
{  
    Serial.begin(9600);
    
    mySerial.begin(BAUDRATE);                                // Beging Stream with MHZ19 baudrate

    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // ESP32 Example 

    myMHZ19.begin(mySerial);                                 // *Imporant, Pass your Stream reference

    //myMHZ19.calibrateZero(2000);                             // Sends zero calibration with the corrasponding 2000 range byte, at byte 7.
}

void loop()
{
    if (millis() - getDataTimer > 2000)
    {
        Serial.println("------------------------------");
        Serial.print("Transmittance: ");
        Serial.print(myMHZ19.getTransmittance(), 7);         // 7 decimals for float/double maximum accuracy
        Serial.println(" %");
        getDataTimer = millis();
    }
}