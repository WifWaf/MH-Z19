/*
 *****Testing Function*****

 Transmittance: 
  As Raw CO2 decreases with increasing CO2, a function has been created 
  named "transmittance". This uses a backwards projection to the zero 
  crossing of x to produce a 7-decimal place % value. (i.e. what % of 
  IR radiation was received of the amount sent).

 Below demonstrates transmittance, and the exponential equation from my 
 sensor under such conditions.
 */
#include <Arduino.h>
#include "MHZ19.h"

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