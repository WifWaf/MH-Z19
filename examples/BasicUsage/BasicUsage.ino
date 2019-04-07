#include <Arduino.h>
#include "MHZ19.h"                                         // include main library
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial or non-uno library compatable device

#define RX_PIN 10                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 11                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;                                             // Constructor for MH-Z19 class
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // Uno example
//HardwareSerial mySerial(1);                              // ESP32 Example

unsigned long getDataTimer = 0;                             // Variable to store timer interval

void setup()
{
    Serial.begin(9600);                                     // For ESP32 baudarte is 115200 etc.

    mySerial.begin(BAUDRATE);                               // Uno example: Begin Stream with MHZ19 baudrate
    
    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // ESP32 Example

    myMHZ19.begin(mySerial);                                // *Important, Pass your Stream reference 

    myMHZ19.autoCalibration();                              // Turn auto calibration ON (disable with autoCalibration(false))
}

void loop()
{
    if (millis() - getDataTimer >= 2000)                    // Check if interval has elapsed (non-blocking delay() equivilant)
    {
        int CO2;                                            // Buffer for CO2
        CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)

        Serial.print("CO2 (ppm): ");                      
        Serial.println(CO2);                                

        int8_t Temp;                                         // Buffer for temperature
        Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
        Serial.print("Temperature (C): ");                  
        Serial.println(Temp);                               

        getDataTimer = millis();                            // Update interval
    }
}
