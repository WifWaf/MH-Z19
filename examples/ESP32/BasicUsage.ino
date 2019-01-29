#include <Arduino.h>
#include "MHZ19.h"                                         // include main library

#define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 17                                          // Tx pin which the MHZ19 Rx pin is attached to

#define SERIAL_NUMBER 1                                    // Serial number according to <HardwareSerial.h>

MHZ19 myMHZ19(RX_PIN, TX_PIN, SERIAL_NUMBER);              // Constructor for above parametesr

unsigned long getDataTimer = 0;                            // Variable to hold timer interval

void setup()
{
    Serial.begin(115200);

    myMHZ19.begin();                                       // Library Begin (this is essential)   

    myMHZ19.autoCalibration();                             // Turn ABC ON    
}

void loop()
{
    if (millis() - getDataTimer >= 2000)                    // Check if interval has elapsed (non-blocking delay() equivilant)
    {
        int CO2;                                            // Buffer for CO2
        CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)

        Serial.print("CO2 (ppm): ");                        
        Serial.println(CO2);                                // Print to serial

        float Temp;                                         // Buffer for temperature
        Temp = myMHZ19.getTemperature();                    // Request Temperature (as Celsius)

        Serial.print("Temperature (C): ");                   
        Serial.println(Temp);                                // Print to serial

        getDataTimer = millis();                             // Update interval
    }
}
