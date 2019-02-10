/* 
    !NOTE - the order of functions is important for correct
    calibration.
    
    This is an example of a calibration sequence works.Details 
    are given on how to use each library function.    
*/

#include <Arduino.h>
#include "MHZ19.h"                                         // include main library
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial

#define RX_PIN 10                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 11                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;                                             // Constructor for MH-Z19 class
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // Constructor for Stream class *change for HardwareSerial, i.e. ESP32 ***

//HardwareSerial mySerial(1);                              // ESP32 Example 

unsigned long getDataTimer = 0;                             // Variable to store timer interval

void verifyRange(int range);                                // Forward Decleration for Non-IDE Environment

void setup()
{
    Serial.begin(9600);

    mySerial.begin(BAUDRATE);                               // Begin Stream with MHZ19 baudrate

    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // ESP32 Example 

    myMHZ19.begin(mySerial);                                // *Important, Pass your Stream reference

    /*            ### setRange(value)###  
       Basic:
       setRange(value) - set range to value (advise 2000 or 5000).
       setRange()      - set range to 2000.

       Advanced:
       Use verifyRange(int range) from this code at the bottom. 
    */
  
    myMHZ19.setRange(2000);                 

    /*            ###calibrateZero()###  
       Basic:
       calibrateZero() - request zero calibration

       Advanced:
       In Testing.
    */

    myMHZ19.calibrateZero();      

    /*             ### setSpan(value)###  
       Basic:
       setSpan(value) - set span to value (strongly recommend 2000)
       setSpan()      - set span to 2000;

    */

    myMHZ19.setSpan(2000); 

    /*            ###autoCalibration(false)###  
       Basic:
       autoCalibration(false) - turns auto calibration OFF. (automatically sent every 12 hours)
       autoCalibration(true)  - turns auto calibration ON.
       autoCalibration()      - turns auto calibration ON.
       Default                - auto calibration OFF.

       Advanced:
       autoCalibration(true, 24) - turns autocalibration ON and calibration period to 24 hrs (maximum value allowed).
    */

    myMHZ19.autoCalibration(false);  
                                    
}

void loop()
{
    if (millis() - getDataTimer >= 2000)     // Check if interval has elapsed (non-blocking delay() equivilant)
    {
        int CO2;
        CO2 = myMHZ19.getCO2();        
        
        Serial.print("CO2 (ppm): ");
        Serial.println(CO2);

        float Temp;                           // Buffer for temperature
        Temp = myMHZ19.getTemperature();      // Request Temperature (as Celsius)

        Serial.print("Temperature (C): ");
        Serial.println(Temp);

        getDataTimer = millis();              // Update inerval
    }
}

void verifyRange(int range)
{
    Serial.println("Requesting new range.");

    myMHZ19.setRange(range);                             // request new range write

    if (myMHZ19.getRange() == range)                     // This checks the MH-Z19 onboard value to see if it's updated.
        Serial.println("Range successfully applied.");   // Success

    else
        Serial.println("Failed to apply range.");        // Failed
}
