/* 
    NOTE - the order of functions is important for correct
    calibration.
    
    HOW TO USE:
    Where other CO2 sensors require an nitrogen atmosphere to "zero"
    the sensor CO2 reference, the MHZ19 "zero" (confusingly) refers to the 
    background CO2 level hardcoded into the device at 400ppm (getBackgroundCO2() 
    sends a command to the device to retrieve the value);

    The best start for your sensor is to wait till CO2 values are as close to background
    levels as possible (currently an average of ~418ppm). Usually at night time and outside 
    if possible, otherwise when the house has been unoccupied for as long as possible such.

    If you are using auto calibration, the sensor will adjust its self every 24 hours
    (note here, the auto calibration algorithm uses the lowest observe CO2 value
    for that set of 24 hours as the zero - so, if your device is under an environment
    which does not fall to these levels, consider turning this off in your setup code). 
    
    If autoCalibration is set to "false", then getting the background calibration levels 
    correct at first try is essential.

    This is an example of the calibration sequence. Details are given on how to use each library 
    function. See main readme for more information.
*/

#include <Arduino.h>
#include "MHZ19.h" 
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial or non-uno library compatable device

#define RX_PIN 10                                          
#define TX_PIN 11                                          
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);    // Uno example
//HardwareSerial mySerial(1);               // ESP32 Example

unsigned long getDataTimer = 0;

void verifyRange(int range);

void setup()
{
    Serial.begin(9600);

    mySerial.begin(BAUDRATE);                                    // Uno example: Begin Stream with MHZ19 baudrate
    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);      // ESP32 Example

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
       autoCalibration(false) - turns auto calibration OFF. (automatically sent before defined period elapses)
       autoCalibration(true)  - turns auto calibration ON.
       autoCalibration()      - turns auto calibration ON.

       Advanced:
       autoCalibration(true, 12) - turns autocalibration ON and calibration period to 12 hrs (maximum 24hrs).
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

        int8_t Temp;                           // Buffer for temperature
        Temp = myMHZ19.getTemperature();       // Request Temperature (as Celsius)

        Serial.print("Temperature (C): ");
        Serial.println(Temp);

        getDataTimer = millis();              // Update inerval
    }
}

void verifyRange(int range)
{
    Serial.println("Requesting new range.");

    myMHZ19.setRange(range);                             // request new range write

    if (myMHZ19.getRange() == range)                     // Send command to device to return it's range value.
        Serial.println("Range successfully applied.");   // Success

    else
        Serial.println("Failed to apply range.");        // Failed
}
