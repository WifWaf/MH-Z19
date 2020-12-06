/* 
                   # THIS SKETCH IS NOT NEEDED WITH AUTOCALIBRATION #
                   
    Unlike typical sensors, calibration() refers to the zero point where CO2 is 400ppm.
    This 400ppm comes from the average atmospheric value of 400ppm (or atleast was).

    Depending on the sensor model, the harcoded value can usually be found by 
    calling getBackgroundCO2();

    So if you intend to manually calibrate your sensor, it's usually best to do so at 
    night and outside after 20 minutes of run time.
    
    Instead if you're using autocalibration, then the sensor takes the lowest value observed 
    in the last 24 hours and adjusts it's self accordingly over a few weeks.

    HOW TO USE:

    ----- Hardware Method  -----
    By pulling the zero HD low (0V) for 7 Secs as per the datasheet.   

    ----- Software Method -----
    Run this sketch, disconnect MHZ19 from device after sketch ends (20+ minutes) and upload new
    code to avoid recalibration.
    
    ----- Auto calibration ----
    As mentioned above if this is set to true, the sensor will adjust it's self over a few weeks 
    according to the lowest observed CO2 values each day. *You don't need to run this sketch!
     
*/

#include <Arduino.h>
#include "MHZ19.h" 
#include <SoftwareSerial.h>     // Remove if using HardwareSerial or non-uno library compatable device

#define RX_PIN 10                                         
#define TX_PIN 11                                          
#define BAUDRATE 9600 

MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);    // Uno example

unsigned long timeElapse = 0;

void verifyRange(int range);

void setup()
{
    Serial.begin(9600);

    mySerial.begin(BAUDRATE);    // sensor serial
    myMHZ19.begin(mySerial);     // pass to library

    myMHZ19.autoCalibration(false);     // make sure auto calibration is off
    Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status
    
    Serial.println("Waiting 20 minutes to stabalise...");
   /* if you don't need to wait (it's already been this amount of time), remove the next 2 lines */
    timeElapse = 12e5;                    //  20 minutes in milliseconds
    while(millis() < timeElapse) {};      //  wait this duration

    Serial.println("Calibrating..");
    myMHZ19.calibrate();    // Take a reading which be used as the zero point for 400 ppm 
                                   
}

void loop()
{
    if (millis() - timeElapse >= 2000)  // Check if interval has elapsed (non-blocking delay() equivilant)
    {
        int CO2;
        CO2 = myMHZ19.getCO2();        
        
        Serial.print("CO2 (ppm): ");
        Serial.println(CO2);

        int8_t Temp;    // Buffer for temperature
        Temp = myMHZ19.getTemperature();    // Request Temperature (as Celsius)

        Serial.print("Temperature (C): ");
        Serial.println(Temp);

        timeElapse = millis();  // Update inerval
    }
}
