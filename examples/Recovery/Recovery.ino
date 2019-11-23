/* Only use this if you are struggling to get rational readings from your
   sensor - this uses the reset command which is not fully tested.
   
   This sequence goes through the standard setup and will attempt
   to reset the device if there is an issue and repeat untill a rational
   result is given.*/

#include <Arduino.h>
#include "MHZ19.h"
#include <SoftwareSerial.h>                                //  Remove if using HardwareSerial or non-uno compatabile device

#define RX_PIN 10 
#define TX_PIN 11 
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // Uno example
//HardwareSerial mySerial(1);                              // ESP32 Example

unsigned long getDataTimer = 0;

void setRange(int range);                                  // Declerations for non-IDE platform
void printErrorCode();

void setup()
{
    Serial.begin(9600);

    mySerial.begin(BAUDRATE);                                // Uno example: Begin Stream with MHZ19 baudrate

    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 Example 

    myMHZ19.begin(mySerial);                                 // *Imporant, Pass your Stream reference

    setRange(2000);                                          // Set Range 2000

     if (myMHZ19.errorCode == RESULT_OK)
         myMHZ19.calibrateZero();                            // Calibrate
     else
        printErrorCode();

    if (myMHZ19.errorCode == RESULT_OK)
        myMHZ19.setSpan(2000);                               // Set Span 2000
    else
        printErrorCode();

    if (myMHZ19.errorCode == RESULT_OK)
        myMHZ19.autoCalibration(false);                       // Turn auto calibration OFF
    else
        printErrorCode();
}

void loop()
{
    static byte timeout = 0;

    if (millis() - getDataTimer >= 2000)               
    {
        int CO2;                                        // Buffer for CO2
        CO2 = myMHZ19.getCO2();                         // Request CO2 (as ppm)    

        if (CO2 < 390 || CO2 > 2000)
        {
            Serial.println("Waiting for verification....");

            timeout++;

            if (timeout > 20)
            {
                Serial.println("Failed to verify.");
                Serial.println("Requesting MHZ19 reset sequence");

                myMHZ19.recoveryReset();                                            // Recovery Reset

                Serial.println("Restarting MHZ19.");
                Serial.println("Waiting for boot duration to elapse.....");

                delay(30000);       
                
                Serial.println("Waiting for boot verification...");

                for (byte i = 0; i < 3; i++)
                {
                    myMHZ19.verify();                                              // verification check

                    if (myMHZ19.errorCode == RESULT_OK)
                        break;
                }

                if (myMHZ19.errorCode == RESULT_OK)
                    Serial.println("Verified boot completion, please restart your device.");

                else
                    Serial.println("Failed to verify boot completion, check UART connection or increase delay.");

                while (1 == 1);
            }
        }

        else
        {
            Serial.println("Verified! Please upload a new sketch to avoid repeat recovery.");
 
            timeout = 0;                                // Rest Timeout Counter

            Serial.print("CO2 (ppm): ");
            Serial.println(CO2);

            int8_t Temp;                                 // Buffer for temperature
            Temp = myMHZ19.getTemperature(false, false); // Request Temperature (as Celsius), new request = false;

            Serial.print("Temperature (C): ");
            Serial.println(Temp);
        }
        getDataTimer = millis();                       
    }
}

void setRange(int range)
{
    Serial.println("Setting range..");

    myMHZ19.setRange(range);                                               // request new range write

    if ((myMHZ19.errorCode == RESULT_OK) && (myMHZ19.getRange() == range)) //RESULT_OK is an alias from the library,
        Serial.println("Range successfully applied.");

    else
    {
        printErrorCode();
    }
}

void printErrorCode()
{
    Serial.println("Communication error. Error Code: ");  // *Print error code using the library variable
    Serial.println(myMHZ19.errorCode);                    //  holds the last recieved code
}  
