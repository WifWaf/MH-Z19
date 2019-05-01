 /* 
    For the library to calculate different values from a single response array, 
    the response is stored into 1 of 4 arrays. However, some function share the
    same response data, therefore the option has be created for any valid
    function (see header file) to request or not request a new response. This 
    reduces the chance of over saturation and communication errors.        
    
    /---- getCO2ppm(isUnlimited, New Request) -----/

    Usage example; getCO2ppm(true, false)     
         
    !Note: The first bool argument selects which command is used, unlimited or limited for 
    CO2. Although is not seen in an obvious way, it changes which command is sent
    and but more importantly which response array is written to.
        
    isUnlimited:
    (true)  Use unlimimted CO2 / command 133. 
    (false) Use limited CO2 / command 134.        
    (true)  Default.

     New Request:
    (true)  Request is sent and response verified .
    (false) Request is not sent or verified.
    (true)  Default.    

     /---- getCO2ppm(isFloat, New Request) -----/

    Usage example; getTempeature(true, false)

    isDec:
    (true)  Use isFloat / command 133. 
    (false) Use isFloat / command 134.        
    (false)  Default.

    New Request:
    (true)  Request is sent and response verified .
    (false) Request is not sent or verified.
    (true)  Default.

*/

#include <Arduino.h>
#include "MHZ19.h"
#include <SoftwareSerial.h>                                //  Remove if using HardwareSerial or non-uno compatabile device

#define RX_PIN 10                                         
#define TX_PIN 11                                        
#define BAUDRATE 9600                                      // Native to the sensor (do not change)

MHZ19 myMHZ19;                                             
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // Uno example,
//HardwareSerial mySerial(1);                              // ESP32 Example

unsigned long getDataTimer = 0;                                                        

void setup()
{
    Serial.begin(9600);

    mySerial.begin(BAUDRATE);                               // Uno example: Begin Stream with MHZ19 baudrate

    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // ESP32 Example

    myMHZ19.begin(mySerial);                                // *Important, Pass your Stream reference
 
    myMHZ19.printCommunication(true, true);                 // *Shows communication between MHZ19 and Device.
                                                            // use printCommunication(true, false) to print as HEX
    Serial.println("\n**** Setting AutoCalibration OFF ****");
    myMHZ19.autoCalibration(false);                         // Turn Auto Calibration OFF    
}                                                         

void loop()
{
    if (millis() - getDataTimer >= 5000)                                // Check if interval has elapsed
    {
        
        Serial.println("\n**** Unlimited CO2 ****");

       /* both printed under unlimited CO2 share command  133 */
        int CO2Unlim = myMHZ19.getCO2(true, true);
        Serial.print("CO2 (ppm): ");
        Serial.println(CO2Unlim);                                        // unlimimted value, new request

        /*  The below function is not fully tested, so please report any issues. */     
        float CO2UnlimTemp = myMHZ19.getTemperature(true, false);
        Serial.print("Temperature (C): ");                             
        Serial.println(CO2UnlimTemp);                                    // decimal value, not new request 

        Serial.println("\n**** Limited CO2 ****");   

       /* both printed under limited CO2 share command  134 */
        int CO2Lim = myMHZ19.getCO2(false, true);                        // limimted value, new request
        Serial.print("CO2 (ppm): "); 
        Serial.println(CO2Lim);                     
      
        int CO2limTemp = myMHZ19.getTemperature(false, false);          // non decimal palce value. Not a new request 
        Serial.print("Temperature (C): ");                                            
        Serial.println(CO2limTemp);                                       
        Serial.println("-----------------------------------"); 

        getDataTimer = millis();                                                      
    }                                                                               
}   
