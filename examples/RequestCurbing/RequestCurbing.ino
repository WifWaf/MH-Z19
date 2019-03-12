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

     /---- getCO2ppm(isDec, New Request) -----/

    Usage example; getTempeature(true, false)

    isDec:
    (true)  Use isDec / command 133. 
    (false) Use isDec / command 134.        
    (false)  Default.

    New Request:
    (true)  Request is sent and response verified .
    (false) Request is not sent or verified.
    (true)  Default.

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

    mySerial.begin(BAUDRATE);                               // Begin Stream with MHZ19 baudrate

    //mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); // ESP32 Example 

    myMHZ19.begin(mySerial);                                // *Important, Pass your Stream reference
 
    myMHZ19.printCommunication(true, true);                 // *Shows communication between MHZ19 and Device.
                                                            // use printCommunication(true, false) to print as HEX

    myMHZ19.autoCalibration(false);                         // Turn Auto Calibration OFF    
}                                                         

void loop()
{
    if (millis() - getDataTimer >= 2000)                                 // Check if interval has elapsed
    {
        
       /* both share command  133 */
        Serial.print("CO2 (ppm): ");
        Serial.println(myMHZ19.getCO2(true, true));                       // unlimimted value, new request
        Serial.print("Temperature (C): ");                             
        Serial.println(myMHZ19.getTemperature(true, false));              // decimal value, not new request                     
        
       /* both share command  134 */
        Serial.print("CO2 (ppm): ");
        Serial.println(myMHZ19.getCO2(false, true));                      // limimted value, new request
        Serial.print("Temperature (C): ");                                            
        Serial.println(myMHZ19.getTemperature(false, false));             // whole integer value, not new request 
                
        getDataTimer = millis();                                                      
    }                                                                               
}   
