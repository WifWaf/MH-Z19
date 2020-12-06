/* 
                  # THIS SKETCH IS NOT NEEDED WITH AUTOCALIBRATION #
                  
    Please don't run this unless you are aware of the function Range and Span carry out. 
    I would recommend avoiding this unless you have to.
    
    Very rough explanation below.

    Range:
    The range of which you intend to measure. I.e 0  - 2000 ppm, 0 - 4000 ppm.
    If you intend to increase it, be aware the sensor accuracy drops the wider the range.
    
    Changing this, usually requires changing Span which is not recommended.

    Span (Zero):
    The difference between the low and high range. In this case, that's 0 (we don't set this)
    and Range. So, it works out to be the same as the range.

    There is a BIG difference however - Span is a zero command, meaning it needs a reference
    CO2 value like calibration(), which matches the span.
    
    I.e. a span of 2000, needs a CO2 environemnt of 2000 for it to work.
*/

/* final note, I've found it's best to calibrate in this order: setRange(2000) -> calibrate() -> zeroSpan(2000) */

#define EXAMPLE_HAS_RANGE 1  // include range example
#define EXAMPLE_HAS_SPAN 0   // include span example

#include <Arduino.h>
#include "MHZ19.h" 
#include <SoftwareSerial.h>     // Uno example

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

    mySerial.begin(BAUDRATE);  // Serial for the sensor
    myMHZ19.begin(mySerial);   // Pass it to the library

    myMHZ19.autoCalibration(false);   // make sure auto calibration is off for this example
    Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status

#if EXAMPLE_HAS_RANGE
    /* Setting the range appears to need no more than the value */
    myMHZ19.setRange(2000);   // Set range to 2000
    Serial.println("Range set");
#endif

#if EXAMPLE_HAS_SPAN 
    /* For this you need a CO2 environment that matches your span. In doing so, you should wait 20 minutes before
       requesting the zero point */
    Serial.println("Waiting 20 minutes to stabalise...");
    timeElapse = 12e5;                    //  20 minutes in milliseconds
    while(millis() < timeElapse) {};      //  wait this duration
    myMHZ19.zeroSpan(2000);   // Set span to 2000
#endif

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

        timeElapse = millis();    // Update inerval
    }
}
