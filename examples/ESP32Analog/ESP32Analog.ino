/* 
   *Note: This is specific to the range you choose. 
   *Note: Below example will not be accurate. 

   The analog output is located on the brown wire on the JST     
   version. On the non-JST version it can be found on the far 
   side, beside the Rx pin. 

   Step 1: Record CO2 ppm and analog values at frequent intervals over your MHZ19 range (i.e. 2000 is default)
   Setp 2: Generated an equation based upon the trend (I.e. y=mx+c (linear), log etc)
   setp 3: Replace analog reading with x within the equation. 
*/

/* MH-Z19 Library Not Required  */                                                           
#include <driver/adc.h>                                                  // For ADC Conversion
#include <Arduino.h>                                                     

#define ANALOGPIN ADC1_CHANNEL_5,                                        // ADC1_CHANNEL_5 is GPIO33 @ <driver/adc.h> (locate GPIO alias in header)                                      

unsigned long myMHZ19Timer = 0;                                        

void setup()
{   
  Serial.begin(115200);                                   
   
  pinMode(33, PULLUP);                                                   // Pullup GPIO33
  
  gpio_num_t adcpin;                                                     // Buffer to hold GPIO number
  adc1_pad_get_io_num(ADC1_CHANNEL_5, &adcpin);                          // Occupies buffer with GPIO number

  Serial.print("\nUsing GPIO: ");                                        // Print GPIO number to which the ADC is attached
  Serial.println(adcpin);                                                // Print GPIO number to which the ADC is attached

  adc1_config_width(ADC_WIDTH_BIT_12);                                         // According to <driver/adc.h>
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);                  // According to <driver/adc.h>
}

void loop()
{  
  if (millis() - myMHZ19Timer >= 2000)                                 
  {
    static int AnalogCO2PPM = 0;

    AnalogCO2PPM = adc1_get_raw(ADC1_CHANNEL_5);                         // Record ADC conversion
    
    Serial.println("-----------------"); 
    Serial.print("Analog Raw: ");                                        // Print raw value to serial
    Serial.println(AnalogCO2PPM);                                        // Print raw value to serial
         
    /* 
      The 12-bit ADC, values in my case started at 320 (400ppm) and 
      ended at  2320 (2000ppm). 
     
      Plotting values in-between, produces the strong linear trend: 
      "y = 8.04917e+5x + 1.37594e+8". This is multiplied as decimal
      places are not required.
     
      As analog signal is the product of the command 134 (0x86), 
      and hence is also floored and capped by the MHZ19 on board 
      range value. 
    */   
 
    AnalogCO2PPM = ((8.04917e+5 * AnalogCO2PPM) + 1.37594e+8);            // Equation for relationship, multiplied by 10^6
    AnalogCO2PPM = (AnalogCO2PPM / 1000000);                              // Value divided by 10^6 (float is unnecessary)
                                                 
    Serial.print("Analog CO2: ");                                         // Print ppm value to serial
    Serial.println(AnalogCO2PPM);                                         // Print ppm value to serial

    myMHZ19Timer = millis();                                             
  }
}