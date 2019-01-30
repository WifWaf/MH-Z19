/* 
   This is an example of adjusting the ADC 12-bit value 
   without writing to efuses. 
   
   Simply record command 134 (0x86) CO2ppm and the concurrent 
   ADC value at frequent intervals over the scale of your 
   MHZ19 defined range - this is possible because the analog 
   value is always proportional to command 134.
   
   A trend characteristic of the ADC unique to that specific 
   ESP32 will be produced. This can be used to generated a
   linear line equation.
   
   Without analog efuse adjustment, this will prove more 
   accurate. However, the equation will need to be adjusted 
   for each range. 
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
}

void loop()
{  
  if (millis() - myMHZ19Timer >= 2000)                                 
  {
    static int AnalogCO2PPM = 0;
    
    adc1_config_width(ADC_WIDTH_BIT_12);                                 // According to <driver/adc.h>
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);          // According to <driver/adc.h>    

    AnalogCO2PPM = adc1_get_raw(ADC1_CHANNEL_5);                         // Record ADC conversion
    
    Serial.println("-----------------"); 
    Serial.print("Analog Raw: ");                                        // Print raw value to serial
    Serial.println(AnalogCO2PPM);                                        // Print raw value to serial
         
    /* 
      The 12-bit ADC, values in my case started at 320 (400ppm) and 
      ended at  2320 (2000ppm).
     

      Plotting values in-between, produces the strong linear trend: 
      "y = 8.04917E-1x + 1.37594E+2". This is multiplied as decimal
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