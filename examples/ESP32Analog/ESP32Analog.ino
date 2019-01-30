/* 
   !Note, mV to PPM only works for a range of 2000.
   Please see AnalgOutSoftCal.cpp for other ranges or 
   increased accuracy.
   
   The analog output is located on the brown wire on the JST     
   version. On the non-JST version it can be found on the far 
   side, beside the Rx pin.
   
   To get a better conversion between bit-rate to mV, the
   esp_adc_cal.h header is used which provides support
   for conversion. See the header file for an explanation of 
   the function parameters.
   
   You can further increase accuracy by adjusting efuses after 
   taking various voltage measurements. See espressif API document
   titled "Analog to Digital Converter. *
   
   Alternatively, see AnalogOutSoftCall.cpp for a "soft" calibration.
*/

/*MHZ19 Library not Required*/
#include <esp_adc_cal.h>                                                            // For ADC Conversion
#include <Arduino.h>

esp_adc_cal_characteristics_t characteristics;                                      // Holds ADC characteristics from esp_adc_cal_characterize

unsigned long myMHZ19Timer = 0;                                                     

void setup()
{
    Serial.begin(115200);
 
    pinMode(33, PULLUP);                                                            // Pullup GPIO33

    gpio_num_t ADCPin;                                                              // Holds GPIO pin from adc1_pad_get_io_num
    adc1_pad_get_io_num(ADC1_CHANNEL_5, &ADCPin);                                   // Occupies ADCPin with GPIO number

    Serial.print("\nUsing GPIO: ");                                                 // Print GPIO number to which the ADC is attached
    Serial.println(ADCPin);                                                         // Print GPIO number to which the ADC is attached

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1109, &characteristics); // Occupies characteristics with ADC parameters
}

void loop()
{
    if (millis() - myMHZ19Timer >= 2000)                                            
    {
        adc1_config_width(ADC_WIDTH_BIT_12);                                         // According to <driver/adc.h>
        adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);                  // According to <driver/adc.h>

        uint32_t ADC12Bit = adc1_get_raw(ADC1_CHANNEL_5);                            // Record ADC 12bit raw value

        Serial.println("-----------------");
        Serial.print("Analog Raw: ");                                                 // Print ADC 12bit raw value to serial
        Serial.println(ADC12Bit);                                                     // Print ADC 12bit raw value to serial

        uint32_t ADCVoltage = esp_adc_cal_raw_to_voltage(ADC12Bit, &characteristics); // Convert ADC12Bit to ADCVoltage

        Serial.print("CO2 ppm(mV): ");                                                // Print mV (equivilant to ppm) to serial
        Serial.println(ADCVoltage);                                                   // Print mV (equivilant to ppm) to serial

        myMHZ19Timer = millis();                                                     
    }
}