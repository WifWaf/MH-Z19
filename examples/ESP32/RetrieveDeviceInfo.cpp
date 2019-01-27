#include "MHZ19.h"                                                                                                 
#include <Arduino.h>

#define RX_PIN 16                                
#define TX_PIN 17                                

#define SERIAL_NUMBER 1                         

MHZ19 myMHZ19(RX_PIN, TX_PIN, SERIAL_NUMBER);   

void setup()
{
  Serial.begin(115200);

  myMHZ19.begin();                               // Library Begin (this is essential)

  /*getVersion(char array[]) returns the version number to the input array. The version is formated
    as the first 2 bytes the major version, and second 2 bytes the minor version. e.g 02.11*/

  char myVersion[4];          
  myMHZ19.getVersion(myVersion);

  Serial.print("Firmware Version: ");

  for(byte i = 0; i < 4; i++)
  {
    Serial.print(myVersion[i]);
    if(i == 1)
      Serial.print(".");    
  }
   Serial.println("");

   Serial.print("Range: ");
   Serial.println(myMHZ19.getRange());   
   Serial.print("Background CO2: ");
   Serial.println(myMHZ19.getBackgroundCO2());
   Serial.print("Temperature Cal: ");
   Serial.println(myMHZ19.getTempAdjustment());

  /* For ESP_LOG */
  /*
  ESP_LOGI(TAG_SYS, "Firmware Version:\t%c%c.%c%c", myVersion[0], myVersion[1], myVersion[2], myVersion[3]);
  ESP_LOGI(TAG_SYS, "Background CO2:\t%d", myMHZ19.getBackgroundCO2());
  ESP_LOGI(TAG_SYS, "Temperature Cal:\t%d", myMHZ19.getTempAdjustment());
  ESP_LOGI(TAG_SYS, "Range:\t%d", myMHZ19.getRange());
  */
}

void loop()
{
while(1==1);
}