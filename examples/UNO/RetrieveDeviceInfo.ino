#include "MHZ19.h"                                                                                                 
#include <Arduino.h>

#define RX_PIN 10                                
#define TX_PIN 11                              

MHZ19 myMHZ19(RX_PIN, TX_PIN);   

void setup()
{
  Serial.begin(9600);

  myMHZ19.begin();                               // Library Begin (this is essential)

  /*getVersion(char array[]) returns the version number to the input array. The version is formated
    as the first 2 bytes the major version, and second 2 bytes the minor version. e.g 02.11*/

  char myVersion[4];          
  myMHZ19.getVersion(myVersion);

  Serial.print("\nFirmware Version: ");
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
}

void loop()
{
while(1==1);
}