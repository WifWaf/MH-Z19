/* -------------------------------------------------
  Author: Jonathan Dempsey JDWifWaf@gmail.com
  
  Version: 1.4.4

  License: CC BY-NC-SA 3.0

  Library supporting MHZ19 sensors
----------------------------------------------------- */

#include "MHZ19.h"
#include "SC16IS750.h"

#ifdef ESP32
#include "esp32-hal-log.h"
#endif

SC16IS750 i2cuart = SC16IS750(SC16IS750_PROTOCOL_I2C);

/*#########################-Commands-##############################*/

byte Commands[13] = {
    120, // 0 Recovery Reset
    121, // 1 ABC Mode ON/OFF
    132, // 2 Raw CO2
    133, // 3 Temp float, CO2 Unlimited
    134, // 4 Temp integer, CO2 limited
    135, // 5 Zero Calibration
    136, // 6 Span Calibration
    153, // 7 Range
    155, // 8 Get Range
    156, // 9 Get Background CO2
    160, // 10 Get Firmware Version
    162, // 11 Get Last Response
    163  // 12 Get Temp Calibration
};

/*#####################-Initiation Functions-#####################*/

MHZ19::MHZ19(byte SDA, byte SDL, byte addr) : _SDA(SDA), _SDL(SDL), _addr(addr){}
 
void MHZ19::begin() 
{
     /* UART to Serial Bridge Initialization */
    i2cuart.begin(BAUDRATE, _SDA, _SDL, _addr);

    if (i2cuart.ping() != 1)
    {
    #if defined (ESP32) && (MHZ19_ERRORS)
        ESP_LOGE(TAG_MHZ19, "SC16IS750 bridge not found");
    #elif MHZ19_ERRORS
        Serial.println("!ERROR: SC16IS750 bridge not found");
    #endif
    }

    /* establish connection */
    verify();

    /* check if successful */
    if (errorCode != RESULT_OK)
    {
        #if defined (ESP32) && (MHZ19_ERRORS)
        ESP_LOGE(TAG_MHZ19, "Initial communication errorCode recieved");
     #elif MHZ19_ERRORS
        Serial.println("!ERROR: Initial communication errorCode recieved");
    #endif
    }
}

/*########################-Set Functions-##########################*/

void MHZ19::setRange(int range)
{
    if (range > 65000)
    {
        #if defined (ESP32) && (MHZ19_ERRORS)
        ESP_LOGE(TAG_MHZ19, "Invalid Range value (0 - 65000)");
     #elif MHZ19_ERRORS
        Serial.println("!ERROR: Invalid Range value (0 - 65000)");
    #endif
        return;
    }

    else
        provisioning(RANGE, range);
}

void MHZ19::setSpan(int span)
{
    if (span > 10000)
    {
        #if defined (ESP32) && (MHZ19_ERRORS)
        ESP_LOGE(TAG_MHZ19, "Invalid Span value (0 - 10000)");
     #elif MHZ19_ERRORS
        Serial.println("!ERROR: Invalid Span value (0 - 10000)");
    #endif
    }
    else
        provisioning(SPANCAL);

    return;
}

void MHZ19::setFilter(bool isON, bool isCleared)
{
    this->storage.settings.filterMode = isON;
    this->storage.settings.filterCleared = isCleared;
}

/*########################-Get Functions-##########################*/

int MHZ19::getCO2(bool isunLimited, bool force)
{
    if (force == true)
    {
        if(isunLimited)
            provisioning(TEMPUNLIM);
        else
            provisioning(TEMPLIM);
     }

    if (errorCode == RESULT_OK || force == false)
    {
        if (!this->storage.settings.filterMode)
        {
            unsigned int validRead = 0;

            if(isunLimited)              
                validRead = makeInt(this->storage.responses.TEMPUNLIM[4], this->storage.responses.TEMPUNLIM[5]);
            else
                validRead = makeInt(this->storage.responses.TEMPLIM[2], this->storage.responses.TEMPLIM[3]);

            if(validRead > 32767)
                validRead = 32767;  // Set to maximum to stop negative values being return due to overflow

            else
                 return validRead;   
        }
        else
        {
           /* FILTER BEGIN ----------------------------------------------------------- */
            unsigned int checkVal[2];
            bool trigFilter = false;

            // Filter was must call the opposest unlimited/limited command to work
            if(!isunLimited)                    
                provisioning(TEMPUNLIM);
            else
                provisioning(TEMPLIM);
            
            checkVal[0] = makeInt(this->storage.responses.TEMPUNLIM[4], this->storage.responses.TEMPUNLIM[5]);
            checkVal[1] = makeInt(this->storage.responses.TEMPLIM[2], this->storage.responses.TEMPLIM[3]);

            // Limited CO2 stays at 410ppm during reset, so comparing unlimited which instead
            // shows an abormal value, reset duration can be found. Limited CO2 ppm returns to "normal"
            // after reset.

            if(this->storage.settings.filterCleared)
            {
                if(checkVal[0] > 32767 || checkVal[1] > 32767 || (((checkVal[0] - checkVal[1]) >= 10) && checkVal[1] == 410))
                {      
                    errorCode = RESULT_FILTER;
                    return 0;
                }     
            }
            else
            {
                if(checkVal[0] > 32767)
                {
                    checkVal[0] = 32767;
                    trigFilter = true;
                }
                if(checkVal[1] > 32767)
                {
                    checkVal[1] = 32767;
                    trigFilter = true;
                }
                if(((checkVal[0] - checkVal[1]) >= 10) && checkVal[1] == 410)
                    trigFilter = true;

                if(trigFilter)
                {              
                    errorCode = RESULT_FILTER;
                }
            }

            if(isunLimited)       
                return checkVal[0];
            else
                return checkVal[1]; 
            /* FILTER END ----------------------------------------------------------- */             
        }              
    }
    return 0;
}

float MHZ19::getCO2Raw(bool force)
{
    if (force == true)
        provisioning(RAWCO2);

    if (errorCode == RESULT_OK || force == false)
        return makeInt(this->storage.responses.RAW[2], this->storage.responses.RAW[3]);

    else
        return 0;
}

float MHZ19::getTransmittance(bool force)
{
    if (force == true)
        provisioning(RAWCO2);

    if (errorCode == RESULT_OK || force == false)
    {
        float calc = (float)makeInt((this->storage.responses.RAW[2]), this->storage.responses.RAW[3]);

        return (calc * 100 / 35000); //  (calc * to percent / x(raw) zero)
    }

    else
        return 0;
}

float MHZ19::getTemperature(bool isFloat, bool force)
{
    if(isFloat)
    {
        static byte baseTemp = 0;
        static bool isSet = false;

        if(!isSet)
        {
            provisioning(TEMPLIM);
            byte buff = (this->storage.responses.TEMPLIM[4] - 38);

            baseTemp = buff - (byte)getTemperatureOffset(true);
            isSet = true;
        }
        
        if(force)
            provisioning(TEMPUNLIM);

        if(errorCode == RESULT_OK || force == false)
        {
           float buff = baseTemp;
           buff += getTemperatureOffset(false);
           return buff;
        }
    }
    
    else if(!isFloat)
    {
    if (force == true)
        provisioning(TEMPLIM);

    if (errorCode == RESULT_OK || force == false)
        return (this->storage.responses.TEMPLIM[4] - 38);
    }
    
    return -273.15;    
}
 
float MHZ19::getTemperatureOffset(bool force)
{
     if (force == true)
        provisioning(TEMPUNLIM);

    if (errorCode == RESULT_OK || force == false)
    {
        /* Value appears to be for CO2 offset (useful for deriving CO2 from raw?) */
        /* Adjustments and calculations are based on observations of temp behavour */

        float calc = (((this->storage.responses.TEMPUNLIM[2] - 8) * 1500) + ((this->storage.responses.TEMPUNLIM[3] * 100) * 1 / 17));
        calc /= 100;
        return calc;
    }

    return -273.15;
} 

int MHZ19::getRange()
{
    /* check get range was recieved */
    provisioning(GETRANGE);

    if (errorCode == RESULT_OK)
        /* convert MH-Z19 memory value and return */
        return (int)makeInt(this->storage.responses.STAT[4], this->storage.responses.STAT[5]);

    else
        return 0;
}

byte MHZ19::getAccuracy(bool force)
{
    if (force == true)
        provisioning(TEMPLIM);

    if (errorCode == RESULT_OK || force == false)
        return this->storage.responses.TEMPLIM[5];

    else
        return 0;

    //GetRange byte 7
}

byte MHZ19::getPWMStatus()
{
    //255 156 byte 4;
    return 0;
}

void MHZ19::getVersion(char rVersion[])
{
    provisioning(GETFIRMWARE);

    if (errorCode == RESULT_OK)
        for (byte i = 0; i < 4; i++)
        {
            rVersion[i] = char(this->storage.responses.STAT[i + 2]);
        }

    else
        memset(rVersion, 0, 4);
}

int MHZ19::getBackgroundCO2()
{
    provisioning(GETCALPPM);

    if (errorCode == RESULT_OK)
        return (int)makeInt(this->storage.responses.STAT[4], this->storage.responses.STAT[5]);

    else
        return 0;
}

byte MHZ19::getTempAdjustment()
{
    provisioning(GETEMPCAL);

    /* 40 is returned here, however this library deductes -2 
     when using temperature function as it appears inaccurate, 
    */

    if (errorCode == RESULT_OK)
        return (this->storage.responses.STAT[3]);

    else
        return 0;
}

byte MHZ19::getLastResponse(byte bytenum)
{
    provisioning(GETLASTRESP);

    if (errorCode == RESULT_OK)
        return (this->storage.responses.STAT[bytenum]);

    else
        return 0;
}

/*######################-Utility Functions-########################*/

void MHZ19::verify()
{
    unsigned long timeStamp = millis();

    /* construct common command (133) */
    constructCommand(TEMPUNLIM);

    write(this->storage.constructedCommand);

    while (read(this->storage.responses.TEMPUNLIM, TEMPUNLIM) != RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Failed to verify connection(1) to sensor.");
         #elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(1) to sensor.");
        #endif
            return;
        }
    }

    /* construct & write last response command (162) */
    constructCommand(GETLASTRESP);
    write(this->storage.constructedCommand);
    
    /* update timeStamp  for next comms iteration */ 
    timeStamp = millis();

    while (read(this->storage.responses.STAT, GETLASTRESP) != RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Failed to verify connection(2) to sensor.");
         #elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(2) to sensor.");
        #endif

            return;
        }
    }

    /* compare CO2 response command(133) against, last response command (162)*/
    for (byte i = 2; i < 6; i++)
    {
        if (this->storage.responses.TEMPUNLIM[i] != this->storage.responses.STAT[i])
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Last response is not as expected, verification failed.");
         #elif MHZ19_ERRORS
            Serial.println("!ERROR: Last response is not as expected, verification failed.");
        #endif

            return;
        }
    }
    return;
}

void MHZ19::autoCalibration(bool isON, byte ABCPeriod)
{
    /* If ABC is ON */
    if(isON)
    {
        /* If a period was defined */
        if (ABCPeriod)
        {
            /* Catch values out of range */
            if(ABCPeriod >= 24)
                ABCPeriod = 24;

            /* Convert to bytes */
             ABCPeriod *= 6.7;
        }      
        /* If no period was defined (for safety, even though default argument is given)*/
        else
            ABCPeriod = 160;    // Default bytes
    } 
    /* If ABC is OFF */
    else  
        ABCPeriod = 0x00;                      // Set command byte to Zero to match command format.     

    /* Update storage */  
    this->storage.settings.ABCRepeat = !isON;  // Set to opposite, as repeat command is sent only when ABC is OFF.

    provisioning(ABC, ABCPeriod);
}

void MHZ19::calibrateZero(int rangeCal)
{
    if (rangeCal)
    {
        int rangevalues[11] = {
            400,
            1000,
            1500,
            2000,
            3000,
            4000,
            5000,
            6000,
            7000,
            8000,
            10000};

        byte result = 0;

        for (byte i = 0; i < 11; i++)
        {
            if (i == 10)
            {
                result = (i + 8);
                break;
            }

            else if ((rangevalues[i] + (rangevalues[i + 1])) / 2 > rangeCal)
            {
                result = (i + 8);
                break;
            }
        }

        provisioning(ZEROCAL, result);
    }

    else
        provisioning(ZEROCAL);
}

void MHZ19::recoveryReset()
{
    provisioning(RECOVER);
}

void MHZ19::printCommunication(bool isDec, bool isPrintComm)
{
    this->storage.settings._isDec = isDec;
    this->storage.settings.printcomm = isPrintComm;
}

/*######################-Inernal Functions-########################*/

void MHZ19::provisioning(Command_Type commandtype, int inData)
{
    /* construct command */
    constructCommand(commandtype, inData);

    /* write to serial */
    write(this->storage.constructedCommand);

    /*return response */
    handleResponse(commandtype);

    /* Check if ABC_OFF needs to run */
    ABCCheck();
}

void MHZ19::constructCommand(Command_Type commandtype, int inData)
{
    /* values for conversions */
    byte High;
    byte Low;

    /* Temporary holder */
    byte asemblecommand[9];

    /* prepare arrays */
    memset(asemblecommand, 0, 9);
    memset(this->storage.constructedCommand, 0, 9);

    /* set address to 'any' */
    asemblecommand[0] = 255; ///(0xFF) 255/FF means 'any' address (where the sensor is located)

    /* set  register */
    asemblecommand[1] = 1; //(0x01) arbitrary byte number

    /* set command */
    asemblecommand[2] = Commands[commandtype]; // assign command value

    switch (commandtype)
    {
    case RECOVER:
        break;
    case ABC:
        if (this->storage.settings.ABCRepeat == false)
            asemblecommand[3] = inData;
        break;
    case RAWCO2:
        break;
    case TEMPUNLIM:
        break;
    case TEMPLIM:
        break;
    case ZEROCAL:
        if (inData)
            asemblecommand[6] = inData;
        break;
    case SPANCAL:
        makeByte(inData, &High, &Low);
        asemblecommand[3] = High;
        asemblecommand[4] = Low;
        break;
    case RANGE:
        makeByte(inData, &High, &Low);
        asemblecommand[6] = High;
        asemblecommand[7] = Low;
        break;
    case GETRANGE:
        break;
    case GETCALPPM:
        break;
    case GETFIRMWARE:
        break;
    case GETEMPCAL:
        break;
    case GETLASTRESP:
        break;
    }

    /* set checksum */
    asemblecommand[8] = getCRC(asemblecommand);

    /* copy bytes from asemblecommand to constructedCommand */
    memcpy(this->storage.constructedCommand, asemblecommand, 9);
}

byte MHZ19::getCRC(byte inBytes[])
{
    /* as shown in datasheet */
    byte x = 0, CRC = 0;

    for (x = 1; x < 8; x++)
    {
        CRC += inBytes[x];
    }

    CRC = 255 - CRC;
    CRC++;

    return CRC;
}

void MHZ19::write(byte toSend[])
{
    /* Send to SC16IS750 */
    for (byte i = 0; i < 9; i++)
    {
        i2cuart.write(toSend[i]);
    }

    /* for print communications */
    if (this->storage.settings.printcomm == true)
        printstream(toSend, true, errorCode);
}

void MHZ19::handleResponse(Command_Type commandtype)
{
    if (this->storage.constructedCommand[2] == Commands[2])      //compare commands byte
        read(this->storage.responses.RAW, commandtype); //returns errornum, passes back response, inputs command

    else if (this->storage.constructedCommand[2] == Commands[3])
        read(this->storage.responses.TEMPUNLIM, commandtype);

    else if (this->storage.constructedCommand[2] == Commands[4])
        read(this->storage.responses.TEMPLIM, commandtype);

    else
        read(this->storage.responses.STAT, commandtype);
}

byte MHZ19::read(byte inBytes[9], Command_Type commandnumber)
{
    /* loop escape */
    unsigned long timeStamp = millis();

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, 9);

    /* prepare errorCode */
    errorCode = RESULT_NULL;

    /* Read from SC16IS750 */
    while (i2cuart.available() == 0)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD) 
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGW(TAG_MHZ19, "Timed out waiting for response");
         #elif MHZ19_ERRORS
            Serial.println("!Warning: Timed out waiting for response");
        #endif

            errorCode = RESULT_TIMEOUT;
            return RESULT_TIMEOUT;
        }
    }

    byte index = 0;
    while (i2cuart.available() > 0)
    {
        inBytes[index] = i2cuart.read();
        index++;
    }

    if (errorCode == RESULT_TIMEOUT)
        return errorCode;

    byte crc = getCRC(inBytes);

    /* CRC error will not overide match error */
    if (inBytes[8] != crc)
        errorCode = RESULT_CRC;

    /* construct error code */
    if (inBytes[0] != this->storage.constructedCommand[0] || inBytes[1] != this->storage.constructedCommand[2])
        errorCode = RESULT_MATCH;

    /* if error has been assigned */
    if (errorCode == RESULT_NULL)
        errorCode = RESULT_OK;

    /* print results */
    if (this->storage.settings.printcomm == true)
        printstream(inBytes, false, errorCode);

    return errorCode;
}

void MHZ19::printstream(byte inBytes[9], bool isSent, byte pserrorCode)
{
   
    #if defined (ESP32) && (MHZ19_ERRORS)      
    if (pserrorCode != RESULT_OK && isSent == false)
    {
        if (this->storage.settings._isDec)
            ESP_LOGE(TAG_MHZ19, "Recieved >> %d %d %d %d %d %d %d %d %d ERROR Code: %d",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8],
                     pserrorCode);
        else
            ESP_LOGE(TAG_MHZ19, "Recieved >> %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x ERROR Code: %d",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8],
                     pserrorCode);
    }
    else
    {
        if (this->storage.settings._isDec)
            ESP_LOGD(TAG_MHZ19, "%s %d %d %d %d %d %d %d %d %d PASS", isSent ? "Sent << " : "Recieved >> ",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8]);
        else

            ESP_LOGD(TAG_MHZ19, "%s %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x PASS", isSent ? "Sent << " : "Recieved >> ",
                     inBytes[0], inBytes[1], inBytes[2], inBytes[3], inBytes[4], inBytes[5], inBytes[6], inBytes[7], inBytes[8]);
    }
     
    #elif MHZ19_ERRORS
    if (pserrorCode != RESULT_OK && isSent == false)
    {
        Serial.print("Recieved >> ");

        if (this->storage.settings._isDec)
        {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < 9; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }

        else
        {
            for (uint8_t i = 0; i < 9; i++)
            {
                Serial.print("0x");
                if (inBytes[i] < 16)
                    Serial.print("0");
                Serial.print(inBytes[i], HEX);
                Serial.print(" ");
            }
        }

        Serial.print("ERROR Code: ");
        Serial.println(pserrorCode);
    }

    else
    {
        isSent ? Serial.print("Sent << ") : Serial.print("Recieved >> ");

        if (this->storage.settings._isDec)
        {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < 9; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }

        else
        {

            for (uint8_t i = 0; i < 9; i++)
            {
                Serial.print("0x");
                if (inBytes[i] < 16)
                    Serial.print("0");
                Serial.print(inBytes[i], HEX);
                Serial.print(" ");
            }
        }
        Serial.println(" ");
    }           
    #endif
    
}

void MHZ19::ABCCheck()
{
    /* check timer interval if dynamic hours have passed and if ABC_OFF was set to true */
    if (((millis() - ABCRepeatTimer) >= 4.32e7) && (this->storage.settings.ABCRepeat == true))
    {
        /* update timer inerval */
        ABCRepeatTimer = millis();

        /* construct ABC_OFF command */
        constructCommand(ABC);

        /* write to serial */
        write(this->storage.constructedCommand);
    }
}

void MHZ19::makeByte(int inInt, byte *high, byte *low)
{
    *high = (byte)(inInt / 256);
    *low = (byte)(inInt % 256);

    return;
}

unsigned int MHZ19::makeInt(byte high, byte low)
{
    unsigned int calc = ((unsigned int)high * 256) + (unsigned int)low;

    return calc;
}
