/* -------------------------------------------------
  Author: Jonathan Dempsey JDWifWaf@gmail.com
  
  Version: 1.5.2

  License: LGPLv3

  Library supporting MHZ19 sensors
----------------------------------------------------- */

#include "MHZ19.h"

/*#########################-Commands-##############################*/

byte Commands[14] = {
    120,	// 0 Recovery Reset        Changes operation mode and performs MCU reset
    121,	// 1 ABC Mode ON/OFF       Turns ABC logic on or off (b[3] == 0xA0 - on, 0x00 - off)
    125,	// 2 Get ABC logic status  (1 - enabled, 0 - disabled)	
    132,	// 3 Raw CO2
    133,	// 4 Temp float, CO2 Unlimited
    134,	// 5 Temp integer, CO2 limited
    135,	// 6 Zero Calibration
    136,	// 7 Span Calibration
    153,	// 8 Range
    155,	// 9 Get Range
    156,	// 10 Get Background CO2
    160,	// 11 Get Firmware Version
    162,	// 12 Get Last Response
    163		// 13 Get Temp Calibration
};

/*#####################-Initiation Functions-#####################*/

void MHZ19::begin(Stream &serial) 
{  
    mySerial = &serial;    
    
    /* establish connection */
    verify();

    /* check if successful */
    if (this->errorCode != RESULT_OK) 
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
        provisioning(SPANCAL, span);
 
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
            provisioning(CO2UNLIM);
        else
            provisioning(CO2LIM);
     }

    if (this->errorCode == RESULT_OK || force == false)
    {
        if (!this->storage.settings.filterMode)
        {
            unsigned int validRead = 0;

            if(isunLimited)              
                validRead = makeInt(this->storage.responses.CO2UNLIM[4], this->storage.responses.CO2UNLIM[5]);
            else
                validRead = makeInt(this->storage.responses.CO2LIM[2], this->storage.responses.CO2LIM[3]);

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
                provisioning(CO2UNLIM);
            else
                provisioning(CO2LIM);
            
            checkVal[0] = makeInt(this->storage.responses.CO2UNLIM[4], this->storage.responses.CO2UNLIM[5]);
            checkVal[1] = makeInt(this->storage.responses.CO2LIM[2], this->storage.responses.CO2LIM[3]);

            // Limited CO2 stays at 410ppm during reset, so comparing unlimited which instead
            // shows an abormal value, reset duration can be found. Limited CO2 ppm returns to "normal"
            // after reset.

            if(this->storage.settings.filterCleared)
            {
                if(checkVal[0] > 32767 || checkVal[1] > 32767 || (((checkVal[0] - checkVal[1]) >= 10) && checkVal[1] == 410))
                {      
                    this->errorCode = RESULT_FILTER;
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
                    this->errorCode = RESULT_FILTER;
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

int MHZ19::getCO2Raw(bool force)
{
    if (force == true)
        provisioning(RAWCO2);

    if (this->errorCode == RESULT_OK || force == false)
        return makeInt(this->storage.responses.RAW[2], this->storage.responses.RAW[3]);

    else
        return 0;
}

float MHZ19::getTransmittance(bool force)
{
    if (force == true)
        provisioning(RAWCO2);

    if (this->errorCode == RESULT_OK || force == false)
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
            provisioning(CO2LIM);
            byte buff = (this->storage.responses.CO2LIM[4] - TEMP_ADJUST);

            baseTemp = buff - (byte)getTemperatureOffset(true);
            isSet = true;
        }
        
        if(force)
            provisioning(CO2UNLIM);

        if(this->errorCode == RESULT_OK || force == false)
        {
           float buff = baseTemp;
           buff += getTemperatureOffset(false);
           return buff;
        }
    }
    
    else if(!isFloat)
    {
        if (force == true)
            provisioning(CO2LIM);

        if (this->errorCode == RESULT_OK || force == false)
            return (this->storage.responses.CO2LIM[4] - TEMP_ADJUST);
    }
    
    return -273.15;    
}
 
float MHZ19::getTemperatureOffset(bool force)
{
     if (force == true)
        provisioning(CO2UNLIM);

    if (this->errorCode == RESULT_OK || force == false)
    {
        /* Value appears to be for CO2 offset (useful for deriving CO2 from raw?) */
        /* Adjustments and calculations are based on observations of temp behavour */
        float calc = (((this->storage.responses.CO2UNLIM[2] - 8) * 1500) + ((this->storage.responses.CO2UNLIM[3] * 100) * 1 / 17));
        calc /= 100;
        return calc;
    }

    return -273.15;
} 

int MHZ19::getRange()
{
    /* check get range was recieved */
    provisioning(GETRANGE);

    if (this->errorCode == RESULT_OK)
        /* convert MH-Z19 memory value and return */
        return (int)makeInt(this->storage.responses.STAT[4], this->storage.responses.STAT[5]);

    else
        return 0;
}

byte MHZ19::getAccuracy(bool force)
{
    if (force == true)
        provisioning(CO2LIM);

    if (this->errorCode == RESULT_OK || force == false)
        return this->storage.responses.CO2LIM[5];

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

    if (this->errorCode == RESULT_OK)
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

    if (this->errorCode == RESULT_OK)
        return (int)makeInt(this->storage.responses.STAT[4], this->storage.responses.STAT[5]);

    else
        return 0;
}

byte MHZ19::getTempAdjustment()
{
    provisioning(GETEMPCAL);

    /* 40 is returned here, however this library uses TEMP_ADJUST 
     when using temperature function as it appears inaccurate, 
    */

    if (this->errorCode == RESULT_OK)
        return (this->storage.responses.STAT[3]);

    else
        return 0;
}

byte MHZ19::getLastResponse(byte bytenum)
{
    provisioning(GETLASTRESP);

    if (this->errorCode == RESULT_OK)
        return (this->storage.responses.STAT[bytenum]);

    else
        return 0;
}

bool MHZ19::getABC()
{
    /* check get ABC logic status (1 - enabled, 0 - disabled) */
    provisioning(GETABC);

    if (this->errorCode == RESULT_OK)
        /* convert MH-Z19 memory value and return */
        return this->storage.responses.STAT[7];
    else
        return 1;
}

/*######################-Utility Functions-########################*/

void MHZ19::verify()
{
    unsigned long timeStamp = millis();

    /* construct common command (133) */
    constructCommand(CO2UNLIM);

    write(this->storage.constructedCommand);

    while (read(this->storage.responses.CO2UNLIM, CO2UNLIM) != RESULT_OK)
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

    /* compare CO2 & temp bytes, command(133), against last response bytes, command (162)*/
    for (byte i = 2; i < 6; i++)
    {
        if (this->storage.responses.CO2UNLIM[i] != this->storage.responses.STAT[i])
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
            ABCPeriod = MHZ19_ABC_PERIOD_DEF;    // Default bytes
    } 
    /* If ABC is OFF */
    else  
        ABCPeriod = MHZ19_ABC_PERIOD_OFF;                      // Set command byte to Zero to match command format.     

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
    byte asemblecommand[MHZ19_DATA_LEN];

    /* prepare arrays */
    memset(asemblecommand, 0, MHZ19_DATA_LEN);
    memset(this->storage.constructedCommand, 0, MHZ19_DATA_LEN);

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
    case CO2UNLIM:
        break;
    case CO2LIM:
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
    memcpy(this->storage.constructedCommand, asemblecommand, MHZ19_DATA_LEN);
}

void MHZ19::write(byte toSend[])
{
    /* for print communications */
    if (this->storage.settings.printcomm == true)
        printstream(toSend, true, this->errorCode);

    /* transfer to buffer */
    mySerial->write(toSend, MHZ19_DATA_LEN); 
 
    /* send */
    mySerial->flush(); 
}

byte MHZ19::read(byte inBytes[MHZ19_DATA_LEN], Command_Type commandnumber)
{
    /* loop escape */
    unsigned long timeStamp = millis();

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, MHZ19_DATA_LEN);

    /* prepare errorCode */
    this->errorCode = RESULT_NULL;

    /* wait until we have exactly the 9 bytes reply (certain controllers call read() too fast) */
    while (mySerial->available() < MHZ19_DATA_LEN)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD) 
        {
            #if defined (ESP32) && (MHZ19_ERRORS) 
            ESP_LOGW(TAG_MHZ19, "Timed out waiting for response");    
            #elif MHZ19_ERRORS
            Serial.println("!Error: Timed out waiting for response");
            #endif  

            this->errorCode = RESULT_TIMEOUT;
            
            /* clear incomplete 9 byte values, limit is finite */
            inBytes[1] = mySerial->available();
            for(uint8_t x = 0; x < inBytes[1]; x++)
            {
                inBytes[0] = mySerial->read();
                #if defined (ESP32) && (MHZ19_ERRORS) 
                ESP_LOGW(TAG_MHZ19, "Clearing Byte: %d", inBytes[0]);  
                #elif MHZ19_ERRORS
                Serial.print("!Warning: Clearing Byte: "); Serial.println(inBytes[0]);
                #endif     
            }
            //return error condition
            return RESULT_TIMEOUT;
        }
    }
    
    /* response recieved, read buffer */
    mySerial->readBytes(inBytes, MHZ19_DATA_LEN);

    if (this->errorCode == RESULT_TIMEOUT)
        return this->errorCode;

    byte crc = getCRC(inBytes);

    /* CRC error will not overide match error */
    if (inBytes[8] != crc)
        this->errorCode = RESULT_CRC;

    /* construct error code */
    if (inBytes[0] != this->storage.constructedCommand[0] || inBytes[1] != this->storage.constructedCommand[2])
        this->errorCode = RESULT_MATCH;

    /* if error has been assigned */
    if (this->errorCode == RESULT_NULL)
        this->errorCode = RESULT_OK;

    /* print results */
    if (this->storage.settings.printcomm == true)
        printstream(inBytes, false, this->errorCode);

    return this->errorCode;
}

void MHZ19::handleResponse(Command_Type commandtype)
{
    if (this->storage.constructedCommand[2] == Commands[RAWCO2])	// compare commands byte
        read(this->storage.responses.RAW, commandtype);				// returns error number, passes back response and inputs command

    else if (this->storage.constructedCommand[2] == Commands[CO2UNLIM])
        read(this->storage.responses.CO2UNLIM, commandtype);

    else if (this->storage.constructedCommand[2] == Commands[CO2LIM])
        read(this->storage.responses.CO2LIM, commandtype);

    else
        read(this->storage.responses.STAT, commandtype);
}

void MHZ19::printstream(byte inBytes[MHZ19_DATA_LEN], bool isSent, byte pserrorCode)
{
    if (pserrorCode != RESULT_OK && isSent == false)
    {
        Serial.print("Recieved >> ");
        if (this->storage.settings._isDec)
        {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }
        else
        {
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++)
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
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }
        else
        {
            for (uint8_t i = 0; i < MHZ19_DATA_LEN; i++)
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

void MHZ19::ABCCheck()
{
	/* check timer interval if dynamic hours have passed and if ABC_OFF was set to true */
	if (((millis() - ABCRepeatTimer) >= 4.32e7) && (this->storage.settings.ABCRepeat == true))
	{
		/* update timer inerval */
		ABCRepeatTimer = millis();
		
		/* construct command to skip next ABC cycle */
		provisioning(ABC, MHZ19_ABC_PERIOD_OFF);
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
