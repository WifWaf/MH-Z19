/*************************************************** 
  Author: Jonathan Dempsey JDWifWaf@gmail.com
  
  Version: 1.4.2

  License: GPL-3.0

  This is a library for the MHZ19 CO2 Sensor 

  The sensors uses UART to communicate and sends
  9 bytes in a modbus-like sequence. The sensor
  responds and the bytes are interpreted.
  
  Considerable time has gone into discovering, 
  implementing and making these commands accessible, so
  please abide the licensing and support open
  source.
 ****************************************************/

#include "MHZ19.h"

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

void MHZ19::begin(Stream &serial) 
{  
    mySerial = &serial;    
    
    /* establish connection */
    stablise();

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
    this->filterMode = isON;
    this->filterCleared = isCleared;
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
        if (!this->filterMode)
        {
            unsigned int validRead = 0;

            if(isunLimited)              
                validRead = bytes2int(responseTEMPUNLIM[4], responseTEMPUNLIM[5]);
            else
                validRead = bytes2int(responseTEMPLIM[2], responseTEMPLIM[3]);

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
            
            checkVal[0] = bytes2int(responseTEMPUNLIM[4], responseTEMPUNLIM[5]);
            checkVal[1] = bytes2int(responseTEMPLIM[2], responseTEMPLIM[3]);

            // Limited CO2 stays at 410ppm during reset, so comparing unlimited which instead
            // shows an abormal value, reset duration can be found. Limited CO2 ppm returns to "normal"
            // after reset.

            if(filterCleared)
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
        return bytes2int(responseRAW[2], responseRAW[3]);

    else
        return 0;
}

float MHZ19::getTransmittance(bool force)
{
    if (force == true)
        provisioning(RAWCO2);

    if (errorCode == RESULT_OK || force == false)
    {
        float calc = (float)bytes2int((responseRAW[2]), responseRAW[3]);

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
            byte buff = (responseTEMPLIM[4] - 38);

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
        return (responseTEMPLIM[4] - 38);
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
        float calc = (((responseTEMPUNLIM[2] - 8) * 1500) + ((responseTEMPUNLIM[3] * 100) * 1 / 17));
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
        return (int)bytes2int(responseSTAT[4], responseSTAT[5]);

    else
        return 0;
}

byte MHZ19::getAccuracy(bool force)
{
    if (force == true)
        provisioning(TEMPLIM);

    if (errorCode == RESULT_OK || force == false)
        return responseTEMPLIM[5];

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
            rVersion[i] = char(responseSTAT[i + 2]);
        }

    else
        memset(rVersion, 0, 4);
}

int MHZ19::getBackgroundCO2()
{
    provisioning(GETCALPPM);

    if (errorCode == RESULT_OK)
        return (int)bytes2int(responseSTAT[4], responseSTAT[5]);

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
        return (responseSTAT[3]);

    else
        return 0;
}

byte MHZ19::getLastResponse(byte bytenum)
{
    provisioning(GETLASTRESP);

    if (errorCode == RESULT_OK)
        return (responseSTAT[bytenum]);

    else
        return 0;
}

/*######################-Utility Functions-########################*/

void MHZ19::stablise()
{
    unsigned long timeStamp = millis();

    /* construct common command (133) */
    constructCommand(TEMPUNLIM);

    write(constructedCommand);

    while (receiveResponse(responseTEMPUNLIM, TEMPUNLIM) != RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
           #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Failed to verify connection(1) to sensor. Failed to stablise");   
            #elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(1) to sensor. Failed to stablise");
            #endif   

            return;
        }
    }

    /* construct & write last response command (162) */
    constructCommand(GETLASTRESP);
    write(constructedCommand);
    
    /* update timeStamp  for next comms iteration */ 
    timeStamp = millis();

    while (receiveResponse(responseSTAT, GETLASTRESP) != RESULT_OK)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD)
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Failed to verify connection(2) to sensor. Failed to stablise");   
            #elif MHZ19_ERRORS
            Serial.println("!ERROR: Failed to verify connection(2) to sensor. Failed to stablise");
            #endif
            
            return;
        }
    }

    /* compare CO2 response command(133) against, last response command (162)*/
    for (byte i = 2; i < 8; i++)
    {
        if (responseTEMPUNLIM[i] != responseSTAT[i])
        {
            #if defined (ESP32) && (MHZ19_ERRORS)
            ESP_LOGE(TAG_MHZ19, "Last response was not found, call back failed. Failed to stablise");   
            #elif MHZ19_ERRORS
            Serial.println("!ERROR: Last response was not found, call back failed. Failed to stablise");
            #endif

            return;
        }
    }
    return;
}

void MHZ19::autoCalibration(bool isON, byte ABCPeriod)
{
    ABCInterval = ABCPeriod;
    ABCInterval /= 2;
    ABCInterval *= 3.6e6;

    if (ABCPeriod && isON)
    {
        if(ABCPeriod >= 24)
            ABCPeriod = 160;            
        else
            ABCPeriod *= 6.7;
    }

    else if (isON)
        ABCPeriod = 160;
 
    ABCRepeat = !isON;     

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
    _isDec = isDec;
    printcomm = isPrintComm;
}

/*######################-Inernal Functions-########################*/

void MHZ19::provisioning(Command_Type commandtype, int inData)
{
    /* construct command */
    constructCommand(commandtype, inData);

    /* write to serial */
    write(constructedCommand);

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
    memset(constructedCommand, 0, 9);

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
        if (ABCRepeat == false)
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
        int2bytes(inData, &High, &Low);
        asemblecommand[3] = High;
        asemblecommand[4] = Low;
        break;
    case RANGE:
        int2bytes(inData, &High, &Low);
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
    asemblecommand[8] = checkSum(asemblecommand);

    /* copy bytes from asemblecommand to constructedCommand */
    memcpy(constructedCommand, asemblecommand, 9);
}

byte MHZ19::checkSum(byte inBytes[])
{
    byte i;
    byte crc = 0;
    for (i = 1; i < 8; i++)
    {
        crc += inBytes[i];
    }
    crc = 255 - crc;
    crc++;

    return crc;
}

void MHZ19::write(byte toSend[])
{
    /* for print communications */
    if (printcomm == true)
        printstream(toSend, true, errorCode);

    /* transfer to buffer */
    mySerial->write(toSend, 9); 
 
    /* send */
    mySerial->flush(); 
}

void MHZ19::handleResponse(Command_Type commandtype)
{
    if (constructedCommand[2] == Commands[2])      //compare commands byte
        receiveResponse(responseRAW, commandtype); //returns errornum, passes back response, inputs command

    else if (constructedCommand[2] == Commands[3])
        receiveResponse(responseTEMPUNLIM, commandtype);

    else if (constructedCommand[2] == Commands[4])
        receiveResponse(responseTEMPLIM, commandtype);

    else
        receiveResponse(responseSTAT, commandtype);
}

byte MHZ19::receiveResponse(byte inBytes[9], Command_Type commandnumber)
{
    /* loop escape */
    unsigned long timeStamp = millis();

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, 9);

    /* prepare errorCode */
    this->errorCode = RESULT_ERR_NULL;

    /* wait for response, allow for defined time before exit */
    while (mySerial->available() <= 0)
    {
        if (millis() - timeStamp >= TIMEOUT_PERIOD) 
        {
            #if defined (ESP32) && (MHZ19_ERRORS) 
            ESP_LOGW(TAG_MHZ19, "Timed out waiting for response");    
            #elif MHZ19_ERRORS
            Serial.println("!Error: Timed out waiting for response");
            #endif  

            this->errorCode = RESULT_ERR_TIMEOUT;
            return RESULT_ERR_TIMEOUT;
        }
    }
    
    /* response recieved, read buffer */
    mySerial->readBytes(inBytes, 9);

    if (errorCode == RESULT_ERR_TIMEOUT)
        return errorCode;

    byte crc = checkSum(inBytes);

    /* CRC error will not overide match error */
    if (inBytes[8] != crc)
        errorCode = RESULT_ERR_CRC;

    /* construct error code */
    if (inBytes[0] != constructedCommand[0] || inBytes[1] != constructedCommand[2])
        errorCode = RESULT_ERR_MATCH;

    /* if error has been assigned */
    if (errorCode == RESULT_ERR_NULL)
        errorCode = RESULT_OK;

    /* print results */
    if (printcomm == true)
        printstream(inBytes, false, errorCode);

    return errorCode;
}

void MHZ19::printstream(byte inBytes[9], bool isSent, byte pserrorCode)
{
    if (pserrorCode != RESULT_OK && isSent == false)
    {
        Serial.print("Recieved >> ");

        if (_isDec)
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

        if (_isDec)
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
}

void MHZ19::ABCCheck()
{  
    /* check timer interval if dynamic hours have passed and if ABC_OFF was set to true */
    if (((millis() - ABCRepeatTimer) >= (ABCInterval)) && (ABCRepeat == true))
    {
        /* update timer inerval */
        ABCRepeatTimer = millis();

        /* construct ABC_OFF command */
        constructCommand(ABC);

        /* write to serial */
        write(constructedCommand);
    }
}

void MHZ19::int2bytes(int inInt, byte *high, byte *low)
{
    *high = (byte)(inInt / 256);
    *low = (byte)(inInt % 256);

    return;
}

unsigned int MHZ19::bytes2int(byte high, byte low)
{
    unsigned int calc = ((unsigned int)high * 256) + (unsigned int)low;
 
    return calc;
}
