/*************************************************** 
  Written by: Jonathan Dempsey JDWifWaf@gmail.com

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
#include "esp32-hal-log.h"

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

MHZ19::MHZ19(byte rx, byte tx, byte s) : _rx(rx), _tx(tx), _s(s) {}

void MHZ19::begin()
{
    /* establish connection */
    stablise();

    /* check if successful */
    if (errorCode != RESULT_OK)
        ESP_LOGE(TAG_MHZ19, "<!ERROR> Failed to establish connection");
}

/*########################-Set Functions-##########################*/

void MHZ19::setRange(int range)
{
    if (range > 65000)
    {
        ESP_LOGE(TAG_MHZ19, "Invalid Range value (0 - 65000)");
        return;
    }

    else
        provisioning(RANGE, range);
}

void MHZ19::setSpan(int span)
{
    if (isZeroLast == false)
        ESP_LOGW(TAG_MHZ19, "Zero Calibration should be sent before Span");

    if (span > 10000)
    {
        ESP_LOGE(TAG_MHZ19, "Invalid Span value (0 - 10000)");
    }
    else
    {
        provisioning(SPANCAL);
        isZeroLast = false;
    }

    return;
}

/*########################-Get Functions-##########################*/

int MHZ19::getCO2(bool force, bool isunLimited)
{
    if (isunLimited == true)
    {
        if (force == true)
            provisioning(TEMPUNLIM);

        if (errorCode == RESULT_OK || force == false)
        {
            return bytes2int(responseTEMPUNLIM[4], responseTEMPUNLIM[5]);
        }
    }

    else if (isunLimited == false)
    {
        if (force == true)
            provisioning(TEMPLIM);

        if (errorCode == RESULT_OK || force == false)
        {
            return bytes2int(responseTEMPLIM[2], responseTEMPLIM[3]);
        }
    }

    return 0;
}

int MHZ19::getCO2Raw(bool force)
{
    if (force == true)
        provisioning(RAWCO2);

    if (errorCode == RESULT_OK || force == false)
    {
        return bytes2int((responseRAW[2]), responseRAW[3]);
    }

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
        return ((calc * 100) / 40000);
    }

    else
        return 0;
}

float MHZ19::getTemperature(bool force, bool isunLimited)
{
    if (isunLimited == true)
    {
        if (force == true)
            provisioning(TEMPUNLIM);

        if (errorCode == RESULT_OK || force == false)
        {
            /* Byte 2 (counting 0) holds magnitudes of 10, which is
            transformed to the correct magnitude by deducting 8 
            (this also handles negatives). 
            
            Byte 3 holds a value of where each increment of 17 
            (in bytes) gives 1 degrees celsius. 
            
            Both calulations are multipled by 100 for resolution. */

            float calc = (((responseTEMPUNLIM[2] - 8) * 1500) + ((responseTEMPUNLIM[3] * 100) * 1 / 17));
            calc /= 100;
            return calc;
        }
    }

    else if (isunLimited == false)
    {
        if (force == true)
            provisioning(TEMPLIM);

        if (errorCode == RESULT_OK || force == false)
            return (responseTEMPLIM[4] - 38);
    }

    return -273.15;
}

int MHZ19::getRange()
{
    /* check get range was recieved */
    provisioning(GETRANGE);

    if (errorCode == RESULT_OK)
        /* convert MH-Z19 memory value and return */
        return bytes2int(responseSTAT[4], responseSTAT[5]);

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
        return bytes2int(responseSTAT[4], responseSTAT[5]);

    else
        return 0;
}

byte MHZ19::getTempAdjustment()
{
    provisioning(GETEMPCAL);

    /* 40 is shown here, however this library deductes -2 
     when using temperature function as it is inaccurate, 
     (confirmed by command 133) */

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
    int timeout = 0;

    ESP_LOGD(TAG_MHZ19, "Waiting for the OK Status..");

    /* construct common command (133) */
    constructCommand(TEMPUNLIM);

    write(constructedCommand);

    while (receiveResponse(responseTEMPUNLIM, TEMPUNLIM) != RESULT_OK)
    {
        delay(500);
        timeout++;
        if (timeout >= 10)
        {
            ESP_LOGE(TAG_MHZ19, "EERROR, failed to recieve OK (1)");
            return;
        }
    }

    /* construct last response command (162) */
    constructCommand(GETLASTRESP);

    write(constructedCommand);

    while (receiveResponse(responseSTAT, GETLASTRESP) != RESULT_OK)
    {
        delay(500);
        timeout++;
        if (timeout >= 10)
        {
            ESP_LOGE(TAG_MHZ19, "EERROR, failed to recieve OK (2)");
            return;
        }
    }

    /* compare CO2 response command(133) against, last response command (162)*/
    for (byte i = 2; i < 8; i++)
    {
        if (responseTEMPUNLIM[i] != responseSTAT[i])
        {
            ESP_LOGE(TAG_MHZ19, "EERROR, callback failed");
        }
    }

    ESP_LOGD(TAG_MHZ19, "Stabalised");
    return;
}

void MHZ19::autoCalibration(bool isON, byte ABCPeriod)
{
    if (ABCPeriod >= 24)
        ABCPeriod = 180;
    else
        ABCPeriod = (ABCPeriod * 7.5); // 7.5 represents the average days in a 4 week month

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

    if (errorCode == RESULT_OK)
        isZeroLast = true;
}

void MHZ19::recoveryReset()
{
    provisioning(RECOVER);
}

/*######################-Inernal Functions-########################*/

void MHZ19::provisioning(Command_Type commandtype, int inData)
{
    /* Check if ABC_OFF needs to run */
    ABCCheck();

    /* construct command */
    constructCommand(commandtype, inData);

    /* write to serial */
    write(constructedCommand);

    /*return response */
    handleResponse(commandtype);
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
    /* open communications */
    HardwareSerial hserial(_s);
    hserial.begin(BAUDRATE, SCONFIG, _rx, _tx);

    /* print for debug */
    printstream(toSend, true, errorCode);

    /* transfer to buffer */
    hserial.write(toSend, 9);

    /* send */
    hserial.flush();
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
    byte TimeOut = 0;

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, 9);

    /* prepare errorCode */
    errorCode = RESULT_ERR_NULL;

    /* open communications */
    HardwareSerial hserial(_s);
    hserial.begin(BAUDRATE, SCONFIG, _rx, _tx);

    /* wait for response, allow for defined time before exit */
    while (hserial.available() <= 0)
    {
        delay(WAIT_READ_DELAY);
        TimeOut++;
        if (TimeOut >= 50)
        {
            ESP_LOGW(TAG_MHZ19, "Timed Out!");
            errorCode = RESULT_ERR_TIMEOUT;
            return RESULT_ERR_TIMEOUT;
        }
    }

    /* response recieved, read buffer */
    hserial.readBytes(inBytes, 9);
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

    /* print results to ESP_LOG */
    printstream(inBytes, false, errorCode);

    return errorCode;
}

void MHZ19::printstream(byte inbytes[9], bool isSent, byte pserrorCode)
{
    if (pserrorCode != RESULT_OK && isSent == false)
    {
        if (isDec)
            ESP_LOGE(TAG_MHZ19, "Recieved >> %d %d %d %d %d %d %d %d %d ERROR Code: %d",
                     inbytes[0], inbytes[1], inbytes[2], inbytes[3], inbytes[4], inbytes[5], inbytes[6], inbytes[7], inbytes[8],
                     pserrorCode);
        else
            ESP_LOGE(TAG_MHZ19, "Recieved >> %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x ERROR Code: %d",
                     inbytes[0], inbytes[1], inbytes[2], inbytes[3], inbytes[4], inbytes[5], inbytes[6], inbytes[7], inbytes[8],
                     pserrorCode);
    }

    else
    {
        if (isDec)
            ESP_LOGD(TAG_MHZ19, "%s %d %d %d %d %d %d %d %d %d PASS", isSent ? "Sent << " : "Recieved >> ",
                     inbytes[0], inbytes[1], inbytes[2], inbytes[3], inbytes[4], inbytes[5], inbytes[6], inbytes[7], inbytes[8]);
        else

            ESP_LOGD(TAG_MHZ19, "%s %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x %#03x PASS", isSent ? "Sent << " : "Recieved >> ",
                     inbytes[0], inbytes[1], inbytes[2], inbytes[3], inbytes[4], inbytes[5], inbytes[6], inbytes[7], inbytes[8]);
    }
}

void MHZ19::ABCCheck()
{
    /* check timer interval if 12 hours have passed and if ABC_OFF was set to true */
    if (((millis() - ABCRepeatTimer) >= (4.32e8)) && (ABCRepeat == true))
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

int MHZ19::bytes2int(byte high, byte low)
{
    int _high = (int)(high);
    int _low = (int)(low);

    return (256 * _high) + _low;
}
