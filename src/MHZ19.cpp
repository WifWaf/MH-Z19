/* ----------------------------------------------------------------
    Contact: JDWifWaf@gmail.com | Version: 1.6.0 | License: LGPLv3
   ---------------------------------------------------------------- */

#include "MHZ19.h"

/*#########################-DIRECTIVES-##############################*/
// Utility ---------------------------------- //
#define INT_LIMIT                32767     // Maximum decimal value for int
#define ULIM_BUF                 0         // Filter buffer index for CO2 unlimited
#define LIM_BUF                  1         // Filter buffer index for CO2 limited

// Config ------------------------------------ //
#define MHZ19_ABC_PER_OFF        0x00
#define MHZ19_ABC_PER_DEF        0xA0
#define MHZ19_ABC_EN             0x00
#define MHZ19_ABC_DIS            0x10
#define MHZ19_FILTER_EN          0x08
#define MHZ19_FILTER_DIS         0x00
#define MHZ19_FILTER_CLR_EN      0x04
#define MHZ19_FILTER_CLR_DIS     0x00
#define MHZ19_COMM_PRNT_EN       0x02
#define MHZ19_COMM_PRNT_DIS      0x00
#define MHZ19_DEC_MODE           0x01
#define MHZ19_HEX_MODE           0X00

// Commands --------------------------------- //
#define MHZ19_COM_REC            0x78         // 0 Recovery Reset        Changes operation mode and performs MCU reset
#define MHZ19_COM_ABC            0x79         // 1 ABC Mode ON/OFF       Turns ABC logic on or off (b[3] == 0xA0 - on, 0x00 - off)
#define MHZ19_COM_ABC_STATUS     0x7D         // 2 Get ABC logic status  (1 - enabled, 0 - disabled)	
#define MHZ19_COM_CO2_RAW        0X84         // 3 Raw CO2
#define MHZ19_COM_CO2_UNLIM      0x85         // 4 Temp float, CO2 Unlimited
#define MHZ19_COM_CO2_LIM        0x86         // 5 Temp integer, CO2 limited
#define MHZ19_COM_CAL_ZERO       0x87         // 6 Zero Calibration
#define MHZ19_COM_CAL_SPAN       0x88         // 7 Span Calibration
#define MHZ19_COM_CAL_RANGE      0X99         // 8 Range
#define MHZ19_COM_RANGE          0x9B         // 9 Get Range
#define MHZ19_COM_CO2_BACK       0X9C         // 10 Get Background CO2
#define MHZ19_COM_FIRMWARE       0xA0         // 11 Get Firmware Version
#define MHZ19_COM_LAST           0XA2         // 12 Get Last Response
#define MHZ19_COM_TEMP_CAL       0xA3         // 13 Get Temp Calibration

// Co-Commands ----------------------------- //
#define MHZ19_ABC_PERIOD_OFF    0x00
#define MHZ19_ABC_PERIOD_DEF    0xA0

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
    if (range > MHZ19_LIB_MAX_RANGE)
    {
        #if defined (ESP32) && (MHZ19_ERRORS)
        ESP_LOGE(TAG_MHZ19, "Invalid Range value (0 - 65000)");
        #elif MHZ19_ERRORS
        Serial.println("!ERROR: Invalid Range value (0 - 65000)");
        #endif 

        return;
    }

    else
        provisioning(MHZ19_COM_RANGE, range);
}

void MHZ19::zeroSpan(int span)
{
    if (span > MHZ19_LIB_MAX_SPAN)
    {
        #if defined (ESP32) && (MHZ19_ERRORS)
        ESP_LOGE(TAG_MHZ19, "Invalid Span value (0 - 10000)");   
        #elif MHZ19_ERRORS
        Serial.println("!ERROR: Invalid Span value (0 - 10000)");
        #endif 
    }
    else
        provisioning(MHZ19_COM_CAL_SPAN, span);
 
    return;
}

void MHZ19::setFilter(bool isON, bool isCleared)
{
    (isON) ? (this->mem.cfg |= MHZ19_FILTER_EN) : (this->mem.cfg &= ~MHZ19_FILTER_EN);
    (isCleared) ? (this->mem.cfg|= MHZ19_FILTER_CLR_EN) : (this->mem.cfg &= ~MHZ19_FILTER_CLR_EN);
}

/*########################-Get Functions-##########################*/

int MHZ19::getCO2(bool isunLimited)
{ 
    (isunLimited) ? provisioning(MHZ19_COM_CO2_UNLIM, 0) : provisioning(MHZ19_COM_CO2_LIM, 0);

    if (this->errorCode == RESULT_OK)
    {
        unsigned int CO2 = 0;
        CO2 = (isunLimited) ? makeInt(this->mem.block.in[4], this->mem.block.in[5]): makeInt(this->mem.block.in[2], this->mem.block.in[3]);

        if(this->mem.cfg & MHZ19_FILTER_EN)
        {
           return filter(isunLimited, CO2);
        }
        else
        {
            if(CO2 > INT_LIMIT)
                CO2 = INT_LIMIT;

            return CO2;
        } 
    }
    return 0;
}

int MHZ19::filter(bool isunLimited, unsigned int CO2)
{
    bool trigFilter = false;           // Keep track of whether any conditions for filter trigger is met
    unsigned int co2[2] = {0, 0};      // Neeed to comapre both CO2 returned bytes. {ulim, lim}.

    if(isunLimited)                    // Filter was must call the opposest unlimited/limited command to work
    {                 
        co2[ULIM_BUF] = CO2;     // save last co2 reading
        provisioning(MHZ19_COM_CO2_LIM, 0);      // request opposite co2 command
        co2[LIM_BUF] = makeInt(this->mem.block.in[2], this->mem.block.in[3]);     // save opposite co2 command
    }
    else
    {
        co2[LIM_BUF] = CO2;
        provisioning(MHZ19_COM_CO2_UNLIM, 0);
        co2[ULIM_BUF] = makeInt(this->mem.block.in[4], this->mem.block.in[5]);
    }

    // Limited CO2 stays at 410ppm during reset, so comparing unlimited which instead
    // shows an abormal value, reset duration can be found. Limited CO2 ppm returns to "normal"
    // after reset.

    if(this->mem.cfg & MHZ19_FILTER_CLR_EN)             // return to be cleared to 0
    {
        if(co2[ULIM_BUF] > INT_LIMIT || co2[LIM_BUF] > INT_LIMIT 
        || ((co2[ULIM_BUF] - co2[LIM_BUF] >= 10) && co2[LIM_BUF] == 410))  // CO2 limited, stays at 410ppm on reset
        {      
            this->errorCode = RESULT_FILTER;  // Upate filter error
            return 0;
        }     
    }
    else      // return not be cleared to 0
    {
        if(co2[ULIM_BUF] > INT_LIMIT)        // Catch out of int range values
        {
            co2[ULIM_BUF] = INT_LIMIT;
            trigFilter = true;
        }
        if(co2[LIM_BUF] > INT_LIMIT)
        {
            co2[LIM_BUF] = INT_LIMIT;
            trigFilter = true;
        }
        if(((co2[ULIM_BUF]  - co2[LIM_BUF])  >= 10) && (co2[LIM_BUF] == 410)) // CO2 limited, stays at 410ppm on reset
            trigFilter = true;

        if(trigFilter)
        {              
            this->errorCode = RESULT_FILTER;  // Upate filter error
        }
    }

    return (isunLimited) ? co2[ULIM_BUF] : co2[LIM_BUF];     
}

unsigned int MHZ19::getCO2Raw()
{
    provisioning(MHZ19_COM_CO2_RAW);

    return (this->errorCode == RESULT_OK) ? makeInt(this->mem.block.in[2], this->mem.block.in[3]) : 0;
}

float MHZ19::getTransmittance()
{
    provisioning(MHZ19_COM_CO2_RAW);

    if (this->errorCode == RESULT_OK )
    {
        float calc = (float)makeInt((this->mem.block.in[2]), this->mem.block.in[3]);

        return (calc * 100 / 35000); //  (calc * to percent / x(raw) zero)
    }
    else
        return 0;
}

float MHZ19::getTemperature(bool isFloat)
{
    provisioning(MHZ19_COM_CO2_LIM);

    if(isFloat)
    {
        static byte baseTemp = 0;    // Temp has a common base value when determing float   
        static byte offSet = 0;      // Offset is the value to be removed form the final value
        static bool isSet = false;   // Flag for if the base temp was recorded in previous iterations

        if(!isSet)
        {
            baseTemp = (this->mem.block.in[4] - MHZ19_LIB_TEMP_ADJUST);
            offSet -= (byte)getTemperatureOffset();
            isSet = true;
        }   

        if(this->errorCode == RESULT_OK)
        {
           float buff = baseTemp;
           buff += offSet;
           return buff;
        }
    }
    
    else if(!isFloat)
    {
        if (this->errorCode == RESULT_OK)
            return (this->mem.block.in[4] - MHZ19_LIB_TEMP_ADJUST);
    }
    
    return -273.15;    
}
 
float MHZ19::getTemperatureOffset()
{
    provisioning(MHZ19_COM_CO2_UNLIM);

    if (this->errorCode == RESULT_OK)
    {
        /* Value appears to be for CO2 offset (useful for deriving CO2 from raw?) */
        /* Adjustments and calculations are based on observations of temp behavour */
        float calc = (((this->mem.block.in[2] - 8) * 1500) + ((this->mem.block.in[3] * 100) * 1 / 17));
        calc /= 100;
        return calc;
    }

    return -273.15;
} 

int MHZ19::getRange()
{
    /* check get range was recieved */
    provisioning(MHZ19_COM_RANGE);

    return (this->errorCode == RESULT_OK) ? (int)makeInt(this->mem.block.in[4], this->mem.block.in[5]) : 0;
}

byte MHZ19::getAccuracy()
{
    provisioning(MHZ19_COM_CO2_LIM);

    return (this->errorCode == RESULT_OK) ? this->mem.block.in[5] : 0; //GetRange byte 7
}

byte MHZ19::getPWMStatus()
{
    provisioning(MHZ19_COM_CO2_BACK);

    return (this->errorCode == RESULT_OK) ? (this->mem.block.in[3]) : 0;
}

void MHZ19::getVersion(char rVersion[])
{
    provisioning(MHZ19_COM_FIRMWARE);

    if (this->errorCode == RESULT_OK)
    {
        for (byte i = 0; i < 4; i++)
        {
            rVersion[i] = char(this->mem.block.in[i + 2]);
        }
    }
    else
        memset(rVersion, 0, 4);
}

int MHZ19::getBackgroundCO2()
{
    provisioning(MHZ19_COM_CO2_BACK);

    return (this->errorCode == RESULT_OK) ? (int)makeInt(this->mem.block.in[4], this->mem.block.in[5]) : 0;
}

byte MHZ19::getTempAdjustment()
{
    provisioning(MHZ19_COM_TEMP_CAL);

    /* 40 is returned here, however this library can use TEMP_ADJUST */

    return (this->errorCode == RESULT_OK) ? (this->mem.block.in[3]) : 0;
}

byte MHZ19::getLastResponse(byte num)
{
    provisioning(MHZ19_COM_LAST);

    return (this->errorCode == RESULT_OK) ? (this->mem.block.in[num]) : 0;
}

bool MHZ19::getABC()
{
    /* check get ABC logic status (1 - enabled, 0 - disabled) */
    provisioning(MHZ19_COM_ABC_STATUS);
    /* convert MH-Z19 memory value and return */
    return (this->errorCode == RESULT_OK) ? this->mem.block.in[7] : 1;
}

/*######################-Utility Functions-########################*/

void MHZ19::verify()
{
    unsigned long timeStamp = millis();
    byte compare[MHZ19_LIB_DATA_LEN];

    /* construct common command (133) */
    constructCommand(MHZ19_COM_CO2_UNLIM);

    write(compare);

    while (read(this->mem.block.in, MHZ19_COM_CO2_UNLIM) != RESULT_OK)
    {
        if (millis() - timeStamp >= MHZ19_LIB_TIMEOUT_PERIOD)
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
    constructCommand(MHZ19_COM_LAST);
    write(this->mem.block.out);
    
    /* update timeStamp  for next comms iteration */ 
    timeStamp = millis();

    while (read(this->mem.block.in, MHZ19_COM_LAST) != RESULT_OK)
    {
        if (millis() - timeStamp >= MHZ19_LIB_TIMEOUT_PERIOD)
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
        if (compare[i] != this->mem.block.in[i])
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

void MHZ19::autoCalibration(bool isON)
{
    /* If ABC is ON */
    if(isON)
    {
        this->mem.cfg &= ~MHZ19_ABC_DIS;  // Clears disable bit
        provisioning(MHZ19_COM_ABC, MHZ19_ABC_PERIOD_DEF);
    } 
    /* If ABC is OFF */
    else 
    {
        this->mem.cfg |= MHZ19_ABC_DIS; 
        provisioning(MHZ19_COM_ABC, MHZ19_ABC_PERIOD_OFF);  // Set command byte to Zero to match command format.
    }
             
    
}

void MHZ19::calibrate()
{
    provisioning(MHZ19_COM_CAL_ZERO);
}

void MHZ19::recoveryReset()
{
    provisioning(MHZ19_COM_REC);
}

void MHZ19::printCommunication(bool isDec, bool isPrintComm)
{
   (isDec) ? (this->mem.cfg |= MHZ19_DEC_MODE) : (this->mem.cfg &= ~MHZ19_DEC_MODE);
   (isPrintComm) ? (this->mem.cfg|= MHZ19_COMM_PRNT_EN) : (this->mem.cfg &= ~MHZ19_COMM_PRNT_EN);
}

/*######################-Inernal Functions-########################*/

void MHZ19::provisioning(byte comm, int inData)
{
    /* construct command */
    constructCommand(comm, inData);

    /* write to serial */
    write(this->mem.block.out);

    /*return response */
    read(this->mem.block.in, comm);	

    /* Check if ABC_OFF needs to run */
    ABCCheck();
}

void MHZ19::constructCommand(byte comm, int inData)
{
    memset(this->mem.block.out, 0, MHZ19_LIB_DATA_LEN);

    /* set address to 'any' */
    this->mem.block.out[0] = 0xFF; ///(0xFF) 255/FF means 'any' address (where the sensor is located)

    /* set register */
    this->mem.block.out[1] = 0x01; //(0x01) arbitrary byte number

    /* set command */
    this->mem.block.out[2] = comm; // assign command value

    switch(comm)
    {
        case  MHZ19_COM_REC:
            break;
        case MHZ19_COM_ABC:
            if (!(this->mem.cfg & MHZ19_ABC_DIS))
                this->mem.block.out[3] = inData;
            break;
        case MHZ19_COM_CO2_RAW:
            break;
        case MHZ19_COM_CO2_UNLIM:
            break;
        case MHZ19_COM_CO2_LIM:
            break;
        case MHZ19_COM_CAL_ZERO:
            if (inData)
                this->mem.block.out[6] = inData;
            break;
        case MHZ19_COM_CAL_SPAN:
            makeByte(inData, this->mem.block.out[3], this->mem.block.out[4]);
            break;
        case MHZ19_COM_CAL_RANGE:
            makeByte(inData, this->mem.block.out[6], this->mem.block.out[7]);
            break;
        case MHZ19_COM_RANGE:
            break;
        case MHZ19_COM_CO2_BACK:
            break;
        case MHZ19_COM_FIRMWARE:
            break;
        case MHZ19_COM_TEMP_CAL:
            break;
        case MHZ19_COM_LAST:
            break;
    }

    /* set checksum */
    this->mem.block.out[8] = getCRC(this->mem.block.out);
}

void MHZ19::write(byte toSend[])
{
    /* for print communications */
    if (this->mem.cfg & MHZ19_COMM_PRNT_EN)
        printstream(toSend, true, this->errorCode);

    /* transfer to buffer */
    mySerial->write(toSend, MHZ19_LIB_DATA_LEN); 
 
    /* send */
    mySerial->flush(); 
}

byte MHZ19::read(byte inBytes[], byte comm)
{
    /* loop escape */
    unsigned long timeStamp = millis();

    /* prepare memory array with unsigned chars of 0 */
    memset(inBytes, 0, MHZ19_LIB_DATA_LEN);

    /* prepare errorCode */
    this->errorCode = RESULT_NULL;

    /* wait until we have exactly the 9 bytes reply (certain controllers call read() too fast) */
    while (mySerial->available() < MHZ19_LIB_DATA_LEN)
    {
        if (millis() - timeStamp >= MHZ19_LIB_TIMEOUT_PERIOD) 
        {
            #if defined (ESP32) && (MHZ19_ERRORS) 
            ESP_LOGW(TAG_MHZ19, "Timed out waiting for response");    
            #elif MHZ19_ERRORS
            Serial.println("!Error: Timed out waiting for response");
            #endif  

            this->errorCode = RESULT_TIMEOUT;   
                     
            /* clear incomplete 9 byte values, limit is finite */
            cleanUp(mySerial->available());

            //return error condition
            return RESULT_TIMEOUT;
        }
    }
    
    /* response recieved, read buffer */
    mySerial->readBytes(inBytes, MHZ19_LIB_DATA_LEN);

    if (this->errorCode == RESULT_TIMEOUT)
        return this->errorCode;

    byte crc = getCRC(inBytes);

    /* CRC error will not overide match error */
    if (inBytes[8] != crc)
        this->errorCode = RESULT_CRC;

    /* construct error code */
    if (inBytes[0] != this->mem.block.out[0] || inBytes[1] != this->mem.block.out[2])
    {
       /* clear rx buffer for deysnc correction */
        cleanUp(mySerial->available());
        this->errorCode = RESULT_MATCH;
    }

    /* if error has been assigned */
    if (this->errorCode == RESULT_NULL)
        this->errorCode = RESULT_OK;

    /* print results */
    if (this->mem.cfg & MHZ19_COMM_PRNT_EN)
        printstream(inBytes, false, this->errorCode);

    return this->errorCode;
}

void MHZ19::printstream(byte inBytes[], bool isSent, byte pserrorCode)
{
    if(pserrorCode != RESULT_OK && isSent == false)
    {
        Serial.print("Recieved >> ");
        if(this->mem.cfg & MHZ19_DEC_MODE)
        {
            Serial.print("DEC: "); 
            for(uint8_t i = 0; i < MHZ19_LIB_DATA_LEN; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }
        else
        {
            for(uint8_t i = 0; i < MHZ19_LIB_DATA_LEN; i++)
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

        if(this->mem.cfg & MHZ19_DEC_MODE)
        {
            Serial.print("DEC: ");
            for (uint8_t i = 0; i < MHZ19_LIB_DATA_LEN; i++)
            {
                Serial.print(inBytes[i]);
                Serial.print(" ");
            }
        }
        else
        {
            for(uint8_t i = 0; i < MHZ19_LIB_DATA_LEN; i++)
            {
                Serial.print("0x");
                if(inBytes[i] < 16)
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
	static unsigned long timer_abc;          // timer for tracking the next ABC cycle skip
    /* check timer interval if dynamic hours have passed and if ABC_OFF was set to true */
	if (((millis() - timer_abc) >= MHZ19_LIB_ABC_INTERVAL) && (this->mem.cfg & MHZ19_ABC_DIS))
	{
		/* update timer inerval */
		timer_abc = millis();
		
		/* construct command to skip next ABC cycle */
		provisioning(MHZ19_COM_ABC, MHZ19_ABC_PERIOD_OFF);
	}
}

void MHZ19::cleanUp(uint8_t cnt)
{
    uint8_t eject = 0;
    for(uint8_t x = 0; x < cnt; x++)
    {
        eject = mySerial->read();
        #if defined (ESP32) && (MHZ19_ERRORS) 
        ESP_LOGW(TAG_MHZ19, "Clearing Byte: %d", eject);  
        #elif MHZ19_ERRORS
        Serial.print("!Warning: Clearing Byte: "); Serial.println(eject);
        #endif     
    }
}

void MHZ19::makeByte(int inInt, byte high, byte low)
{
    high = (byte)(inInt / 256);
    low = (byte)(inInt % 256);

    return;
}

unsigned int MHZ19::makeInt(byte high, byte low)
{
    unsigned int calc = ((unsigned int)high * 256) + (unsigned int)low;
 
    return calc;
}
