/* 
Description:
This is a example code for Sandbox Electronics' I2C/SPI to UART bridge module.
You can get one of those products on 
http://sandboxelectronics.com

Version:
V0.1

Release Date:
2014-02-16

Author:
Tiequan Shao          info@sandboxelectronics.com

Lisence:
CC BY-NC-SA 3.0

Please keep the above information when you use this code in your project. 
*/ 


#ifndef _SC16IS750_H_
#define _SC16IS750_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

/*
VDD VDD 0x90 (1001 000X)
VDD VSS 0x92 (1001 001X)
VDD SCL 0x94 (1001 010X)
VDD SDA 0x96 (1001 011X)
VSS VDD 0x98 (1001 100X)
VSS VSS 0x9A (1001 101X)
VSS SCL 0x9C (1001 110X)
VSS SDA 0x9E (1001 111X)
SCL VDD 0xA0 (1010 000X)
SCL VSS 0xA2 (1010 001X)
SCL SCL 0xA4 (1010 010X)
SCL SDA 0xA6 (1010 011X)
SDA VDD 0xA8 (1010 100X)
SDA VSS 0xAA (1010 101X)
SDA SCL 0xAC (1010 110X)
SDA SDA 0xAE (1010 111X)
*/

//Device Address

//A:VDD
//B:GND
//C:SCL
//D:SDA
#define     SC16IS750_ADDRESS_AA     (0X90) 
#define     SC16IS750_ADDRESS_AB     (0X92)
#define     SC16IS750_ADDRESS_AC     (0X94)
#define     SC16IS750_ADDRESS_AD     (0X96)
#define     SC16IS750_ADDRESS_BA     (0X98)
#define     SC16IS750_ADDRESS_BB     (0X9A)
#define     SC16IS750_ADDRESS_BC     (0X9C)
#define     SC16IS750_ADDRESS_BD     (0X9E)
#define     SC16IS750_ADDRESS_CA     (0XA0)
#define     SC16IS750_ADDRESS_CB     (0XA2)
#define     SC16IS750_ADDRESS_CC     (0XA4)
#define     SC16IS750_ADDRESS_CD     (0XA6)
#define     SC16IS750_ADDRESS_DA     (0XA8)
#define     SC16IS750_ADDRESS_DB     (0XAA)
#define     SC16IS750_ADDRESS_DC     (0XAC)
#define     SC16IS750_ADDRESS_DD     (0XAE)


//General Registers
#define     SC16IS750_REG_RHR        (0x00)
#define     SC16IS750_REG_THR        (0X00)
#define     SC16IS750_REG_IER        (0X01)
#define     SC16IS750_REG_FCR        (0X02)
#define     SC16IS750_REG_IIR        (0X02)
#define     SC16IS750_REG_LCR        (0X03)
#define     SC16IS750_REG_MCR        (0X04)
#define     SC16IS750_REG_LSR        (0X05)
#define     SC16IS750_REG_MSR        (0X06)
#define     SC16IS750_REG_SPR        (0X07)
#define     SC16IS750_REG_TCR        (0X06)
#define     SC16IS750_REG_TLR        (0X07)
#define     SC16IS750_REG_TXLVL      (0X08)
#define     SC16IS750_REG_RXLVL      (0X09)
#define     SC16IS750_REG_IODIR      (0X0A)
#define     SC16IS750_REG_IOSTATE    (0X0B)
#define     SC16IS750_REG_IOINTENA   (0X0C)
#define     SC16IS750_REG_IOCONTROL  (0X0E)
#define     SC16IS750_REG_EFCR       (0X0F)

//Special Registers
#define     SC16IS750_REG_DLL        (0x00)
#define     SC16IS750_REG_DLH        (0X01)

//Enhanced Registers
#define     SC16IS750_REG_EFR        (0X02)
#define     SC16IS750_REG_XON1       (0X04)
#define     SC16IS750_REG_XON2       (0X05)
#define     SC16IS750_REG_XOFF1      (0X06)
#define     SC16IS750_REG_XOFF2      (0X07)

//
#define     SC16IS750_INT_CTS        (0X80)
#define     SC16IS750_INT_RTS        (0X40)
#define     SC16IS750_INT_XOFF       (0X20)
#define     SC16IS750_INT_SLEEP      (0X10)
#define     SC16IS750_INT_MODEM      (0X08)
#define     SC16IS750_INT_LINE       (0X04)
#define     SC16IS750_INT_THR        (0X02)
#define     SC16IS750_INT_RHR        (0X01)

//Application Related 

#define     SC16IS750_CRYSTCAL_FREQ (14745600UL) 
//#define 	SC16IS750_CRYSTCAL_FREQ (1843200UL)	  
//#define     SC16IS750_CRYSTCAL_FREQ (16000000UL)    
//#define     SC16IS750_DEBUG_PRINT   (0)
#define     SC16IS750_PROTOCOL_I2C  (0)
#define     SC16IS750_PROTOCOL_SPI  (1)


class SC16IS750 : public Stream
{ 
    public:
        SC16IS750(uint8_t prtcl = SC16IS750_PROTOCOL_I2C);
        void begin(uint32_t baud, uint8_t SDA, uint8_t SDL, uint8_t addr = SC16IS750_ADDRESS_AD);                           
        int read();
        size_t write(uint8_t val);
        int available();
        void pinMode(uint8_t pin, uint8_t io);
        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);
		uint8_t ping();
	//	void setTimeout(uint32_t);
	//	size_t readBytes(char *buffer, size_t length);
		int peek();
		void flush();
		uint8_t GPIOGetPortState(void);
		uint8_t InterruptPendingTest(void);
		void    SetPinInterrupt(uint8_t io_int_ena);
		void    InterruptControl(uint8_t int_ena);
		void    ModemPin(uint8_t gpio); //gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
		void    GPIOLatch(uint8_t latch);
           
    private:
        uint8_t _SDA, _SDL;
        uint8_t device_address_sspin;
        uint8_t protocol;
	//	uint32_t timeout;
        int16_t SetBaudrate(uint32_t baudrate);
        uint8_t ReadRegister(uint8_t reg_addr);
        void    WriteRegister(uint8_t reg_addr, uint8_t val);
        void    SetLine(uint8_t data_length, uint8_t parity_select, uint8_t stop_length );
        void    GPIOSetPinMode(uint8_t pin_number, uint8_t i_o);
        void    GPIOSetPinState(uint8_t pin_number, uint8_t pin_state);
		
        uint8_t GPIOGetPinState(uint8_t pin_number);
        void    GPIOSetPortMode(uint8_t port_io);
        void    GPIOSetPortState(uint8_t port_state);
        void    ResetDevice(void);
                    
        void    __isr(void);
        void    FIFOEnable(uint8_t fifo_enable);
        void    FIFOReset(uint8_t rx_fifo);
        void    FIFOSetTriggerLevel(uint8_t rx_fifo, uint8_t length);
        uint8_t FIFOAvailableData(void);
        uint8_t FIFOAvailableSpace(void);
        void    WriteByte(uint8_t val);
        int     ReadByte(void);
        void    EnableTransmit(uint8_t tx_enable);
	//	int16_t readwithtimeout();
		int 	peek_buf;
		uint8_t peek_flag;
		
};

#endif