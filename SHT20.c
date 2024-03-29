/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
� Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com


//-----------------0.Documentation section--------------------//
Project : SHT20 - ATMega8 communication using I2C display with LCD 16x2
This program using ATMega8 I2C to read the temp and humility of SHT20
Then display the value through LED 16x2
Process: 	- Implementation LCD display  (OK)
			- Implementation I2C		  (OK)
			- Reading value of SHT20       
				a. Display receive value to LCD with TWI lib: Failed with TWI library, 3 receive data are the same
				b. Display receive value to LCD with I2C lib
Version : 1.0
Date    : 7/03/2019
Author  : Tran Minh Thuan (ItachiVN)
Company : Viet Mold Machine

Chip type               : ATmega8A
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/
//-----------------1. Pre-processor Directives Section --------------------//
#include <mega8.h>
#include <alcd.h>       // Alphanumeric LCD library
#include <delay.h>      // Delay library
#include <twi.h>        // TWI library
#include <stdio.h>
#include <stdlib.h>
//-----------------2. Global declaration section--------------------//
#define ERROR_I2C_TIMEOUT                     998
#define ERROR_BAD_CRC                         999
#define SHT20_ADDR	                          0x40
#define TRIGGER_TEMP_MEASURE_HOLD             0xE3
#define TRIGGER_HUMD_MEASURE_HOLD             0xE5 
#define TRIGGER_TEMP_MEASURE_NOHOLD           0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD           0xF5
#define WRITE_USER_REG                        0xE6
#define READ_USER_REG                         0xE7
#define SOFT_RESET                            0xFE
#define USER_REGISTER_RESOLUTION_MASK         0x81
#define USER_REGISTER_RESOLUTION_RH12_TEMP14  0x00
#define USER_REGISTER_RESOLUTION_RH8_TEMP12   0x01
#define USER_REGISTER_RESOLUTION_RH10_TEMP13  0x80
#define USER_REGISTER_RESOLUTION_RH11_TEMP11  0x81
#define USER_REGISTER_END_OF_BATTERY          0x40
#define USER_REGISTER_HEATER_ENABLED          0x04
#define USER_REGISTER_DISABLE_OTP_RELOAD      0x02
#define MAX_WAIT                              100
#define DELAY_INTERVAL                        10
#define SHIFTED_DIVISOR                       0x988000
#define MAX_COUNTER                           (MAX_WAIT/DELAY_INTERVAL) // 100/10=10
/* --------------------Global variable----------------------*/
unsigned char Data_Recv[5]={0};			//Pointer to data receive of slave I2C
unsigned char Data_Disp[16]={0};
unsigned int System_fail=0;             //Monitor CRC fails
/* --------------------Function decleration---------------------*/
unsigned int Read_SHT20(unsigned char u8cmd);
unsigned char CheckCRC(unsigned int message_from_sensor, unsigned char check_value_from_sensor);
float ReadTemperature_SHT20(void);
float ReadHumidity_SHT20(void);
void Display_Number(unsigned char numb);
/* --------------------Function definition---------------------*/
void main(void)
{
    twi_master_init(100); //TWI initialization 100 kHz
    lcd_init(16);         //LCD initialization 16x2
    #asm("sei")           //Global enable interrupts
	while (1)
    {
        float Temp=0;
        float Humi=0;
        Temp=ReadTemperature_SHT20();
        if(Temp!=0)
        {
            lcd_gotoxy(0,0);
            lcd_puts("Nhiet do: ");
            ftoa(Temp,3,Data_Disp);                           
            lcd_puts(Data_Disp);
        } 
        else System_fail++;
        Humi= ReadHumidity_SHT20();
        if(Humi!=0)
        {  
            lcd_gotoxy(0,1);
            lcd_puts("Do am: ");
            ftoa(Humi,3,Data_Disp);                           
            lcd_puts(Data_Disp);
        }     
        else System_fail++;
        Display_Number(System_fail);
        delay_ms(100); 
          
    }
}
/* Read SHT by sending proper command
   Input: 8-bit command (Table 6- Datasheet)
   Output: 14 -bit received from SHT20 or 0 if wrong data
   Note:  
*/
unsigned int Read_SHT20(unsigned char u8cmd)
{
    unsigned char Command_Send[5]={0};	  //Pointer to data array send to Slave I2C
    unsigned int u16_datarecv=0;   
    Command_Send[0]=u8cmd; 
    twi_master_trans(SHT20_ADDR,&Command_Send[0],2,0,0);                //send request
    delay_ms(100);
    if(twi_master_trans(SHT20_ADDR,&Command_Send[2],0,&Data_Recv[0],3)) //if data collect is true
    {   
        u16_datarecv=(Data_Recv[0]<<8)|Data_Recv[1];  
        if(CheckCRC(u16_datarecv,Data_Recv[2])!=0) return 0;            //return 0 if CRC is wrong
        else u16_datarecv=u16_datarecv&0xFFFC;                          //14-bit resolution
    } 
    return u16_datarecv;
}

/* Read SHT20 Temperature
   Input: None
   Output: Float number of Temperature
   Note:  
*/
float ReadTemperature_SHT20(void)
{
    float tempTemperature=0;
    float realTemperature=0;
    unsigned int rawTemperature = Read_SHT20(TRIGGER_TEMP_MEASURE_NOHOLD);
    if(rawTemperature!=0) //No error CRC occur
    {
         tempTemperature = rawTemperature * (175.72 / 65536.0);
         realTemperature = tempTemperature - 46.85;
    }
    return (realTemperature);
}

/* Read SHT20 Humidity
   Input: None
   Output: Float number of Humidity
   Note:  
*/
float ReadHumidity_SHT20(void)
{
    float tempRH=0;
    float rh=0;
    unsigned int rawHumidity = Read_SHT20(TRIGGER_HUMD_MEASURE_NOHOLD);
    if(rawHumidity!=0)
    {
        tempRH = rawHumidity * (125.0 / 65536.0);
        rh = tempRH - 6.0;
    }
    return (rh);
}

/* Display_Number through LCD
   Input: numb to display
   Output: none
   Note: 0<numb<99
*/ 
void Display_Number(unsigned char numb)
{
    unsigned char tens=0,digit=0;
    tens=numb/10;
    digit=numb%(tens*10);
    lcd_gotoxy(14,1);
    lcd_putchar(tens+'0');
    lcd_gotoxy(15,1);
    lcd_putchar(digit+'0');   
} 

unsigned char CheckCRC(unsigned int message_from_sensor, unsigned char check_value_from_sensor)
{
    unsigned long int remainder=0,divsor=0;
    unsigned char i=0;
    remainder = (unsigned long int)message_from_sensor << 8;
    remainder |= check_value_from_sensor;
    divsor = SHIFTED_DIVISOR;
    for(i = 0 ; i < 16 ; i++){
        if(remainder & (unsigned long int)1 << (23 - i))
		{
            remainder ^= divsor;
        }
        divsor >>= 1;
    }
    return (unsigned char)remainder;
}
 
