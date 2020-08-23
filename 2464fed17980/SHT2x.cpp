/*
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
 
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
 
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

//=============================================================================
// Port of the SHT21 example code from Sensirion
// port by Serge Sozonoff
//=============================================================================

#include "SHT2x.h"
#include "mbed.h"

SHT2x::SHT2x (PinName p_sda, PinName p_scl) : i2c(p_sda, p_scl)  {
}

//==============================================================================//
int SHT2x::SHT2x_CheckCrc(int data[], int nbrOfBytes, int checksum)
//==============================================================================//
{
    int crc = 0;
    int byteCtr;
    //calculates 8-Bit checksum with given polynomial
    for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr) {
        crc ^= (data[byteCtr]);
        for (int bit = 8; bit > 0; --bit) {
            if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else crc = (crc << 1);
        }
    }
    if (crc != checksum) return CHECKSUM_ERROR;
    else return 0;
}

//===========================================================================//
int SHT2x::i2cWrite(int data)
//===========================================================================//
{
    if (i2c.write(data) < 1) return ACK_ERROR;
    else return 0;
}

//===========================================================================//
int SHT2x::SHT2x_ReadUserRegister(int *pRegisterValue)
//===========================================================================//
{
    int checksum;   //variable for checksum byte
    int error=0;    //variable for error code

    i2c.start();
    error |= i2cWrite(I2C_ADR_W);
    error |= i2cWrite(USER_REG_R);

    i2c.start();
    error |= i2cWrite(I2C_ADR_R);


    *pRegisterValue = i2c.read(ACK);

    checksum=i2c.read(NoACK);

    error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);

    i2c.stop();
    return error;
}
//===========================================================================//
int SHT2x::SHT2x_WriteUserRegister(int *pRegisterValue)
//===========================================================================//
{
    int error=0;   //variable for error code
    i2c.start();

    error |= i2cWrite(I2C_ADR_W);
    error |= i2cWrite(USER_REG_W);
    error |= i2cWrite(*pRegisterValue);
    i2c.stop();

    return error;
}
//===========================================================================//
int SHT2x::SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand)
//===========================================================================//
{
    int  checksum;   //checksum
    int  data[2];    //data array for checksum verification
    int  error=0;    //error variable
    int i;

    //-- write I2C sensor address and command --
    i2c.start();

    error |= i2cWrite(I2C_ADR_W); // I2C Adr
    switch (eSHT2xMeasureType) {
        case HUMIDITY:
            error |= i2cWrite(TRIG_RH_MEASUREMENT_HM);
            break;
        case TEMP:
            error |= i2cWrite(TRIG_T_MEASUREMENT_HM);
            break;
        default:
            break;
    }

    //-- wait until hold master is released --
    i2c.start();

    __disable_irq();     // Disable Interrupts

    error |= i2cWrite(I2C_ADR_R);

    //SCL=HIGH;                     // set SCL I/O port as input
    i2c.sclAsInput();

    for (i=0; i<1000; i++) {      // wait until master hold is released or
        wait_ms(1);    // a timeout (~1s) is reached
        if (i2c.sclRead() == 1) break;
    }

    //-- check for timeout --
    if (i2c.sclRead() == 0) error |= TIME_OUT_ERROR;
    //-- read two data bytes and one checksum byte --

    i2c.sclNormal();

    data[0] = i2c.read(ACK);
    data[1] = i2c.read(ACK);

    *pMeasurand = data[0] << 8;
    *pMeasurand |= data[1];

    checksum = i2c.read(NoACK);

    __enable_irq();     // Enable Interrupts

    //-- verify checksum --
    error |= SHT2x_CheckCrc (data, 2, checksum);

    i2c.stop();
    return error;
}
//===========================================================================//
int SHT2x::SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand)
//===========================================================================//
{
    int  checksum;   //checksum
    int  data[2];    //data array for checksum verification
    int  error = 0;    //error variable
    int i = 0;        //counting variableSample Code SHT21

    //-- write I2C sensor address and command --
    i2c.start();

    error |= i2cWrite(I2C_ADR_W);
    switch (eSHT2xMeasureType) {
        case HUMIDITY:
            error |= i2cWrite(TRIG_RH_MEASUREMENT_POLL);
            break;
        case TEMP:
            error |= i2cWrite(TRIG_T_MEASUREMENT_POLL);
            break;
        default:
            break;

    }
    //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
    do {
        i2c.start();
        wait_ms(10);
        if (i++ >= 20) break;
    } while (i2c.write(I2C_ADR_R) == 0);

    if (i >= 20) error |= TIME_OUT_ERROR;

    //-- read two data bytes and one checksum byte --
    data[0] = i2c.read(ACK);
    data[1] = i2c.read(ACK);

    *pMeasurand = data[0] << 8;
    *pMeasurand |= data[1];

    checksum = i2c.read(NoACK);
    //-- verify checksum --
    error |= SHT2x_CheckCrc (data,2,checksum);
    i2c.stop();
    return error;
}
//===========================================================================//
int SHT2x::SHT2x_SoftReset()
//===========================================================================//
{
    int  error=0;           //error variable
    i2c.start();
    error |= i2cWrite(I2C_ADR_W); // I2C Adr
    error |= i2cWrite(SOFT_RESET);                            // Command
    i2c.stop();
    wait_ms(15);
    return error;
}
//==============================================================================//
float SHT2x::SHT2x_CalcRH(int u16sRH)
//==============================================================================//
{
    float humidityRH;              // variable for result
    u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --
    humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    return humidityRH;
}
//==============================================================================//
float SHT2x::SHT2x_CalcTemperatureC(int u16sT)
//==============================================================================//
{
    float temperatureC;            // variable for result
    u16sT &= ~0x0003;           // clear bits [1..0] (status bits)
    //-- calculate temperature [in degrees C] --
    temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
    return temperatureC;
}

//==============================================================================//
float SHT2x::SHT2x_GetDewpoint(float h, float t)
//==============================================================================//
{
    float logEx, dew_point;
    logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
    dew_point = (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);
    return dew_point;
}

//==============================================================================//
int SHT2x::SHT2x_GetSerialNumber(int u8SerialNumber[])
//==============================================================================//
{
    int  error=0;                          //error variable
    //Read from memory location 1
    i2c.start();
    error |= i2cWrite(I2C_ADR_W);
    error |= i2cWrite(0xFA);         //Command for readout on-chip memory
    error |= i2cWrite(0x0F);         //on-chip memory address
    i2c.start();
    error |= i2cWrite(I2C_ADR_R);    //I2C address
    u8SerialNumber[5] = i2c.read(ACK); //Read SNB_3
    i2c.read(ACK);                     //Read CRC SNB_3 (CRC is not analyzed)
    u8SerialNumber[4] = i2c.read(ACK); //Read SNB_2
    i2c.read(ACK);                     //Read CRC SNB_2 (CRC is not analyzed)
    u8SerialNumber[3] = i2c.read(ACK); //Read SNB_1Sample Code SHT21
    i2c.read(ACK);                     //Read CRC SNB_1 (CRC is not analyzed)
    u8SerialNumber[2] = i2c.read(ACK); //Read SNB_0
    i2c.read(NoACK);                  //Read CRC SNB_0 (CRC is not analyzed)
    i2c.stop();
    //Read from memory location 2
    i2c.start();
    error |= i2cWrite(I2C_ADR_W);    //I2C address
    error |= i2cWrite(0xFC);         //Command for readout on-chip memory
    error |= i2cWrite(0xC9);         //on-chip memory address
    i2c.start();
    error |= i2cWrite(I2C_ADR_R);    //I2C address
    u8SerialNumber[1] = i2c.read(ACK); //Read SNC_1
    u8SerialNumber[0] = i2c.read(ACK); //Read SNC_0
    i2c.read(ACK);
    u8SerialNumber[7] = i2c.read(ACK); //Read SNA_1
    u8SerialNumber[6] = i2c.read(ACK); //Read SNA_0
    i2c.read(NoACK);                  //Read CRC SNA0/1 (CRC is not analyzed)
    i2c.stop();
    return error;
}