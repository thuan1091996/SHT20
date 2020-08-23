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

#include "mbed.h"
#include "SHT21_I2C.h"

#ifndef SHT2x_H
#define SHT2x_H

const int POLYNOMIAL = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001
const int ACK = 1;
const int NoACK = 0;

// sensor command
enum etSHT2xCommand {
    TRIG_T_MEASUREMENT_HM    = 0xE3, // command trig. temp meas. hold master
    TRIG_RH_MEASUREMENT_HM   = 0xE5, // command trig. humidity meas. hold master
    TRIG_T_MEASUREMENT_POLL  = 0xF3, // command trig. temp meas. no hold master
    TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
    USER_REG_W               = 0xE6, // command writing user register
    USER_REG_R               = 0xE7, // command reading user register
    SOFT_RESET               = 0xFE  // command soft reset
};

enum etSHT2xResolution {
    SHT2x_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
    SHT2x_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
    SHT2x_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
    SHT2x_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
    SHT2x_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
};

enum etSHT2xEob {
    SHT2x_EOB_ON             = 0x40, // end of battery
    SHT2x_EOB_MASK           = 0x40, // Mask for EOB bit(6) in user reg.
};

enum etSHT2xHeater {
    SHT2x_HEATER_ON          = 0x04, // heater on
    SHT2x_HEATER_OFF         = 0x00, // heater off
    SHT2x_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
};
// measurement signal selection

enum etSHT2xMeasureType {
    HUMIDITY,
    TEMP
};

enum etI2cHeader {
    I2C_ADR_W                = 128,   // sensor I2C address + write bit
    I2C_ADR_R                = 129    // sensor I2C address + read bit
};

// Error codes
enum etError {
    ACK_ERROR                = 0x01,
    TIME_OUT_ERROR           = 0x02,
    CHECKSUM_ERROR           = 0x04,
    UNIT_ERROR               = 0x08
};

class SHT2x : public Base {
public:
    SHT2x (PinName p_sda, PinName p_scl);

    int SHT2x_CheckCrc(int data[], int nbrOfBytes, int checksum);
    int SHT2x_ReadUserRegister(int *pRegisterValue);
    int SHT2x_WriteUserRegister(int *pRegisterValue);
    int SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand);
    int SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, int *pMeasurand);
    int SHT2x_SoftReset();
    float SHT2x_CalcRH(int u16sRH);
    float SHT2x_CalcTemperatureC(int u16sT);
    int SHT2x_GetSerialNumber(int u8SerialNumber[]);
    float SHT2x_GetDewpoint(float h, float t);    

protected:
    SHT_I2C i2c;    
    int i2cWrite(int data);

private:

};
#endif