/*
    Copyright (c) 2010 Andy Kirkham
 
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

#include "mbed.h"

#ifndef SHT21_I2C_H
#define SHT21_I2C_H

class SHT_I2C : public I2C {

protected:
    PinName _scl;
    PinName _sda;
    
public:

    SHT_I2C(PinName sda, PinName scl, const char *name = NULL) : I2C(sda, scl, name) {
        _sda = sda;
        _scl = scl;
    }
    
    void sclAsInput(void) {
        switch(_scl) {
            case p10:
                LPC_PINCON->PINSEL0 &= ~(3UL << 2); // p10, P0.1 as GPIO
                LPC_GPIO0->FIODIR &= ~(1UL << 1);   // p10, P0.1 as Input
                break;
            case p27:
                LPC_PINCON->PINSEL0&=~(3UL << 22);  // p27, P0.11 as GPIO
                LPC_GPIO0->FIODIR &= ~(1UL << 11);  // p27, P0.11 as input
                break;                
        }
    }
    
    int sclRead(void) {
        switch(_scl) {
            case p10: return (LPC_GPIO0->FIOPIN & (1UL <<  1)) ? 1 : 0;
            case p27: return (LPC_GPIO0->FIOPIN & (1UL << 11)) ? 1 : 0;                
            default:
                return 0;
        }
    }
    
    void sclNormal(void) {
        switch(_scl) {
            case p10:
                LPC_PINCON->PINSEL0 &= ~(3UL << 2); // p10, P0.1 as I2C SCL1
                LPC_PINCON->PINSEL0 |=  (3UL << 2);
                break;
            case p27:
                LPC_PINCON->PINSEL0 &= ~(3UL << 22); // p27, P0.11 as I2C SCL2
                LPC_PINCON->PINSEL0 |=  (2UL << 22);            
                break;                
        }
    }

};

#endif