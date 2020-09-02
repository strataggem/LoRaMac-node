/**
 * @file      i2c-board.c
 *
 * @brief     Target board I2C driver implementation
 *
 * @copyright Copyright (c) 2020 The Things Industries B.V.
 *
 * Revised BSD License
 * Copyright The Things Industries B.V 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Things Industries B.V nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE THINGS INDUSTRIES B.V BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "board.h"
#include "i2c-board.h"

#include "em_i2c.h"
#include "i2cspm.h"

void I2cMcuInit( I2c_t* obj, I2cId_t i2cId, PinNames scl, PinNames sda )
{
    obj->I2cId = i2cId;
    obj->Scl.pin = scl;
    obj->Sda.pin = sda;

    I2CSPM_Init_TypeDef i2c_init = I2CSPM_INIT_DEFAULT;
    i2c_init.port = I2C(i2cId);
    i2c_init.sclPort = PORT_FROM_GPIO(scl);
    i2c_init.sclPin  = PIN_FROM_GPIO(scl);
    i2c_init.sdaPort = PORT_FROM_GPIO(sda);
    i2c_init.sdaPin  = PIN_FROM_GPIO(sda);
    I2CSPM_Init(&i2c_init);
}

void I2cMcuDeInit( I2c_t* obj )
{


}

void I2cMcuFormat( I2c_t* obj, I2cMode mode, I2cDutyCycle dutyCycle, bool I2cAckEnable, I2cAckAddrMode AckAddrMode,
                   uint32_t I2cFrequency )
{
    while(1);
    return 0;
}

uint8_t I2cMcuWriteBuffer( I2c_t* obj, uint8_t deviceAddr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    while(1);
    return 0;
}

uint8_t I2cMcuReadBuffer( I2c_t* obj, uint8_t deviceAddr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    while(1);
    return 0;
}
