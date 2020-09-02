/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */

#include "gpio-board.h"
#include "board-config.h"
#include "em_gpio.h"

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
    obj->pin = pin;

    if( pin == NC )
    {
        return;
    }

    if( mode == PIN_INPUT )
    {
        GPIO_PinModeSet(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin), (type == 0) ? gpioModeInput : gpioModeInputPull, type == PIN_PULL_UP);
    }
    else // mode output
    {
        GPIO_PinModeSet(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin), gpioModePushPull, value);
    }
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    //ext_irq_register( obj->pin, irqHandler );
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    //ext_irq_register( obj->pin, NULL );
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    if(!obj)
    {
        //assert_param( FAIL );
        while( 1 );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return;
    }

    if(value)
        GPIO_PinOutSet(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin));
    else
        GPIO_PinOutClear(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin));

}

void GpioMcuToggle( Gpio_t *obj )
{
    if(!obj)
    {
        //assert_param( FAIL );
        while( 1 );
    }

    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return;
    }
    GPIO_PinOutToggle(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin));
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if(!obj)
    {
        //assert_param( FAIL );
        while( 1 );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return 0;
    }
    return ( uint32_t )GPIO_PinInGet(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin));
}
