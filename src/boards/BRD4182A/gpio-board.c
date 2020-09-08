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
#include "gpiointerrupt.h"
#include <stdint.h>

//static GpioIrqHandler *ioHandler[16];
//
//static void gpioIntHandler(uint8_t pin)
//{
//	if(ioHandler[pin] != NULL)
//		ioHandler[pin]();
//}

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
    else if( mode == PIN_OUTPUT)
    {
        GPIO_PinModeSet(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin), gpioModePushPull, value);
    }
    else if (mode == PIN_ANALOGIC)
    {
        GPIO_PinModeSet(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin), gpioModeDisabled, value);
    }
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
	printf("CFG IRQ pin %04X, mode %02X\n", obj->pin, irqMode);
	GPIO_PinModeSet(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin), gpioModeInputPull, 0);
	GPIO_IntConfig(PORT_FROM_GPIO(obj->pin), PIN_FROM_GPIO(obj->pin),
			(irqMode == IRQ_RISING_EDGE) || (irqMode == IRQ_RISING_FALLING_EDGE),
			(irqMode == IRQ_FALLING_EDGE) || (irqMode == IRQ_RISING_FALLING_EDGE), true);
	GPIOINT_CallbackRegister(PIN_FROM_GPIO(obj->pin), (GPIOINT_IrqCallbackPtr_t)irqHandler);
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{

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
