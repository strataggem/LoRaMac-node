/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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

#include "board-config.h"
#include "utilities.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "gps.h"
#include "rtc-board.h"
#include "sx126x-board.h"
#include "board.h"
#include "gpiointerrupt.h"

/*
 * MCU objects
 */
Gpio_t Led1;
Uart_t Uart1;
I2c_t I2c;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
}

void BoardInitPeriph( void )
{
    GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void BoardInitMcu( void )
{
	GPIOINT_Init();

    RtcInit();
    SpiInit(&SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX126xIoInit();
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

    //Restart system
    NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
    SpiDeInit( &SX126x.Spi );
}

uint32_t BoardGetRandomSeed( void )
{
    return 0;
}

void BoardGetUniqueId( uint8_t *id )
{

}

uint8_t BoardGetBatteryLevel( void )
{
    return 0; //  Battery level [0: node is connected to an external power source ...
}

uint8_t GetBoardPowerSource( void )
{
    return BATTERY_POWER;
}

void BoardLowPowerHandler( void )
{
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending 
     * and cortex will not enter low power anyway
     */

    // Call low power handling function.

    __enable_irq( );
}

#if !defined ( __CC_ARM )

/*
 * Function to be used by stdout for printf etc
 */
//int _write( int fd, const void *buf, size_t count )
//{
//    while( UartPutBuffer( &Uart1, ( uint8_t* )buf, ( uint16_t )count ) != 0 ){ };
//    return count;
//}

/*
 * Function to be used by stdin for scanf etc
 */
//int _read( int fd, const void *buf, size_t count )
//{
//    size_t bytesRead = 0;
//    while( UartGetBuffer( &Uart1, ( uint8_t* )buf, count, ( uint16_t* )&bytesRead ) != 0 ){ };
//    // Echo back the character
//    while( UartPutBuffer( &Uart1, ( uint8_t* )buf, ( uint16_t )bytesRead ) != 0 ){ };
//    return bytesRead;
//}

#else

#include <stdio.h>

// Keil compiler
int fputc( int c, FILE *stream )
{
    while( UartPutChar( &Uart1, ( uint8_t )c ) != 0 );
    return c;
}

int fgetc( FILE *stream )
{
    uint8_t c = 0;
    while( UartGetChar( &Uart1, &c ) != 0 );
    // Echo back the character
    while( UartPutChar( &Uart1, c ) != 0 );
    return ( int )c;
}

#endif

#ifdef USE_FULL_ASSERT

#include <stdio.h>

/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %u\n", file, line) */

    printf( "Wrong parameters value: file %s on line %u\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
