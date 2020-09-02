#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "em_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

/*!
 * Board MCU pins definitions
 */

// make a PIN_NAME int with high nibble = port and low nibble = pin
#define GPIO_FROM_PORT_PIN(port, pin) ((((port)&0xFu) << 4) + ((pin)&0xFu))
#define PIN_FROM_GPIO(n) (((n)&0xFu) << 0)
#define PORT_FROM_GPIO(n) ((n) >> 4)

#define RADIO_RESET                                 GPIO_FROM_PORT_PIN( GPIO_PORTD, 1 )
#define RADIO_MOSI                                  GPIO_FROM_PORT_PIN( GPIO_PORTC, 2 )
#define RADIO_MISO                                  GPIO_FROM_PORT_PIN( GPIO_PORTC, 3 )
#define RADIO_SCLK                                  GPIO_FROM_PORT_PIN( GPIO_PORTC, 1 )
#define RADIO_NSS                                   GPIO_FROM_PORT_PIN( GPIO_PORTC, 0 )
#define RADIO_BUSY                                  GPIO_FROM_PORT_PIN( GPIO_PORTC, 4 )
#define RADIO_DIO_1                                 GPIO_FROM_PORT_PIN( GPIO_PORTB, 0 )

#define LED_1                                       GPIO_FROM_PORT_PIN( GPIO_PORTD, 0 )

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
