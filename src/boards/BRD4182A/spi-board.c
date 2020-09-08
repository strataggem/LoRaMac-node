
#include "spi-board.h"
#include "board-config.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    obj->Mosi.pin=mosi;
    obj->Miso.pin=miso;
    obj->Sclk.pin=sclk;
    obj->Nss.pin=nss;

    // Enable GPIO clock
    CMU_ClockEnable(cmuClock_GPIO, true);
    // Configure RX pin as an input
    GPIO_PinModeSet(PORT_FROM_GPIO(miso), PIN_FROM_GPIO(miso), gpioModeInput, 0);
    // Configure TX pin as an output
    GPIO_PinModeSet(PORT_FROM_GPIO(mosi), PIN_FROM_GPIO(mosi), gpioModePushPull, 0);
    // Configure CLK pin as an output low (CPOL = 0)
    GPIO_PinModeSet(PORT_FROM_GPIO(sclk), PIN_FROM_GPIO(sclk), gpioModePushPull, 0);

    if(nss != NC)
    {
        // Configure CS pin as an output and drive inactive high
        GPIO_PinModeSet(PORT_FROM_GPIO(nss), PIN_FROM_GPIO(nss), gpioModePushPull, 1);
    }

    // Enable peripheral clock
    CMU_ClockEnable(cmuClock_USART0, true);

    // Default asynchronous initializer (master mode, 1 Mbps, 8-bit data)
    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
    init.msbf = true; // MSB first transmission for SPI compatibility
    init.baudrate = 10000000; //10MHz clock
    /*
     * Route USART0 RX, TX, and CLK to the specified pins.  Note that CS is
     * not controlled by USART0 so there is no write to the corresponding
     * USARTROUTE register to do this.
     */
    GPIO->USARTROUTE[0].RXROUTE = (PORT_FROM_GPIO(miso) << _GPIO_USART_RXROUTE_PORT_SHIFT)
                          | (PIN_FROM_GPIO(miso) << _GPIO_USART_RXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].TXROUTE = (PORT_FROM_GPIO(mosi) << _GPIO_USART_TXROUTE_PORT_SHIFT)
                          | (PIN_FROM_GPIO(mosi) << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].CLKROUTE = (PORT_FROM_GPIO(sclk) << _GPIO_USART_CLKROUTE_PORT_SHIFT)
                          | (PIN_FROM_GPIO(sclk) << _GPIO_USART_CLKROUTE_PIN_SHIFT);

    // Enable USART interface pins
    GPIO->USARTROUTE[0].ROUTEEN =   GPIO_USART_ROUTEEN_RXPEN |  // MISO
            GPIO_USART_ROUTEEN_TXPEN |  // MOSI
            GPIO_USART_ROUTEEN_CLKPEN;  // SCLK

    // Configure and enable USART0
    USART_InitSync(USART0, &init);
}

void SpiDeInit( Spi_t *obj )
{
    USART_Enable(USART(obj->SpiId), usartDisable);

    // Configure RX pin as an input
    GPIO_PinModeSet(PORT_FROM_GPIO(obj->Miso.pin), PIN_FROM_GPIO(obj->Miso.pin), gpioModeDisabled, 0);
    // Configure TX pin as an output
    GPIO_PinModeSet(PORT_FROM_GPIO(obj->Mosi.pin), PIN_FROM_GPIO(obj->Mosi.pin), gpioModeDisabled, 0);
    // Configure CLK pin as an output low (CPOL = 0)
    GPIO_PinModeSet(PORT_FROM_GPIO(obj->Sclk.pin), PIN_FROM_GPIO(obj->Sclk.pin), gpioModeDisabled, 0);
    // Configure CS pin as an output and drive inactive high
    GPIO_PinModeSet(PORT_FROM_GPIO(obj->Nss.pin), PIN_FROM_GPIO(obj->Nss.pin), gpioModeDisabled, 0);
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    //This is the HAL version but only support 8bit transfer
    //return USART_SpiTransfer(USART(obj->SpiId), outData);

    //this is the version not limited in bit width
    while (!(USART(obj->SpiId)->STATUS & USART_STATUS_TXBL));
    USART(obj->SpiId)->TXDATA = (uint32_t)outData;
    while (!(USART(obj->SpiId)->STATUS & USART_STATUS_TXC));
    return (uint16_t)USART(obj->SpiId)->RXDATA;
}
