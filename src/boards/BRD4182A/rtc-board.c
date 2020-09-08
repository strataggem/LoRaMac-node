

#include "board-config.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"

#include "rtc-board.h"

#include "em_rtcc.h"
#include "em_cmu.h"
#include "em_gpio.h"

#define MIN_ALARM_DELAY     3 // in ticks

enum
{
    ALARM_STOPPED = 0,
    ALARM_RUNNING = 1
};

/*!
 * RTC timer context 
 */
typedef struct
{
    uint32_t Time;  // Reference time
    uint32_t Delay; // Reference Timeout duration
    uint32_t AlarmState;
}RtcTimerContext_t;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

static volatile bool RtcTimeoutPendingInterrupt = false;
static volatile bool RtcTimeoutPendingPolling = false;

/*!
 * Used to store the Seconds and SubSeconds.
 * 
 * WARNING: Temporary fix fix. Should use MCU NVM internal
 *          registers
 */
uint32_t RtcBkupRegisters[] = { 0, 0 };

/*!
 * \brief Callback for the hw_timer when alarm expired
 */
static void RtcAlarmIrq( void );

/*!
 * \brief Callback for the hw_timer when counter overflows
 */
static void RtcOverflowIrq( void );

void RtcInit( void )
{
    //ensure the LFRCO is stopped before setting high precision mode? (to avoid bus fault as stated in datasheet)
    //CMU_ClockEnable(cmuClock_LFRCO, false);
    CMU_LFRCOSetPrecision(cmuPrecisionHigh);
    CMU_ClockEnable(cmuClock_LFRCO, true);
    //TODO: add LFXO? LFRCO has only 500ppm max precision in precision mode
    CMU_ClockSelectSet(cmuClock_RTCCCLK, cmuSelect_LFRCO);
    CMU_ClockEnable(cmuClock_RTCC, true);

    // Initialize alarm (RTC Compare Channel 1)
    RTCC_CCChConf_TypeDef alarm_init = RTCC_CH_INIT_COMPARE_DEFAULT;
    RTCC_ChannelInit(1, &alarm_init);
    // initially disable alarm IRQ
    RTCC_IntDisable(RTCC_IEN_CC1);

    // Initialize RTCC counter with 1ms ticks
    RTCC_Init_TypeDef rtc_init = RTCC_INIT_DEFAULT;
    RTCC_Init(&rtc_init);


    NVIC_ClearPendingIRQ(RTCC_IRQn);
    NVIC_EnableIRQ(RTCC_IRQn);
}

uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = ( uint32_t )RTCC_CounterGet();
    return ( uint32_t )RtcTimerContext.Time;
}

uint32_t RtcGetTimerContext( void )
{
    return RtcTimerContext.Time;
}

uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
    //1.024kHz RTC clock
    uint32_t seconds = milliseconds / 1000;
    uint32_t ms = milliseconds % 1000;
    return ( uint32_t )( (seconds << 10) + ( ms << 10) / 1000);
}

TimerTime_t RtcTick2Ms( uint32_t tick )
{
    //1.024kHz RTC clock
    uint32_t seconds = tick >> 10;
    tick = tick & 0x3FF;
    return ( ( seconds * 1000 ) + ( ( tick * 1000 ) >> 10 ) );
}

void RtcDelayMs( TimerTime_t milliseconds )
{
    uint32_t delayTicks = 0;
    uint32_t refTicks = RtcGetTimerValue( );

    delayTicks = RtcMs2Tick( milliseconds );

    // Wait delay ms
    while( ( ( RtcGetTimerValue( ) - refTicks ) ) < delayTicks )
    {
        __asm volatile ("nop");
    }
}

void RtcSetAlarm( uint32_t timeout )
{
    RtcStartAlarm( timeout );
}

void RtcStopAlarm( void )
{
    RtcTimerContext.AlarmState = ALARM_STOPPED;
}

void RtcStartAlarm( uint32_t timeout )
{
    CRITICAL_SECTION_BEGIN( );

    RtcStopAlarm( );

    RtcTimerContext.Delay = timeout;

#if( RTC_DEBUG_PRINTF_STATE == RTC_DEBUG_ENABLE )
    printf( "TIMEOUT \t%010ld\t%010ld\n", RtcTimerContext.Time, RtcTimerContext.Delay );
#endif


    //initially alarm should be handled in IRQ
    RtcTimeoutPendingInterrupt = true;
    //initially alarm should NOT be handled manually (by polling)
    RtcTimeoutPendingPolling = false;

    RtcTimerContext.AlarmState = ALARM_RUNNING;

    uint32_t ticks = RtcTimerContext.Time + RtcTimerContext.Delay;
    uint32_t current = RTCC_CounterGet();
    RTCC_ChannelCompareValueSet(1, ticks);
    RTCC_IntEnable(RTCC_IEN_CC1);


    if((ticks - current - 1) >= (UINT32_MAX >> 1)) {
        // if difference is more than half of max assume timer has passed
    }
    if((ticks - current) < 10) {
        // if too close the matching interrupt does not trigger, so handle same as passed
    }

    if((ticks - current) < 10)
    {
        // If timer already passed
        if( RtcTimeoutPendingInterrupt == true )
        {
            // And interrupt not handled, mark as polling
            RTCC_IntDisable(RTCC_IEN_CC1);

            RtcTimeoutPendingPolling = true;
            RtcTimeoutPendingInterrupt = false;
        }
    }
    CRITICAL_SECTION_END( );
}

uint32_t RtcGetTimerValue( void )
{
    return ( uint32_t )RTCC_CounterGet();
}

uint32_t RtcGetTimerElapsedTime( void )
{
    return ( uint32_t)( RTCC_CounterGet() - RtcTimerContext.Time );
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    uint32_t counterValue = RTCC_CounterGet();

    uint32_t seconds = ( uint32_t )counterValue >> 10;

    uint32_t ticks =  ( uint32_t )counterValue & 0x3FF;

    *milliseconds = RtcTick2Ms( ticks );

    return seconds;
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
    CRITICAL_SECTION_BEGIN( );
    RtcBkupRegisters[0] = data0;
    RtcBkupRegisters[1] = data1;
    CRITICAL_SECTION_END( );
}

void RtcBkupRead( uint32_t* data0, uint32_t* data1 )
{
    CRITICAL_SECTION_BEGIN( );
    *data0 = RtcBkupRegisters[0];
    *data1 = RtcBkupRegisters[1];
    CRITICAL_SECTION_END( );
}

void RtcProcess( void )
{
    CRITICAL_SECTION_BEGIN( );

    if( (  RtcTimerContext.AlarmState == ALARM_RUNNING ) && ( RtcTimeoutPendingPolling == true ) )
    {
        if( RtcGetTimerElapsedTime( ) >= RtcTimerContext.Delay )
        {
            RtcTimerContext.AlarmState = ALARM_STOPPED;

            // Because of one shot the task will be removed after the callback
            RtcTimeoutPendingPolling = false;

            // NOTE: The handler should take less then 1 ms otherwise the clock shifts
            TimerIrqHandler( );
        }
    }
    CRITICAL_SECTION_END( );
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    return period;
}


void RTCC_IRQHandler()
{
    uint32_t flags = RTCC_IntGetEnabled();

    if(flags & RTCC_IF_CC1)
    {
        RtcAlarmIrq();
    }

    if(flags & RTCC_IF_OF)
    {
        RtcOverflowIrq();
    }

    RTCC_IntClear(flags);
}

static void RtcAlarmIrq( void )
{
    RtcTimerContext.AlarmState = ALARM_STOPPED;
    // Because of one shot the task will be removed after the callback
    RtcTimeoutPendingInterrupt = false;

    // NOTE: The handler should take less then 1 ms otherwise the clock shifts
    TimerIrqHandler( );
}

static void RtcOverflowIrq( void )
{
    //RtcTimerContext.Time += ( uint64_t )( 1 << 32 );
}
