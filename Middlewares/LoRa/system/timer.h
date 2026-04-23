/*
	MIT License

	Copyright (c) 2026 DRAGINO Team

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>     
#include "utilities.h"
#include "time.h"
	 
/* Exported types ------------------------------------------------------------*/

/*!
 * \brief Timer object description
 */
typedef struct TimerEvent_s
{
    uint32_t Timestamp;         //! Expiring timer value in ticks from TimerContext
    uint32_t ReloadValue;       //! Reload Value when Timer is restarted
    bool IsRunning;             //! Is the timer currently running
    void ( *Callback )( void ); //! Timer IRQ callback function
    struct TimerEvent_s *Next;  //! Pointer to the next Timer object.
} TimerEvent_t;

#ifndef TimerTime_t
typedef uint64_t TimerTime_t;
#endif

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/*!
 * \brief Number of seconds elapsed between Unix and GPS epoch
 */
#define UNIX_GPS_EPOCH_OFFSET                       315964800

/**
  * @brief Max timer mask
  */
#define TIMERTIME_T_MAX ( ( uint32_t )~0 )

/*!
 * \brief Structure holding the system time in seconds and miliseconds.
 */
typedef struct SysTime_s
{
    uint32_t Seconds;
    int16_t  SubSeconds;
}SysTime_t;

/*!
 * \brief Adds 2 SysTime_t values
 *
 * \param a Value
 * \param b Value to added
 *
 * \retval result Addition result (SysTime_t value)
 */
SysTime_t SysTimeAdd( SysTime_t a, SysTime_t b );

/*!
 * \brief Subtracts 2 SysTime_t values
 *
 * \param a Value
 * \param b Value to be subtracted
 *
 * \retval result Subtraction result (SysTime_t value)
 */
SysTime_t SysTimeSub( SysTime_t a, SysTime_t b );

/*!
 * \brief Sets the system time with the number of sconds elapsed since epoch
 *
 * \param [IN] sysTime Structure provideing the number of seconds and 
 *                     subseconds elapsed since epoch
  */
void SysTimeSet( SysTime_t sysTime );

/*!
 * \brief Gets the current system number of sconds elapsed since epoch
 *
 * \retval sysTime Structure provideing the number of seconds and 
 *                 subseconds elapsed since epoch
  */
SysTime_t SysTimeGet( void );

/*!
 * \brief Gets current MCU system time
 *
 * \retval sysTime    Current seconds/sub-seconds since Mcu started
 */
SysTime_t SysTimeGetMcuTime( void );

/*!
 * Converts the given SysTime to the equivalent RTC value in milliseconds
 *
 * \param [IN] sysTime System time to be converted
 * 
 * \retval timeMs The RTC converted time value in ms
 */
uint32_t SysTimeToMs( SysTime_t sysTime );

/*!
 * Converts the given RTC value in milliseconds to the equivalent SysTime
 *
 * \param [IN] timeMs The RTC time value in ms to be converted
 * 
 * \retval sysTime Converted system time
 */
SysTime_t SysTimeFromMs( uint32_t timeMs );

/*!
 * \brief Converts a given time in seconds since UNIX epoch into calendar time.
 *
 * \param [IN]  timestamp The time since UNIX epoch to convert into calendar time.
 * \param [OUT] localtime Pointer to the calendar time object which will contain
                          the result of the conversion.
 */
void SysTimeLocalTime( const uint32_t timestamp, struct tm *localtime );

/*!
 * \brief Initializes the timer object
 *
 * \remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * \param [IN] obj          Structure containing the timer object parameters
 * \param [IN] callback     Function callback called at the end of the timeout
 */
void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) );

/*!
 * \brief Timer IRQ event handler
 *
 * \note Head Timer Object is automaitcally removed from the List
 *
 * \note e.g. it is snot needded to stop it
 */
void TimerIrqHandler( void );

/*!
 * \brief Starts and adds the timer object to the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStart( TimerEvent_t *obj );

/*!
 * \brief Stops and removes the timer object from the list of timer events
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerStop( TimerEvent_t *obj );

/*!
 * \brief Resets the timer object
 *
 * \param [IN] obj Structure containing the timer object parameters
 */
void TimerReset( TimerEvent_t *obj );

/*!
 * \brief Set timer new timeout value
 *
 * \param [IN] obj   Structure containing the timer object parameters
 * \param [IN] value New timer timeout value
 */
void TimerSetValue( TimerEvent_t *obj, uint32_t value );


/*!
 * \brief Read the current time
 *
 * \retval returns current time in ms
 */
TimerTime_t TimerGetCurrentTime( void );

/*!
 * \brief Return the Time elapsed since a fix moment in Time
 *
 * \param [IN] savedTime    fix moment in Time
 * \retval time             returns elapsed time in ms
 */
TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime );

/*!
 * \brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * \param [IN] period Time period to compensate
 * \param [IN] temperature Current temperature
 *
 * \retval Compensated time period
 */
TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature );

/*!
 * \brief Manages the entry into ARM cortex deep-sleep mode
 */
void TimerLowPowerHandler( void );

#ifdef __cplusplus
}
#endif

/*! \} defgroup LORA_TIMER */
/*! \} addtogroup LORA */

#endif /* __TIMER_H__ */
