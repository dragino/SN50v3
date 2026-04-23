#ifndef __DELAY_H__
#define __DELAY_H__

#include <stdint.h>

/*! 
 * Blocking delay of "s" seconds
 */
void Delay( float s );

/*! 
 * Blocking delay of "ms" milliseconds
 */
void DelayMs( uint32_t ms );

/*! \} defgroup LORA_DELAY */
/*! \} addtogroup LORA */

#endif // __DELAY_H__

