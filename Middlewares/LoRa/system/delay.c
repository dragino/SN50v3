#include "tremo_delay.h"
#include "delay.h"

void Delay( float s )
{
    DelayMs( s * 1000.0f );
}

void DelayMs( uint32_t ms )
{
    delay_ms(ms);                    
}
