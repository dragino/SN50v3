#include "tremo_regs.h"
#include "tremo_system.h"

extern uint32_t count1,count2;

/**
 * @brief Reset the chip
 * @param None
 * @retval None
 */
void system_reset(void)
{
		*((uint8_t *)(0x2000FE0B)) =0xAA;
		*((uint8_t *)(0x2000FE0C)) =count1>>24;
		*((uint8_t *)(0x2000FE0D)) =count1>>16;
		*((uint8_t *)(0x2000FE0E)) =count1>>8;
		*((uint8_t *)(0x2000FE0F)) =count1;
		*((uint8_t *)(0x2000FE10)) =count2>>24;
		*((uint8_t *)(0x2000FE11)) =count2>>16;
		*((uint8_t *)(0x2000FE12)) =count2>>8;
		*((uint8_t *)(0x2000FE13)) =count2;
    NVIC_SystemReset();
}

/**
 * @brief Read the chip id of the chip  
 * @param id The chip id read from the chip
 * @retval ERRNO_OK Read chip id successfully 
 * @retval ERRNO_ERROR The input parameter is NULL
 */
int32_t system_get_chip_id(uint32_t* id)
{
    if (!id)
        return ERRNO_ERROR;

    id[0] = EFC->SN_L;
    id[1] = EFC->SN_H;

    return ERRNO_OK;
}

