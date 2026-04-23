#include "stdio.h"	
#include "flash_eraseprogram.h"
#include "timer.h"
#include "log.h"
#include "tremo_flash.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

void  FLASH_program(uint32_t add, uint8_t *data, uint8_t count)
{
	__disable_irq();	
	if(flash_program_bytes(add,data,count)==ERRNO_FLASH_SEC_ERROR)
	{
		LOG_PRINTF(LL_DEBUG,"write config error\r\n");
	}
	__enable_irq();
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
