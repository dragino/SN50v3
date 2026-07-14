#ifndef __FLASH_ERASEPROGRAM_H__
#define __FLASH_ERASEPROGRAM_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  initialises the 
 *
 * @note
 * @retval None
 */
#define FLASH_USER_START_ADDR_CONFIG   (0x0803F000)        /* Start @ of user Flash area store config */
#define FLASH_USER_END_ADDR            (0x08040000)        /* End @ of user Flash area store key*/

#define FLASH_USER_START_ADDR_KEY      (0x0803E000)

#define EEPROM_USER_START_ADDR_KEY     (0x0803E000)
#define EEPROM_USER_START_ADDR_CONFIG  (0x0803F000)
//#define EEPROM_USER_END_ADDR_CONFIG    (0x08040000)

/*
 *3328
 */
#define SRAM_SENSOR_DATA_STORE_ACK_START_ADDR   (0x2000F100)          
#define SRAM_SENSOR_DATA_STORE_ACK_END_ADDR     (0x2000FE00)

/**
 * 52KB
 * 
 */	 
#define FLASH_SENSOR_DATA_START_ADDR   (0x08031000)          
#define FLASH_SENSOR_DATA_END_ADDR     (0x0803E000)

#define EEPROM_USER_END_ADDR_CONFIG    (0x08031000)
#define EEPROM_USER_CUT_ADDR_CONFIG    (EEPROM_USER_END_ADDR_CONFIG+0x04*60)
#define EEPROM_USER_DEL_ADDR_CONFIG    (EEPROM_USER_CUT_ADDR_CONFIG+0x04*30)

void  FLASH_program(uint32_t add, uint8_t *data, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_ERASEPROGRAM_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
