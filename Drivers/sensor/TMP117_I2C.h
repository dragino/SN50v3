#ifndef __TMP117_I2C_H__
#define __TMP117_I2C_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h> 
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  
 *
 * @note
 * @retval None
 */
void TMP117_I2C_GPIO_MODE_Config(void);
void TMP117_I2C_delay(void);
void TMP117_I2C_Start(void);
void TMP117_I2C_Stop(void);
uint8_t TMP117_I2C_WaitAck(void);
void TMP117_I2C_NAck(void);
void TMP117_I2C_Ack(void);
void TMP117_I2C_SendByte(uint8_t _ucByte);
uint8_t TMP117_I2C_ReadByte(unsigned char ack);
uint8_t TMP117_I2C_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
void TMP117_I2C_SDA_IN(void);
void TMP117_I2C_SDA_OUT(void);
uint8_t TMP117_I2C_Write_Byte(uint8_t addr,uint8_t data);
uint8_t TMP117_I2C_Read_Byte(uint8_t addr);
uint8_t TMP117_I2C_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

uint16_t TMP117_Set_Config(void);
float get_tmp117_temp(void);
void TMP117_soft_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __TMP117_I2C_H__ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
