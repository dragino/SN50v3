#ifndef __I2C_A_H__
#define __I2C_A_H__

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
// I2C acknowledge
typedef enum{
  ACK  = 0,
  NACK = 1,
}etI2cAck;

//-- Enumerations -------------------------------------------------------------
// Error codes
typedef enum{
  NO_ERROR       = 0x00, // no error
  ACK_ERROR      = 0x01, // no acknowledgment error
  CHECKSUM_ERROR = 0x02, // checksum mismatch error
  TIMEOUT_ERROR  = 0x04, // timeout error
  PARM_ERROR     = 0x80, // parameter out of range error
}etError;

//=============================================================================
void I2c_hal_Init(void);
//=============================================================================
// Initializes the ports for I2C interface.
//-----------------------------------------------------------------------------

//=============================================================================
void I2c_hal_StartCondition(void);
//=============================================================================
// Writes a start condition on I2C-Bus.
//-----------------------------------------------------------------------------
// remark: Timing (delay) may have to be changed for different microcontroller.
//       _____
// SDA:       |_____
//       _______
// SCL:         |___

//=============================================================================
void I2c_hal_StopCondition(void);
//=============================================================================
// Writes a stop condition on I2C-Bus.
//-----------------------------------------------------------------------------
// remark: Timing (delay) may have to be changed for different microcontroller.
//              _____
// SDA:   _____|
//            _______
// SCL:   ___|

//=============================================================================
etError I2c_hal_WriteByte(uint8_t txByte);
//=============================================================================
// Writes a byte to I2C-Bus and checks acknowledge.
//-----------------------------------------------------------------------------
// input:  txByte       transmit byte
//
// return: error:       ACK_ERROR = no acknowledgment from sensor
//                      NO_ERROR  = no error
//
// remark: Timing (delay) may have to be changed for different microcontroller.

//=============================================================================
etError I2c_hal_ReadByte(uint8_t *rxByte, etI2cAck ack, uint8_t timeout);

etError I2c_hal_GeneralCallReset(void);

void I2C_GPIO_MODE_Config(void);
void I2C_GPIO_MODE_ANALOG(void);
void I2C_delay(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WaitAck(void);
void I2C_NAck(void);
void I2C_Ack(void);
void I2C_SendByte(uint8_t _ucByte);
uint8_t I2C_ReadByte(unsigned char ack);
uint8_t I2C_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t I2C_Read_reg_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
void SDA_IN(void);
void SDA_OUT(void);
uint8_t I2C_Write_Byte(uint8_t addr,uint8_t data);
uint8_t I2C_Read_Byte(uint8_t addr);
uint8_t I2C_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t I2C_Write_reg_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
#ifdef __cplusplus
}
#endif

#endif /* __I2C_A_H__ */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
