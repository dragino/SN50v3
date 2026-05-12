/**
  ******************************************************************************
  * @file    ZB25VQ32.h
  * @modify  MCD Application Team
  * @brief   This file contains all the description of the
  *          ZB25VQ32 memory.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ZB25VQ32_H
#define ZB25VQ32_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32wlxx_hal.h"

/** @defgroup ZB25VQ32_Exported_Constants ZB25VQ32 Exported Constants
  * @{
  */
/* Device ID & Secure OTP length *********************************************/
/* Table 7.4(1) Manufacturer and Device Identification(SPI and QPI Mode)
   * 5.1 Flash Memory Array
   * The memory is organized as:
   * 4194304 Bytes
   * 64 blocks of 64K-Byte
   * 1024 sectors of 4K-Byte
   * 16384 pages (256 Bytes each)
   */
 
#define ZB25VQ32_MANUFACTURER_ID             ((uint8_t)0x5E)
#define ZB25VQ32_DEVICE_MEM_TYPE             ((uint8_t)0x40)
#define ZB25VQ32_DEVICE_MEM_CAPACITY         ((uint8_t)0x16)

#define ZB25VQ32_BLOCK_64K                   (uint32_t) (64 * 1024)        
#define ZB25VQ32_BLOCK_32K                   (uint32_t) (32 * 1024)        
#define ZB25VQ32_SECTOR_4K                   (uint32_t) (4  * 1024)        

#define ZB25VQ32_FLASH_SIZE                  (uint32_t) (32*1024*1024/8)   /*!< 32 MBits => 4MBytes */
#define ZB25VQ32_SECTOR_SIZE                 ZB25VQ32_SECTOR_4K
#define ZB25VQ32_BLOCK_SIZE                  ZB25VQ32_BLOCK_64K
#define ZB25VQ32_PAGE_SIZE                   256U                          /*!< 16384 pages of 256 Bytes */

/**
  * @brief  ZB25VQ32 Timing configuration
	* Page program time: 500us typical
  * Sector erase time: 45ms typical
  * Block erase time: 250ms typical
  * Chip erase time: 12s typical
  */

#define ZB25VQ32_CHIP_ERASE_MAX_TIME                  14000U /* time in millisecond to erase the full flash */
#define ZB25VQ32_BLOCK_ERASE_MAX_TIME                 2000U /* time in millisecond to erase a block of 64K */
#define ZB25VQ32_SECTOR_ERASE_MAX_TIME                200U  /* time in millisecond to erase a sector of 4K */
#define ZB25VQ32_WRITE_REG_MAX_TIME                   40U   /* time in millisecond to write to Status Register */
#define ZB25VQ32_PAGE_PROGRAM_MAX_TIME                3U    /* time in millisecond to write a page of 256 bytes */

/* ZB25VQ32 Component Error codes *********************************************/
#define ZB25VQ32_OK                           0
#define ZB25VQ32_ERROR_INIT                  -1
#define ZB25VQ32_ERROR_COMMAND               -2
#define ZB25VQ32_ERROR_TRANSMIT              -3
#define ZB25VQ32_ERROR_RECEIVE               -4
#define ZB25VQ32_ERROR_AUTOPOLLING           -5
#define ZB25VQ32_ERROR_MEMORYMAPPED          -6
#define ZB25VQ32_ERROR_ADDRESS               -7

/******************************************************************************
  * @brief  ZB25VQ32 Commands
  ****************************************************************************/
/***** Read/Write Array Commands (3 Byte Address Command Set) **************/
/* Read Operations */
#define ZB25VQ32_READ_CMD                             0x03U   /*!< READ, Normal Read 3 Byte Address; SPI 1-1-1                    */
#define ZB25VQ32_FAST_READ_CMD                        0x0BU   /*!< FAST READ, Fast Read 3 Byte Address; SPI 1-1-1                 */

/* Program Operations */
#define ZB25VQ32_PAGE_PROG_CMD                        0x02U   /*!< PP, Page Program 3 Byte Address; SPI 1-1-1           */

/* Erase Operations */
#define ZB25VQ32_SECTOR_ERASE_4K_CMD                  0x20U   /*!< SE, Sector Erase 4KB 3 Byte Address; SPI 1-1-0       */
#define ZB25VQ32_BLOCK_ERASE_64K_CMD                  0xD8U   /*!< BE, Block Erase 64KB 3 Byte Address; SPI 1-1-0       */
#define ZB25VQ32_CHIP_ERASE_CMD                       0xC7U   /*!< CE, Chip Erase 0 Byte Address; SPI 1-0-0             */

/***** Register/Setting Commands *********************************************/
#define ZB25VQ32_WRITE_ENABLE_CMD                     0x06U   /*!< WREN, Write Enable; SPI                        */
#define ZB25VQ32_WRITE_DISABLE_CMD                    0x04U   /*!< WRDI, Write Disable; SPI                       */

#define ZB25VQ32_READ_STATUS_REG_CMD                  0x05U   /*!< RDSR, Read Status Register; SPI                */
#define ZB25VQ32_WRITE_STATUS_REG_CMD                 0x01U   /*!< WRSR, Write Status Register; SPI               */

#define ZB25VQ32_DEEP_POWER_DOWN_CMD                  0xB9U   /*!< DP, Deep power down;               SPI */
#define ZB25VQ32_RELEASE_FROM_DEEP_POWER_DOWN_CMD     0xABU   /*!< RDP, Release from Deep Power down; SPI */

/***** ID/Security Commands **************************************************/
/* Identification Operations */
#define ZB25VQ32_READ_ID_CMD                          0x9FU   /*!< RDID, Read IDentification; SPI                         */
#define ZB25VQ32_READ_ELECTRONIC_ID_CMD               0xABU   /*!< RES, Read Electronic ID; SPI                           */
#define ZB25VQ32_READ_ELECTRONIC_MANFACTURER_ID_CMD   0x90U   /*!< REMS, Read Electronic Manufacturer ID & Device ID; SPI */

#define ZB25VQ32_READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5AU   /*!< RDSFDP, Read Serial Flash Discoverable Parameter; SPI  */

#define ZB25VQ32_DOUBLE_OUTPUT_MODE_CMD               0x3BU   /*!< DREAD, Double Output Mode; SPI                         */

/******************************************************************************
  * @brief  ZB25VQ32 Registers
  ****************************************************************************/
/* Status Register */
#define ZB25VQ32_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
#define ZB25VQ32_SR_WREN                     ((uint8_t)0x02)    /*!< Write enable latch */

/**
  * @}
  */

/** @defgroup ZB25VQ32_Exported_Types ZB25VQ32 Exported Types
  * @{
  */

typedef struct
{
  uint32_t FlashSize;                        /*!< Size of the flash                             */
  uint32_t EraseSectorSize;                  /*!< Size of sectors for the erase operation       */
  uint32_t EraseSectorsNumber;               /*!< Number of sectors for the erase operation     */
  uint32_t EraseSubSectorSize;               /*!< Size of subsector for the erase operation     */
  uint32_t EraseSubSectorNumber;             /*!< Number of subsector for the erase operation   */
  uint32_t EraseSubSector1Size;              /*!< Size of subsector 1 for the erase operation   */
  uint32_t EraseSubSector1Number;            /*!< Number of subsector 1 for the erase operation */
  uint32_t ProgPageSize;                     /*!< Size of pages for the program operation       */
  uint32_t ProgPagesNumber;                  /*!< Number of pages for the program operation     */
} ZB25VQ32_Info_t;

typedef enum
{
  ZB25VQ32_ERASE_4K = 0,                 /*!< 4K size Sector erase */
  ZB25VQ32_ERASE_64K,                    /*!< 64K size Block erase */
  ZB25VQ32_ERASE_CHIP                    /*!< Whole chip erase     */
} ZB25VQ32_Erase_t;

/**
  * @}
  */

void MX_SPI1_Init(void);
	
/** @defgroup ZB25VQ32_Exported_Functions ZB25VQ32 Exported Functions
  * @{
  */
/* Function by commands combined */
int32_t ZB25VQ32_GetFlashInfo(ZB25VQ32_Info_t *pInfo);
int32_t ZB25VQ32_AutoPollingMemReady(void);
int32_t ZB25VQ32_AutoPollingMemReadyToWrite(void);

/* Read/Write Array Commands (3 Bytes Address Command Set) ********************/
int32_t ZB25VQ32_Read(uint8_t *pData, uint32_t ReadAddr, uint16_t Size);
int32_t ZB25VQ32_FastRead(uint8_t *pData, uint32_t ReadAddr, uint16_t Size);
int32_t ZB25VQ32_PageProgram(uint8_t *pData, uint32_t WriteAddr, uint16_t Size);
int32_t ZB25VQ32_SectorErase(uint32_t SectorAddress);
int32_t ZB25VQ32_BlockErase(uint32_t BlockAddress, ZB25VQ32_Erase_t BlockSize);
int32_t ZB25VQ32_ChipErase(void);

/* Register/Setting Commands *************************************************/
int32_t ZB25VQ32_WriteEnable(void);
int32_t ZB25VQ32_WriteDisable(void);
int32_t ZB25VQ32_ReadStatusRegister(uint8_t *Value);
int32_t ZB25VQ32_WriteStatusRegister(uint8_t Value);
int32_t ZB25VQ32_EnterDeepPowerDown(void);
int32_t ZB25VQ32_ReleaseDeepPowerDown(void);

/* ID/Security Commands ******************************************************/
int32_t ZB25VQ32_ReadID(uint8_t *ID);
int32_t ZB25VQ32_ReadSFDP(uint8_t *pData, uint32_t ReadAddr, uint16_t Size);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ZB25VQ32_H */
