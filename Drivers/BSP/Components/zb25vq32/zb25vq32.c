/**
  ******************************************************************************
  * @file    ZB25VQ32.c
  * @Author  MCD Application Team
  * @brief   This file provides the ZB25VQ32 drivers.
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

/* Includes ------------------------------------------------------------------*/
#include "zb25vq32.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @defgroup ZB25VQ32 ZB25VQ32
  * @{
  */
#define ZB25VQ32_SPI_TIMEOUT 1000U /* 1000ms */

SPI_HandleTypeDef hspi1;

/** @defgroup ZB25VQ32_Exported_Functions ZB25VQ32 Exported Functions
  * @{
  */



/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
   
  }
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,  GPIO_PIN_SET);
	
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    PA5     ------> SPI1_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */
    /* Reset peripherals */
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    PA5     ------> SPI1_SCK
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_5);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/**
  * @brief  Get Flash information
  * @param  pInfo pointer to Device Info structure
  * @retval Status
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_GetFlashInfo(ZB25VQ32_Info_t *pInfo)
{
  /* Configure the structure with the memory configuration */
  pInfo->FlashSize              = ZB25VQ32_FLASH_SIZE;
  pInfo->EraseSectorSize        = ZB25VQ32_BLOCK_64K;
  pInfo->EraseSectorsNumber     = (ZB25VQ32_FLASH_SIZE / ZB25VQ32_BLOCK_64K);
  pInfo->EraseSubSectorSize     = ZB25VQ32_SECTOR_4K;
  pInfo->EraseSubSectorNumber   = (ZB25VQ32_FLASH_SIZE / ZB25VQ32_SECTOR_4K);
  pInfo->EraseSubSector1Size    = ZB25VQ32_SECTOR_4K;
  pInfo->EraseSubSector1Number  = (ZB25VQ32_FLASH_SIZE / ZB25VQ32_SECTOR_4K);
  pInfo->ProgPageSize           = ZB25VQ32_PAGE_SIZE;
  pInfo->ProgPagesNumber        = (ZB25VQ32_FLASH_SIZE / ZB25VQ32_PAGE_SIZE);

  return ZB25VQ32_OK;
}

/**
  * @brief  Wait until Write In Progress (WIP) bit is equal to 0
  * @param  Ctx Component object pointer
  * @retval Status
  *    - ZB25VQ32_ERROR_AUTOPOLLING
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_AutoPollingMemReady(void)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t statusRegister;
  uint8_t cmd = ZB25VQ32_READ_STATUS_REG_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,  GPIO_PIN_RESET);
  /* Send the command */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
  else
  {
    do
    {
      if (HAL_SPI_Receive(&hspi1, &statusRegister, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
      {
        ret = ZB25VQ32_ERROR_AUTOPOLLING;
      }
    } while ((ret == ZB25VQ32_OK) && ((statusRegister & ZB25VQ32_SR_WIP) != 0U));
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Wait until Write Enable Latch (WEL) bit is set to 1
  * @param  Component object pointer
  * @retval Status
  *    - ZB25VQ32_ERROR_RECEIVE
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_AutoPollingMemReadyToWrite(void)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t statusRegister;
  uint8_t cmd = ZB25VQ32_READ_STATUS_REG_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send the command */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
  else
  {
    do
    {
      if (HAL_SPI_Receive(&hspi1, &statusRegister, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
      {
        ret = ZB25VQ32_ERROR_RECEIVE;
      }
    } while ((ret == ZB25VQ32_OK) && ((statusRegister & ZB25VQ32_SR_WREN) != ZB25VQ32_SR_WREN));
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/* Read/Write Array Commands (3 Byte Address Command Set) *********************/
/**
  * @brief  Reads an amount of data from the memory.
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start address
  * @param  Size Size of data to read in Byte
  * @retval Status
  *    - ZB25VQ32_ERROR_RECEIVE
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_Read(uint8_t *pData, uint32_t ReadAddr, uint16_t Size)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd[4U];

  /* Send the command */
  cmd[0U] = ZB25VQ32_READ_CMD;
  cmd[1U] = (uint8_t)((ReadAddr & 0x00FF0000U) >> 16);
  cmd[2U] = (uint8_t)((ReadAddr & 0x0000FF00U) >> 8);
  cmd[3U] = (uint8_t)(ReadAddr & 0x000000FFU);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, cmd, 4U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
  else
  {
    if (HAL_SPI_Receive(&hspi1, pData, Size, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
    {
      ret = ZB25VQ32_ERROR_RECEIVE;
    }
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Reads an amount of data from the memory.
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start address
  * @param  Size Size of data to read in Byte
  * @retval Status
  *    - ZB25VQ32_ERROR_RECEIVE
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_FastRead(uint8_t *pData, uint32_t ReadAddr, uint16_t Size)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd[5U];

  /* Send the command */
  cmd[0U] = ZB25VQ32_FAST_READ_CMD;
  cmd[1U] = (uint8_t)((ReadAddr & 0x00FF0000U) >> 16);
  cmd[2U] = (uint8_t)((ReadAddr & 0x0000FF00U) >> 8);
  cmd[3U] = (uint8_t)(ReadAddr & 0x000000FFU);
  cmd[4U] = 0xAAU; /* Dummy */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, cmd, 5U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
  else
  {
    if (HAL_SPI_Receive(&hspi1, pData, Size, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
    {
      ret = ZB25VQ32_ERROR_RECEIVE;
    }
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Writes an amount of data to the SPI memory.
  *    For 256 bytes page program, the 8 least significant address bits byte
  *    should be set to 0 this function otherwise returns ZB25VQ32_ERROR_ADDRESS
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ 256
  * @retval Status
  *    - ZB25VQ32_ERROR_ADDRESS
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_ERROR_TRANSMIT
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_PageProgram(uint8_t *pData, uint32_t WriteAddr, uint16_t Size)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd[4U];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  if ((Size >= ZB25VQ32_PAGE_SIZE) && ((WriteAddr & 0x000000FFU) != 0U))
  {
    ret = ZB25VQ32_ERROR_ADDRESS;
  }
  else
  {
    /* 1- Send Page Program (PP) command */
    cmd[0U] = ZB25VQ32_PAGE_PROG_CMD;
    cmd[1U] = (uint8_t)((WriteAddr & 0x00FF0000U) >> 16);
    cmd[2U] = (uint8_t)((WriteAddr & 0x0000FF00U) >> 8);
    cmd[3U] = (uint8_t)(WriteAddr & 0x000000FFU);
    if (HAL_SPI_Transmit(&hspi1, cmd, 4U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
    {
      ret = ZB25VQ32_ERROR_COMMAND;
    }
    else
    {
      /* 2- Send the data */
      if (HAL_SPI_Transmit(&hspi1, pData, Size, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
      {
        ret = ZB25VQ32_ERROR_TRANSMIT;
      }
    }
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Erases the specified sector of the SPI memory
  *         ZB25VQ32 support 4K size block erase command.
  * @param  Ctx Component object pointer
  * @param  SectorAddress Sector address to erase
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_SectorErase(uint32_t SectorAddress)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd[4U];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* 1- Send Sector Erase (SE) for erasing the data of chosen block */
  cmd[0U] = ZB25VQ32_SECTOR_ERASE_4K_CMD;
  cmd[1U] = (uint8_t)((SectorAddress & 0x00FF0000U) >> 16);
  cmd[2U] = (uint8_t)((SectorAddress & 0x0000FF00U) >> 8);
  cmd[3U] = (uint8_t)(SectorAddress & 0x000000FFU);
  if (HAL_SPI_Transmit(&hspi1, cmd, 4U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    return ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Erases the specified block of the SPI memory
  *         ZB25VQ32 64K size block erase command.
  * @param  Ctx Component object pointer
  * @param  BlockAddress Block address to erase
  * @param  BlockSize Block size to erase
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_BlockErase(uint32_t BlockAddress, ZB25VQ32_Erase_t BlockSize)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd[4U];

  /* Setup erase command */
  switch (BlockSize)
  {
    default :
    case ZB25VQ32_ERASE_4K :
      cmd[0U] = ZB25VQ32_SECTOR_ERASE_4K_CMD;
      break;

    case ZB25VQ32_ERASE_64K :
      cmd[0U] = ZB25VQ32_BLOCK_ERASE_64K_CMD;
      break;

    case ZB25VQ32_ERASE_CHIP :
      return ZB25VQ32_ChipErase();
      break;
  }

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* 1- Send Block Erase (BE) for erasing the data of chosen block */
  cmd[1U] = (uint8_t)((BlockAddress & 0x00FF0000U) >> 16);
  cmd[2U] = (uint8_t)((BlockAddress & 0x0000FF00U) >> 8);
  cmd[3U] = (uint8_t)(BlockAddress & 0x000000FFU);
  if (HAL_SPI_Transmit(&hspi1, cmd, 4U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Whole chip erase of the SPI memory
  * @param  Ctx Component object pointer
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_ChipErase(void)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd = ZB25VQ32_CHIP_ERASE_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* 1- Send Chip Erase (CE) command to erase whole chip */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/* Register/Setting Commands **************************************************/
/**
  * @brief  This function sets the (WEL) Write Enable Latch bit
  * @param  Ctx Component object pointer
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_WriteEnable(void)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd = ZB25VQ32_WRITE_ENABLE_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send Write Enable (WREN) command to set Write Enable Latch (WEL) bit in the Status Register */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  This function resets the (WEL) Write Enable Latch bit
  * @param  Ctx Component object pointer
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_WriteDisable(void)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd = ZB25VQ32_WRITE_DISABLE_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send Write Disable (WRDI) command to unset Write Enable Latch (WEL) bit in the Status Register */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    return ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Read Flash Status register
  * @param  Ctx Component object pointer
  * @param  Value pointer to status register value
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_ERROR_RECEIVE
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_ReadStatusRegister(uint8_t *Value)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd = ZB25VQ32_READ_STATUS_REG_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send the command */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
  else
  {
    if (HAL_SPI_Receive(&hspi1, Value, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
    {
      ret = ZB25VQ32_ERROR_RECEIVE;
    }
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Write Flash Status register value
  * @param  Ctx Component object pointer
  * @param  Value Status register value
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_WriteStatusRegister(uint8_t Value)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd[2U];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send the command */
  cmd[0U] = ZB25VQ32_WRITE_STATUS_REG_CMD;
  cmd[1U] = Value;
  if (HAL_SPI_Transmit(&hspi1, cmd, 2U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Deep power down
  *         The device is not active and all Write/Program/Erase instruction are ignored.
  * @param  Ctx Component object pointer
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_EnterDeepPowerDown(void)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd = ZB25VQ32_DEEP_POWER_DOWN_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send Deep Powerdown (DP) command to enter deep powerdown mode */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Release Deep power down
  *         The device is now active and all Write/Program/Erase instruction are available.
  * @param  Ctx Component object pointer
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_ReleaseDeepPowerDown(void)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd = ZB25VQ32_RELEASE_FROM_DEEP_POWER_DOWN_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send Release from Deep Powerdown (RDP) command to exit from deep powerdown mode */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/* ID/Security Commands *******************************************************/
/**
  * @brief  Read Flash 3 Byte IDs 
  * 9Fh(SPI Mode) Manufacturer ID = 5E Memory Type =40h Capacity = 16h
  *         Manufacturer ID, Memory type, Memory density
  * @param  Ctx Component object pointer
  * @param  ID pointer to flash id value
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_ERROR_RECEIVE
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_ReadID(uint8_t *ID)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd = ZB25VQ32_READ_ID_CMD;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send the command */
  if (HAL_SPI_Transmit(&hspi1, &cmd, 1U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
  else
  {
    if (HAL_SPI_Receive(&hspi1, ID, 3U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
    {
      ret = ZB25VQ32_ERROR_RECEIVE;
    }
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}

/**
  * @brief  Reads an amount of data from the memory
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start address
  * @param  Size Size of data to read in Byte
  * @retval Status
  *    - ZB25VQ32_ERROR_COMMAND
  *    - ZB25VQ32_ERROR_RECEIVE
  *    - ZB25VQ32_OK
  */
int32_t ZB25VQ32_ReadSFDP(uint8_t *pData, uint32_t ReadAddr, uint16_t Size)
{
  int32_t ret = ZB25VQ32_OK;
  uint8_t cmd[5U];

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  /* Send the command */
  cmd[0U] = ZB25VQ32_READ_SERIAL_FLASH_DISCO_PARAM_CMD;
  cmd[1U] = (uint8_t)((ReadAddr & 0x00FF0000U) >> 16);
  cmd[2U] = (uint8_t)(ReadAddr & 0x0000FF00U) >> 8;
  cmd[3U] = (uint8_t)(ReadAddr & 0x000000FFU);
  cmd[4U] = 0xAAU; /* Dummy */
  if (HAL_SPI_Transmit(&hspi1, cmd, 5U, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
  {
    ret = ZB25VQ32_ERROR_COMMAND;
  }
  else
  {
    if (HAL_SPI_Receive(&hspi1, pData, Size, ZB25VQ32_SPI_TIMEOUT) != HAL_OK)
    {
      ret = ZB25VQ32_ERROR_RECEIVE;
    }
  }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return ret;
}
