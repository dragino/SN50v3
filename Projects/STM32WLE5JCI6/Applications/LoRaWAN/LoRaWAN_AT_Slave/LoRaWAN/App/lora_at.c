/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_at.c
  * @author  MCD Application Team
  * @brief   AT command API
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "lora_at.h"
#include "sys_app.h"
#include "stm32_tiny_sscanf.h"
#include "app_version.h"
#include "stm32_systime.h"
#include "subghz_phy_version.h"
#include "test_rf.h"
#include "adc_if.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "radio.h"
#include "lora_info.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static bool ClassBEnableRequest = false;

/*!
 * User application buffer
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Get 4 bytes values in hex
  * @param  from string containing the 16 bytes, something like ab:cd:01:21
  * @param  value buffer that will contain the bytes read
  * @retval The number of bytes read
  */
static int32_t sscanf_uint32_as_hhx(const char *from, uint32_t *value);

/**
  * @brief  Get 16 bytes values in hex
  * @param  from string containing the 16 bytes, something like ab:cd:01:...
  * @param  pt buffer that will contain the bytes read
  * @retval The number of bytes read
  */
static int32_t sscanf_16_hhx(const char *from, uint8_t *pt);

/**
  * @brief  Print 4 bytes as %02x
  * @param  value containing the 4 bytes to print
  */
static void print_uint32_as_02x(uint32_t value);

/**
  * @brief  Print 16 bytes as %02X
  * @param  pt pointer containing the 16 bytes to print
  */
static void print_16_02x(uint8_t *pt);

/**
  * @brief  Print 8 bytes as %02X
  * @param  pt pointer containing the 8 bytes to print
  */
static void print_8_02x(uint8_t *pt);

/**
  * @brief  Print an int
  * @param  value to print
  */
static void print_d(int32_t value);

/**
  * @brief  Print an unsigned int
  * @param  value to print
  */
static void print_u(uint32_t value);

/**
  * @brief  Certif Rejoin timer callback function
  * @param  context ptr of Certif Rejoin context
  */
static void OnCertifTimer(void *context);

/**
  * @brief  Certif send function
  */
static void CertifSend(void);

/**
  * @brief  Check if character in parameter is alphanumeric
  * @param  Char for the alphanumeric check
  */
static int32_t isHex(char Char);

/**
  * @brief  Converts hex string to a nibble ( 0x0X )
  * @param  Char hex string
  * @retval the nibble. Returns 0xF0 in case input was not an hex string (a..f; A..F or 0..9)
  */
static uint8_t Char2Nibble(char Char);

/**
  * @brief  Convert a string into a buffer of data
  * @param  str string to convert
  * @param  data output buffer
  * @param  Size of input string
  */
static int32_t stringToData(const char *str, uint8_t *data, uint32_t Size);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
ATEerror_t AT_return_ok(const char *param)
{
  return AT_OK;
}

ATEerror_t AT_return_error(const char *param)
{
  return AT_ERROR;
}

ATEerror_t AT_reset(const char *param)
{
  /* USER CODE BEGIN AT_reset_1 */

  /* USER CODE END AT_reset_1 */
  NVIC_SystemReset();
  /* USER CODE BEGIN AT_reset_2 */

  /* USER CODE END AT_reset_2 */
}

/* ---------------  Context Store commands --------------------------- */
ATEerror_t AT_restore_factory_settings(const char *param)
{
  /* USER CODE BEGIN AT_restore_factory_settings_1 */

  /* USER CODE END AT_restore_factory_settings_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    /* System Reboot*/
    NVIC_SystemReset();
  }

  return AT_OK;
  /* USER CODE BEGIN AT_restore_factory_settings_2 */

  /* USER CODE END AT_restore_factory_settings_2 */
}

/* --------------- Information command --------------- */
ATEerror_t AT_bat_get(const char *param)
{
  /* USER CODE BEGIN AT_bat_get_1 */

  /* USER CODE END AT_bat_get_1 */
  print_d(SYS_GetBatteryLevel());

  return AT_OK;
  /* USER CODE BEGIN AT_bat_get_2 */

  /* USER CODE END AT_bat_get_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
static int32_t sscanf_uint32_as_hhx(const char *from, uint32_t *value)
{
  /* USER CODE BEGIN sscanf_uint32_as_hhx_1 */

  /* USER CODE END sscanf_uint32_as_hhx_1 */
  return tiny_sscanf(from, "%hhx %hhx %hhx %hhx",
                     &((unsigned char *)(value))[3],
                     &((unsigned char *)(value))[2],
                     &((unsigned char *)(value))[1],
                     &((unsigned char *)(value))[0]);
  /* USER CODE BEGIN sscanf_uint32_as_hhx_2 */

  /* USER CODE END sscanf_uint32_as_hhx_2 */
}

static int32_t sscanf_16_hhx(const char *from, uint8_t *pt)
{
  /* USER CODE BEGIN sscanf_16_hhx_1 */

  /* USER CODE END sscanf_16_hhx_1 */
  return tiny_sscanf(from, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                     &pt[0], &pt[1], &pt[2], &pt[3], &pt[4], &pt[5], &pt[6],
                     &pt[7], &pt[8], &pt[9], &pt[10], &pt[11], &pt[12], &pt[13],
                     &pt[14], &pt[15]);
  /* USER CODE BEGIN sscanf_16_hhx_2 */

  /* USER CODE END sscanf_16_hhx_2 */
}

static void print_uint32_as_02x(uint32_t value)
{
  /* USER CODE BEGIN print_uint32_as_02x_1 */

  /* USER CODE END print_uint32_as_02x_1 */
  AT_PRINTF("%02X %02X %02X %02X\r\n",
            (unsigned)((unsigned char *)(&value))[3],
            (unsigned)((unsigned char *)(&value))[2],
            (unsigned)((unsigned char *)(&value))[1],
            (unsigned)((unsigned char *)(&value))[0]);
  /* USER CODE BEGIN print_uint32_as_02x_2 */

  /* USER CODE END print_uint32_as_02x_2 */
}

static void print_16_02x(uint8_t *pt)
{
  /* USER CODE BEGIN print_16_02x_1 */

  /* USER CODE END print_16_02x_1 */
  AT_PRINTF("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            pt[0], pt[1], pt[2], pt[3],
            pt[4], pt[5], pt[6], pt[7],
            pt[8], pt[9], pt[10], pt[11],
            pt[12], pt[13], pt[14], pt[15]);
  /* USER CODE BEGIN print_16_02x_2 */

  /* USER CODE END print_16_02x_2 */
}

static void print_8_02x(uint8_t *pt)
{
  /* USER CODE BEGIN print_8_02x_1 */

  /* USER CODE END print_8_02x_1 */
  AT_PRINTF("%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            pt[0], pt[1], pt[2], pt[3], pt[4], pt[5], pt[6], pt[7]);
  /* USER CODE BEGIN print_8_02x_2 */

  /* USER CODE END print_8_02x_2 */
}

static void print_d(int32_t value)
{
  /* USER CODE BEGIN print_d_1 */

  /* USER CODE END print_d_1 */
  AT_PRINTF("%d\r\n", value);
  /* USER CODE BEGIN print_d_2 */

  /* USER CODE END print_d_2 */
}

static void print_u(uint32_t value)
{
  /* USER CODE BEGIN print_u_1 */

  /* USER CODE END print_u_1 */
  AT_PRINTF("%u\r\n", value);
  /* USER CODE BEGIN print_u_2 */

  /* USER CODE END print_u_2 */
}

static uint8_t Char2Nibble(char Char)
{
  if (((Char >= '0') && (Char <= '9')))
  {
    return Char - '0';
  }
  else if (((Char >= 'a') && (Char <= 'f')))
  {
    return Char - 'a' + 10;
  }
  else if ((Char >= 'A') && (Char <= 'F'))
  {
    return Char - 'A' + 10;
  }
  else
  {
    return 0xF0;
  }
  /* USER CODE BEGIN CertifSend_2 */

  /* USER CODE END CertifSend_2 */
}

static int32_t stringToData(const char *str, uint8_t *data, uint32_t Size)
{
  /* USER CODE BEGIN stringToData_1 */

  /* USER CODE END stringToData_1 */
  char hex[3];
  hex[2] = 0;
  int32_t ii = 0;
  while (Size-- > 0)
  {
    hex[0] = *str++;
    hex[1] = *str++;

    /*check if input is hex */
    if ((isHex(hex[0]) == -1) || (isHex(hex[1]) == -1))
    {
      return -1;
    }
    /*check if input is even nb of character*/
    if ((hex[1] == '\0') || (hex[1] == ','))
    {
      return -1;
    }
    data[ii] = (Char2Nibble(hex[0]) << 4) + Char2Nibble(hex[1]);

    ii++;
  }

  return 0;
  /* USER CODE BEGIN stringToData_2 */

  /* USER CODE END stringToData_2 */
}

static int32_t isHex(char Char)
{
  /* USER CODE BEGIN isHex_1 */

  /* USER CODE END isHex_1 */
  if (((Char >= '0') && (Char <= '9')) ||
      ((Char >= 'a') && (Char <= 'f')) ||
      ((Char >= 'A') && (Char <= 'F')))
  {
    return 0;
  }
  else
  {
    return -1;
  }
  /* USER CODE BEGIN isHex_2 */

  /* USER CODE END isHex_2 */
}

/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
