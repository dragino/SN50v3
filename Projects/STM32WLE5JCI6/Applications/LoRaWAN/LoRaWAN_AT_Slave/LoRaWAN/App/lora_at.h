/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_at.h
  * @author  MCD Application Team
  * @brief   Header for driver at.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LORA_AT_H__
#define __LORA_AT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32_adv_trace.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/*
 * AT Command Id errors. Note that they are in sync with ATError_description static array
 * in command.c
 */
typedef enum eATEerror
{
  AT_OK = 0,
  AT_ERROR,
  AT_PARAM_ERROR,
  AT_BUSY_ERROR,
  AT_TEST_PARAM_OVERFLOW,
  AT_NO_NET_JOINED,
  AT_RX_ERROR,
  AT_NO_CLASS_B_ENABLE,
  AT_DUTYCYCLE_RESTRICTED,
  AT_CRYPTO_ERROR,
  AT_MAX,
} ATEerror_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* AT printf */
#define AT_PRINTF(...)     do{  UTIL_ADV_TRACE_COND_FSend(VLEVEL_OFF, T_REG_OFF, TS_OFF, __VA_ARGS__);}while(0)
#define AT_PPRINTF(...)    do{ } while( UTIL_ADV_TRACE_OK \
                               != UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__) ) /*Polling Mode*/

/* AT Command strings. Commands start with AT */
#define AT_DEBUG      "+DEBUG"
#define AT_RESET      "Z"
#define AT_FDR        "+FDR"
#define AT_DEUI       "+DEUI"  
#define AT_APPEUI     "+APPEUI"  
#define AT_APPKEY     "+APPKEY"  
#define AT_DEVADDR    "+DADDR"  
#define AT_APPSKEY    "+APPSKEY"  
#define AT_NWKSKEY    "+NWKSKEY"
#define AT_ADR        "+ADR"
#define AT_TXP        "+TXP"
#define AT_DR         "+DR"
#define AT_DCS        "+DCS"
#define AT_PNM        "+PNM"
#define AT_RX2FQ      "+RX2FQ"
#define AT_RX2DR      "+RX2DR"
#define AT_RX1DL      "+RX1DL"
#define AT_RX2DL      "+RX2DL"
#define AT_JN1DL      "+JN1DL"
#define AT_JN2DL      "+JN2DL"
#define AT_NJM        "+NJM"
#define AT_NWKID      "+NWKID"
#define AT_FCU        "+FCU"
#define AT_FCD        "+FCD"
#define AT_CLASS      "+CLASS"
#define AT_JOIN       "+JOIN"
#define AT_NJS        "+NJS"
//#define AT_SEND       "+SEND"
//#define AT_SENDB      "+SENDB"
#define AT_RECV       "+RECV"
#define AT_RECVB      "+RECVB"
#define AT_VER        "+VER"
#define AT_CFM        "+CFM"
#define AT_CFS        "+CFS"
#define AT_SNR        "+SNR"
#define AT_RSSI       "+RSSI"
#define AT_TDC        "+TDC"
#define AT_PORT       "+PORT"
#define AT_DISAT      "+DISAT"
#define AT_PWORD      "+PWORD"
#define AT_CHS        "+CHS"
#define AT_CHE        "+CHE"
#define AT_SLEEP      "+SLEEP"
#define AT_CFG        "+CFG"
#define AT_PDTA       "+PDTA"
#define AT_PLDTA      "+PLDTA"
#define AT_CLRDTA     "+CLRDTA"
#define AT_EXT        "+EXT"
#define AT_BAT        "+BAT"
#define AT_WMOD       "+WMOD"
#define AT_ARTEMP     "+ARTEMP"
#define AT_CITEMP     "+CITEMP"
#define AT_SETCNT     "+SETCNT"
#define AT_DWELLT     "+DWELLT"
#define AT_RJTDC      "+RJTDC"
#define AT_RPL        "+RPL"
#define AT_DEBUG      "+DEBUG"
#define AT_TIMESTAMP  "+TIMESTAMP"
#define AT_LEAPSEC    "+LEAPSEC"
#define AT_SYNCMOD    "+SYNCMOD"
#define AT_SYNCTDC    "+SYNCTDC"
#define AT_PID        "+PID"
#define AT_UOA        "+UOA"
#define AT_DDETECT    "+DDETECT"
#define AT_SETMAXNBTRANS    "+SETMAXNBTRANS"
#define AT_UUID       "+UUID" 
#define AT_RXDATEST   "+RXDATEST"
#define AT_PNACKMD    "+PNACKMD"
#define AT_GETSENSORVALUE   "+GETSENSORVALUE"
#define AT_EXTPLOGIC  "+EPLOGIC"
#define AT_LEDALARM   "+LEDALARM"
#define AT_DEVNONCE  			  "+DEVNONCE"
#define AT_INTMOD1    "+INTMOD1"

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Return AT_OK in all cases
  * @param  param string of the AT command - unused
  * @retval AT_OK
  */
ATEerror_t AT_return_ok(const char *param);

/**
  * @brief  Return AT_ERROR in all cases
  * @param  param string of the AT command - unused
  * @retval AT_ERROR
  */
ATEerror_t AT_return_error(const char *param);

/* --------------- General commands --------------- */
/**
  * @brief  Print the version of the AT_Slave FW
  * @param  param String parameter
  * @retval AT_OK
  */
ATEerror_t AT_version_get(const char *param);

/**
  * @brief  Get the local time in UTC format
  * @param  param String parameter
  * @retval AT_OK
  */
ATEerror_t AT_LocalTime_get(const char *param);

ATEerror_t AT_LocalTime_set(const char *param);
/**
  * @brief  Trig a reset of the MCU
  * @param  param string of the AT command - unused
  * @retval AT_OK
  */
ATEerror_t AT_reset(const char *param);

/* --------------- Context Store commands --------------- */
/**
  * @brief  Restore factory settings in Eeprom
  * @param  param string of the AT command - unused
  * @retval AT_OK
  */
ATEerror_t AT_restore_factory_settings(const char *param);

/**
  * @brief  Store current settings in Eeprom
  * @param  param string of the AT command - unused
  * @retval AT_OK
  */
ATEerror_t AT_store_context(const char *param);

/* --------------- Information command --------------- */
/**
  * @brief  Get the battery level
  * @param  param String parameter
  * @retval AT_OK
  */
ATEerror_t AT_bat_get(const char *param);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __LORA_AT_H__ */
