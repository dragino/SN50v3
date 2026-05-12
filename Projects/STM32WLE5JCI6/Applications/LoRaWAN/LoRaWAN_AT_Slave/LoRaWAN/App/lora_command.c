/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_command.c
  * @author  MCD Application Team
  * @brief   Main command driver dedicated to command AT
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
#include <string.h>
#include "platform.h"
#include "lora_at.h"
#include "lora_command.h"
#include "utilities.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32wlxx_hal.h"
#include "mw_log_conf.h"
#include "lora_app.h"
#include "flash_eraseprogram.h"
#include "flash_if.h"
#include "version.h"
#include "bsp.h"
#include "stm32_systime.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* comment the following to have help message */
/* #define NO_HELP */
/* #define AT_RADIO_ACCESS */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief  Structure defining an AT Command
  */
struct ATCommand_s
{
  const char *string;                       /*< command string, after the "AT" */
  const int32_t size_string;                /*< size of the command string, not including the final \0 */
  ATEerror_t (*get)(const char *param);     /*< =? after the string to get the current value*/
  ATEerror_t (*set)(const char *param);     /*< = (but not =?\0) after the string to set a value */
  ATEerror_t (*run)(const char *param);     /*< \0 after the string - run the command */
#if !defined(NO_HELP)
  const char *help_string;                  /*< to be printed when ? after the string */
#endif /* !NO_HELP */
};

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define CMD_SIZE                        540
#define CIRC_BUFF_SIZE                  8

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  Array corresponding to the description of each possible AT Error
  */
static const char *const ATError_description[] =
{
  "\r\nOK\r\n",                     /* AT_OK */
  "\r\nAT_ERROR\r\n",               /* AT_ERROR */
  "\r\nAT_PARAM_ERROR\r\n",         /* AT_PARAM_ERROR */
  "\r\nAT_BUSY_ERROR\r\n",          /* AT_BUSY_ERROR */
  "\r\nAT_TEST_PARAM_OVERFLOW\r\n", /* AT_TEST_PARAM_OVERFLOW */
  "\r\nAT_NO_NETWORK_JOINED\r\n",   /* AT_NO_NET_JOINED */
  "\r\nAT_RX_ERROR\r\n",            /* AT_RX_ERROR */
  "\r\nAT_NO_CLASS_B_ENABLE\r\n",   /* AT_NO_CLASS_B_ENABLE */
  "\r\nAT_DUTYCYCLE_RESTRICTED\r\n", /* AT_DUTYCYCLE_RESTRICTED */
  "\r\nAT_CRYPTO_ERROR\r\n",        /* AT_CRYPTO_ERROR */
  "\r\nerror unknown\r\n",          /* AT_MAX */
};

#define ARGC_LIMIT 16
#define ATCMD_SIZE (242 * 2 + 18)
#define PORT_LEN 4

#define QUERY_CMD		0x01
#define EXECUTE_CMD		0x02
#define DESC_CMD        0x03
#define SET_CMD			0x04

static char ReceivedData[255];
static unsigned ReceivedDataSize = 0;
static uint8_t ReceivedDataPort=0;
static TimerEvent_t ATcommandsTimer;
static void OnTimerATcommandsEvent(void *context);
static uint8_t at_PASSWORD_comp(char *argv);
uint8_t dwelltime;
uint8_t parse_flag=0;
bool atrecve_flag=0;
bool debug_flags=0;
bool message_flags=0;
uint8_t atcmd[ATCMD_SIZE];
uint16_t atcmd_index = 0;
volatile bool g_atcmd_processing = false;
//uint8_t g_default_key[LORA_KEY_LENGTH] = {0x41, 0x53, 0x52, 0x36, 0x35, 0x30, 0x58, 0x2D, 
//                                          0x32, 0x30, 0x31, 0x38, 0x31, 0x30, 0x33, 0x30};

extern uint8_t password_len;
extern uint8_t password_get[8];
extern uint16_t inmode_delay,inmode2_delay,inmode3_delay;	
extern uint8_t inmode,inmode2,inmode3;
extern uint16_t LoRaMacDevNonce;
extern bool join_mothod;
extern LoRaMainCallback_t *LoRaMainCallbacks;
extern char *band_string;
extern uint32_t LoRaMacState;
extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;
extern TimerEvent_t TxTimer;
extern TimerEvent_t ReJoinTimer;
extern TimerEvent_t CalibrationUTCTimer;
extern TimerEvent_t sht20tempcompTimer;
extern TimerEvent_t exttempcompTimer;
extern TimerEvent_t DS18B20IDTimer;
extern TimerEvent_t DownlinkDetectTimeoutTimer;
extern TimerEvent_t UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer;
extern TimerEvent_t RedLEDAlarmTimer1;

uint8_t symbtime1_value;
uint8_t symbtime2_value;
uint8_t flag1=0;
uint8_t flag2=0;
uint8_t exitintmode;
uint8_t exitmode;
uint8_t s31f_ext;
uint8_t valid_flag=0;
uint16_t valid_time=0;
extern uint8_t Ext;
extern __IO uint8_t is_time_to_send;
extern uint32_t count;
extern __IO uint32_t write_address;
extern bool sleep_status;
uint16_t power_time=0;
extern uint32_t APP_TX_DUTYCYCLE;
extern uint16_t REJOIN_TX_DUTYCYCLE;
extern uint8_t response_level;

extern uint32_t UpLinkCounter;
extern bool exit_send_flag;
extern uint8_t work_mode;
extern uint16_t collection_interval;
extern short comp_temp1_sht20,comp_temp2_sht20;
extern short comp_temp1_exttemp,comp_temp2_exttemp;
extern uint8_t temp_change;
extern uint8_t collection_second;

extern uint8_t is_PDTA_command;
extern uint8_t is_PLDTA_command;
extern bool FDR_status;

extern uint8_t currentLeapSecond;
extern uint8_t time_synchronization_method;
extern uint8_t time_synchronization_interval;
extern uint8_t pid_flag;
extern uint8_t RX2DR_setting_status;
extern uint8_t pnackmd_switch;
extern uint8_t extpower_logic;

extern uint8_t downlink_detect_switch;
extern uint16_t downlink_detect_timeout;

extern uint8_t confirmed_uplink_counter_retransmission_increment_switch;
extern uint8_t confirmed_uplink_retransmission_nbtrials;

extern uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch;
extern uint8_t LinkADR_NbTrans_retransmission_nbtrials;
extern uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout;

extern uint32_t uuid1_in_sflash,uuid2_in_sflash;

uint8_t write_key_in_flash_status=0,write_config_in_flash_status=0;
extern uint8_t led_alarm_switch;
extern uint8_t external_sensor_type_for_work_mode3;
extern uint16_t external_sensor_sample_interval_for_work_mode3;
extern uint8_t external_sensor_sample_sum_for_work_mode3;
extern uint8_t temp_alarm_switch_for_work_mode3;
extern void save_devNonce_buff(void);
/**
  * @brief  Array of all supported AT Commands
  */
static const struct ATCommand_s ATCommand[] =
{
//////  {
//////    .string = AT_LTIME,
//////    .size_string = sizeof(AT_LTIME) - 1,
//////#ifndef NO_HELP
//////    .help_string = "AT"AT_LTIME" Get the local time in UTC format\r\n",
//////#endif /* !NO_HELP */
//////    .get = AT_LocalTime_get,
//////    .set = AT_LocalTime_set,
//////    .run = AT_return_error,
//////  },

//////  {
//////    .string = AT_RESET,
//////    .size_string = sizeof(AT_RESET) - 1,
//////#ifndef NO_HELP
//////    .help_string = "AT"AT_RESET" Trig a MCU reset\r\n",
//////#endif /* !NO_HELP */
//////    .get = AT_return_error,
//////    .set = AT_return_error,
//////    .run = AT_reset,
//////  },

//////  /* Context Store commands */
//////  {
//////    .string = AT_RFS,
//////    .size_string = sizeof(AT_RFS) - 1,
//////#ifndef NO_HELP
//////    .help_string = "AT"AT_RFS ": Restore EEPROM Factory Settings\r\n",
//////#endif /* !NO_HELP */
//////    .get = AT_return_error,
//////    .set = AT_return_error,
//////    .run = AT_restore_factory_settings,
//////  },

//////  /* Information command */
//////  {
//////    .string = AT_BAT,
//////    .size_string = sizeof(AT_BAT) - 1,
//////#ifndef NO_HELP
//////    .help_string = "AT"AT_BAT" Get the battery Level in mV\r\n",
//////#endif /* !NO_HELP */
//////    .get = AT_bat_get,
//////    .set = AT_return_error,
//////    .run = AT_return_error,
//////  },
//////  /* USER CODE BEGIN ATCommand */

//////  /* USER CODE END ATCommand */
};

typedef struct {
	char *cmd;
	int (*fn)(int opt, int argc, char *argv[]);	
}at_cmd_t;

//AT functions
static int at_debug_func(int opt, int argc, char *argv[]);
static int at_reset_func(int opt, int argc, char *argv[]);
static int at_fdr_func(int opt, int argc, char *argv[]);
static int at_deui_func(int opt, int argc, char *argv[]);
static int at_appeui_func(int opt, int argc, char *argv[]);
static int at_appkey_func(int opt, int argc, char *argv[]);
static int at_devaddr_func(int opt, int argc, char *argv[]);
static int at_appskey_func(int opt, int argc, char *argv[]);
static int at_nwkskey_func(int opt, int argc, char *argv[]);
static int at_adr_func(int opt, int argc, char *argv[]);
static int at_txp_func(int opt, int argc, char *argv[]);
static int at_dr_func(int opt, int argc, char *argv[]);
static int at_dcs_func(int opt, int argc, char *argv[]);
static int at_pnm_func(int opt, int argc, char *argv[]);
static int at_rx2fq_func(int opt, int argc, char *argv[]);
static int at_rx2dr_func(int opt, int argc, char *argv[]);
static int at_rx1dl_func(int opt, int argc, char *argv[]);
static int at_rx2dl_func(int opt, int argc, char *argv[]);
static int at_jn1dl_func(int opt, int argc, char *argv[]);
static int at_jn2dl_func(int opt, int argc, char *argv[]);
static int at_njm_func(int opt, int argc, char *argv[]);
static int at_nwkid_func(int opt, int argc, char *argv[]);
static int at_fcu_func(int opt, int argc, char *argv[]);
static int at_fcd_func(int opt, int argc, char *argv[]);
static int at_class_func(int opt, int argc, char *argv[]);
static int at_join_func(int opt, int argc, char *argv[]);
static int at_njs_func(int opt, int argc, char *argv[]);
static int at_recv_func(int opt, int argc, char *argv[]);
static int at_recvb_func(int opt, int argc, char *argv[]);
static int at_ver_func(int opt, int argc, char *argv[]);
static int at_cfm_func(int opt, int argc, char *argv[]);
//static int at_cfs_func(int opt, int argc, char *argv[]);
static int at_snr_func(int opt, int argc, char *argv[]);
static int at_rssi_func(int opt, int argc, char *argv[]);
static int at_tdc_func(int opt, int argc, char *argv[]);
static int at_port_func(int opt, int argc, char *argv[]);
//static int at_disat_func(int opt, int argc, char *argv[]);
static int at_pword_func(int opt, int argc, char *argv[]);
static int at_chs_func(int opt, int argc, char *argv[]);
#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
static int at_che_func(int opt, int argc, char *argv[]);
#endif
static int at_pdta_func(int opt, int argc, char *argv[]);
static int at_pldta_func(int opt, int argc, char *argv[]);
static int at_clrdta_func(int opt, int argc, char *argv[]);
static int at_sleep_func(int opt, int argc, char *argv[]);
static int at_ext_func(int opt, int argc, char *argv[]);
static int at_bat_func(int opt, int argc, char *argv[]);
static int at_cfg_func(int opt, int argc, char *argv[]);
static int at_wmod_func(int opt, int argc, char *argv[]);
static int at_artemp_func(int opt, int argc, char *argv[]);
static int at_citemp_func(int opt, int argc, char *argv[]);
static int at_setcnt_func(int opt, int argc, char *argv[]);
static int at_intmod1_func(int opt, int argc, char *argv[]);
#if defined ( REGION_AU915 ) || defined ( REGION_AS923 )
static int at_dwellt_func(int opt, int argc, char *argv[]);
#endif
static int at_rjtdc_func(int opt, int argc, char *argv[]);
static int at_rpl_func(int opt, int argc, char *argv[]);
static int at_timestamp_func(int opt, int argc, char *argv[]);
static int at_leapsec_func(int opt, int argc, char *argv[]);
static int at_syncmod_func(int opt, int argc, char *argv[]);
static int at_synctdc_func(int opt, int argc, char *argv[]);
static int at_pid_func(int opt, int argc, char *argv[]);
static int at_uoa_func(int opt, int argc, char *argv[]);
static int at_downlink_detect_func(int opt, int argc, char *argv[]);
static int at_setmaxnbtrans_func(int opt, int argc, char *argv[]);
static int at_uuid_func(int opt, int argc, char *argv[]);
static int at_rxdatatest_func(int opt, int argc, char *argv[]);
static int at_pnackmd_func(int opt, int argc, char *argv[]);
static int at_getsensorvalue_func(int opt, int argc, char *argv[]);
static int at_extplogic_func(int opt, int argc, char *argv[]);
static int at_ledalarm_func(int opt, int argc, char *argv[]);
static int at_devnonce_func(int opt, int argc, char *argv[]);

static at_cmd_t g_at_table[] = {
	  {AT_DEBUG, at_debug_func},
		{AT_RESET, at_reset_func},
		{AT_FDR, at_fdr_func},
	  {AT_DEUI, at_deui_func},
		{AT_APPEUI, at_appeui_func},
		{AT_APPKEY, at_appkey_func},
		{AT_DEVADDR, at_devaddr_func},
		{AT_APPSKEY, at_appskey_func},
		{AT_NWKSKEY, at_nwkskey_func},	
		{AT_ADR, at_adr_func},
		{AT_TXP, at_txp_func},
		{AT_DR, at_dr_func},
		{AT_DCS, at_dcs_func},
		{AT_PNM, at_pnm_func},
		{AT_RX2FQ, at_rx2fq_func},
		{AT_RX2DR, at_rx2dr_func},
		{AT_RX1DL, at_rx1dl_func},
		{AT_RX2DL, at_rx2dl_func},
		{AT_JN1DL, at_jn1dl_func},
		{AT_JN2DL, at_jn2dl_func},
		{AT_NJM, at_njm_func},
		{AT_NWKID, at_nwkid_func},
		{AT_FCU, at_fcu_func},
		{AT_FCD, at_fcd_func},
		{AT_CLASS, at_class_func},
		{AT_JOIN, at_join_func},
		{AT_NJS, at_njs_func},
		{AT_RECVB,at_recvb_func},
		{AT_RECV,at_recv_func},		
		{AT_VER, at_ver_func},
		{AT_CFM, at_cfm_func},
		{AT_SNR, at_snr_func},
		{AT_RSSI, at_rssi_func},
		{AT_TDC, at_tdc_func},
		{AT_PORT, at_port_func},
		{AT_PWORD, at_pword_func},
		{AT_CHS, at_chs_func},
		#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
		{AT_CHE, at_che_func},
		#endif
		{AT_INTMOD1, at_intmod1_func},
//		{AT_PDTA, at_pdta_func},
//		{AT_PLDTA, at_pldta_func},
//		{AT_CLRDTA, at_clrdta_func},
		{AT_SLEEP, at_sleep_func},
//		{AT_EXT, at_ext_func},
//		{AT_BAT, at_bat_func},
		{AT_CFG, at_cfg_func},
//		{AT_WMOD, at_wmod_func},
//		{AT_ARTEMP, at_artemp_func},
//		{AT_CITEMP, at_citemp_func},
//		{AT_SETCNT, at_setcnt_func},
		#if defined ( REGION_AU915 ) || defined ( REGION_AS923 )
		{AT_DWELLT, at_dwellt_func},
		#endif
		{AT_RJTDC, at_rjtdc_func},
		{AT_RPL, at_rpl_func},
		{AT_TIMESTAMP, at_timestamp_func},
		{AT_LEAPSEC, at_leapsec_func},
		{AT_SYNCMOD, at_syncmod_func},
		{AT_SYNCTDC, at_synctdc_func},
//		{AT_PID, at_pid_func},
//		{AT_UOA, at_uoa_func},
		{AT_DDETECT, at_downlink_detect_func},
		{AT_SETMAXNBTRANS, at_setmaxnbtrans_func},
		{AT_UUID, at_uuid_func},
		{AT_RXDATEST,at_rxdatatest_func},
//		{AT_PNACKMD,at_pnackmd_func},
		{AT_GETSENSORVALUE,at_getsensorvalue_func},
//		{AT_EXTPLOGIC,at_extplogic_func},
//		{AT_LEDALARM,at_ledalarm_func},
		{AT_DEVNONCE,at_devnonce_func},
};

#define AT_TABLE_SIZE	(sizeof(g_at_table) / sizeof(at_cmd_t))

uint8_t rxdata_change_hex(const char *hex, uint8_t *bin)
{
	uint8_t leg_temp=0;
  uint8_t flags=0;
	uint8_t *cur = bin;
  uint16_t hex_length = strlen(hex);
  const char *hex_end = hex + hex_length;
  uint8_t num_chars = 0;
  uint8_t byte = 0;
	
  while (hex < hex_end) {
        flags=0;
        if ('A' <= *hex && *hex <= 'F') {
            byte |= 10 + (*hex - 'A');
        } else if ('a' <= *hex && *hex <= 'f') {
            byte |= 10 + (*hex - 'a');
        } else if ('0' <= *hex && *hex <= '9') {
            byte |= *hex - '0';
        } else if (*hex == ' ') {
           flags=1;
        }else {
            return -1;
        }
        hex++;
        
        if(flags==0)
        {
          num_chars++;
          if (num_chars >= 2) {
              num_chars = 0;
              *cur++ = byte;
							leg_temp++;
              byte = 0;
             } else {
              byte <<= 4;
             }
        }
    }	
	
	 return leg_temp;
}

void set_at_receive(uint8_t AppPort, uint8_t* Buff, uint8_t BuffSize)
{
  if (255 <= BuffSize)
    BuffSize = 255;
  memcpy1((uint8_t *)ReceivedData, Buff, BuffSize);
  ReceivedDataSize = BuffSize;
  ReceivedDataPort = AppPort;
}

static int hex2bin(const char *hex, uint8_t *bin, uint16_t bin_length)
{
	  uint8_t flags=0;
    uint16_t hex_length = strlen(hex);
    const char *hex_end = hex + hex_length;
    uint8_t *cur = bin;
    uint8_t num_chars = 0;
    uint8_t byte = 0;

    if ((hex_length + 1) / 2 > bin_length) {
       if((hex_length!=11)&&(hex_length!=23)&&(hex_length!=47))
       {
        return -1;
       }
    }

    while (hex < hex_end) {
        flags=0;
        if ('A' <= *hex && *hex <= 'F') {
            byte |= 10 + (*hex - 'A');
        } else if ('a' <= *hex && *hex <= 'f') {
            byte |= 10 + (*hex - 'a');
        } else if ('0' <= *hex && *hex <= '9') {
            byte |= *hex - '0';
        } else if (*hex == ' ') {
           flags=1;
        }else {
            return -1;
        }
        hex++;
        
        if(flags==0)
        {
          num_chars++;
          if (num_chars >= 2) {
              num_chars = 0;
              *cur++ = byte;
              byte = 0;
             } else {
              byte <<= 4;
             }
        }
    }
		
    return cur - bin;
}

static int at_debug_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS;   
          debug_flags=1;
					snprintf((char *)atcmd, ATCMD_SIZE, "Enter Debug mode\r\n");
					
          break;
        }
		case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Set more info output\r\n");
			    break;
				}		
		
     default: break;
    }
    
    return ret;
}

static int at_reset_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS;
          *((uint8_t *)(0x2000F00A))=0x11;	
          HAL_Delay(100);					
          NVIC_SystemReset();
          break;
        }
				
		case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Trig a reset of the MCU\r\n");
			    break;
				}
		
     default: break;
    }
    
    return ret;
}

static int at_fdr_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS;   
					uint8_t status[128]={0};
					memset(status, 0x00, 128);
					__disable_irq();
//					flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
					FLASH_IF_Erase((void *)FLASH_USER_START_ADDR_CONFIG, FLASH_PAGE_SIZE);
					HAL_Delay(5);
//					if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
					if(FLASH_IF_Write((void *)FLASH_USER_START_ADDR_CONFIG, status, 128) ==FLASH_IF_ERROR)	
					{
						snprintf((char *)atcmd, ATCMD_SIZE, "FDR error\r\n");
					}
					__enable_irq();
		
					HAL_Delay(100);
          NVIC_SystemReset();
          break;
        }
				
		case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Reset Parameters to Factory Default, Keys Reserve\r\n");
					break;
				}
		
     default: break;
    }
    
    return ret;
}

static int at_deui_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[8];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            lora_config_deveui_get(buf);                                                                                                          
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 8);
            if (length == 8) {
                    lora_config_deveui_set(buf);						
                    ret = LWAN_SUCCESS; 
                    write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Device EUI\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_appeui_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[8];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            lora_config_appeui_get(buf);	                                                                                                           
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 8);
            if (length == 8) {
                    lora_config_appeui_set(buf);
                    ret = LWAN_SUCCESS;
							      write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Application EUI\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_appkey_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[16];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            lora_config_appkey_get(buf);	                                                                                                            
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);								
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 16);
            if (length == 16) {
                    lora_config_appkey_set(buf);                  
                    ret = LWAN_SUCCESS;
                    write_key_in_flash_status=1;							
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Application Key\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_devaddr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint32_t devaddr;
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;					
            devaddr=lora_config_devaddr_get();	                                                                                                             
            snprintf((char *)atcmd, ATCMD_SIZE, "%08X\r\n", (unsigned int)devaddr);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
					
            uint8_t buf[4];
            length = hex2bin((const char *)argv[0], buf, 4);
            if (length == 4) {
							      devaddr = buf[0] << 24 | buf[1] << 16 | buf[2] <<8 | buf[3];
                    lora_config_devaddr_set(devaddr);
                    ret = LWAN_SUCCESS;
							      write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Device Address\r\n");
					break;
				}
        default: break;
    }
    
    return ret;
}

static int at_appskey_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[16];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            lora_config_appskey_get(buf);                                                                                                  
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);           
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 16);
            if (length == 16) {
                    lora_config_appskey_set(buf);
                    ret = LWAN_SUCCESS;   
							      write_key_in_flash_status=1;
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Application Session Key\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_nwkskey_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t length;
    uint8_t buf[16];
        
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            lora_config_nwkskey_get(buf);	                                                                                                           
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", \
                             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);            
            break;
        }

        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 16);
            if (length == 16) {
                    lora_config_nwkskey_set(buf);
                    ret = LWAN_SUCCESS; 
                    write_key_in_flash_status=1;							
                    atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Network Session Key\r\n");
					break;
				}
				
        default: break;
    }
    
    return ret;
}

static int at_adr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t adr;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_ADR;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.AdrEnable);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            adr = strtol((const char *)argv[0], NULL, 0);
            if(adr<2)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_ADR;
							  mib.Param.AdrEnable = adr;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
									
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Adaptive Data Rate setting. (0: off, 1: on)\r\n");
					break;
				}
				
        default: break;
    }

    return ret;
}

static int at_txp_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t power;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						
            
						mib.Type = MIB_CHANNELS_TX_POWER;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.ChannelsTxPower);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            power = strtol((const char *)argv[0], NULL, 0);
            if((power<16)||((power>=40)&&(power<=52)))
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_CHANNELS_TX_POWER;
							  mib.Param.ChannelsTxPower = power;
								status=LoRaMacMibSetRequestConfirm(&mib);
								
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;					
        }
						
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Transmit Power (0-5, MAX:0, MIN:5, according to LoRaWAN Spec)\r\n");
					break;
				}	
        default: break;
    }

    return ret;
}

static int at_dr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t datarate;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_CHANNELS_DATARATE;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.ChannelsDatarate);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            datarate = strtol((const char *)argv[0], NULL, 0);

						#if defined( REGION_AS923 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_AU915 )
						if((datarate==7)||(datarate>=14))
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_CN470 )
						if(datarate>=6)	
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_CN779 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_EU433 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_IN865 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_EU868 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;	
						}
				    #elif defined( REGION_KR920 )
						if(datarate>=6)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}
				    #elif defined( REGION_US915 )
						if(((datarate>=5)&&(datarate<=7))||(datarate>=14))
						{
						ret = LWAN_PARAM_ERROR;break;								
						}
				    #elif defined( REGION_RU864 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;			
						}	
				    #elif defined( REGION_KZ865 )
						if(datarate>=8)
						{
						ret = LWAN_PARAM_ERROR;break;							
						}			
				    #endif
		
						lora_config_tx_datarate_set(datarate) ;
						snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after AT+ADR=0\r\n");
						ret = LWAN_SUCCESS;              
						write_config_in_flash_status=1;
						
            break;
        }
						
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Data Rate. (0-7 corresponding to DR_X)\r\n");
					break;
				}	
        default: break;
    }

    return ret;
}

static int at_dcs_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
            if (lora_config_duty_cycle_get() == LORA_ENABLE)											
					  {
							status=1;
						}
						else
						{
							status=0;							
						}	
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", status);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
					  ret = LWAN_SUCCESS;
					  write_config_in_flash_status=1;
					  atcmd[0] = '\0';
					
            if(status==0)
            {
                lora_config_duty_cycle_set(LORA_DISABLE);								                
            }
						else if(status==1)
						{
							  lora_config_duty_cycle_set(LORA_ENABLE);
						}
            else
            {   
                write_config_in_flash_status=0;							
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the ETSI Duty Cycle setting - 0=disable, 1=enable - Only for testing\r\n");
					break;
				}	
        default: break;
    }

    return ret;
}
	
static int at_pnm_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_PUBLIC_NETWORK;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.EnablePublicNetwork);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
            if(status<2)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status1;
								mib.Type = MIB_PUBLIC_NETWORK;
							  mib.Param.EnablePublicNetwork = status;
								status1=LoRaMacMibSetRequestConfirm(&mib);
								
							  if(status1==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the public network mode. (0: off, 1: on)\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx2fq_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t freq;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RX2_CHANNEL;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.Rx2Channel.Frequency);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            freq = strtol((const char *)argv[0], NULL, 0);
            if(100000000<freq && freq<999000000)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_RX2_CHANNEL;
					      LoRaMacMibGetRequestConfirm(&mib);
							
								mib.Type = MIB_RX2_CHANNEL;
							
								mib.Param.Rx2Channel = ( Rx2ChannelParams_t ){ freq , mib.Param.Rx2Channel.Datarate };
					
								status=LoRaMacMibSetRequestConfirm(&mib);
								
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Rx2 window frequency\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx2dr_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t dr;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RX2_CHANNEL;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.Rx2Channel.Datarate);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            dr = strtol((const char *)argv[0], NULL, 0);
            if(dr<16)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
						 	  mib.Type = MIB_RX2_CHANNEL;
					      LoRaMacMibGetRequestConfirm(&mib);
							
								mib.Type = MIB_RX2_CHANNEL;
							  mib.Param.Rx2Channel = ( Rx2ChannelParams_t ){ mib.Param.Rx2Channel.Frequency , dr };
								status=LoRaMacMibSetRequestConfirm(&mib);
								RX2DR_setting_status=1;
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Rx2 window data rate (0-7 corresponding to DR_X)\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx1dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RECEIVE_DELAY_1;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.ReceiveDelay1);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_RECEIVE_DELAY_1;
							  mib.Param.ReceiveDelay1 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
                atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the delay between the end of the Tx and the Rx Window 1 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rx2dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_RECEIVE_DELAY_2;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.ReceiveDelay2);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_RECEIVE_DELAY_2;
							  mib.Param.ReceiveDelay2 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the delay between the end of the Tx and the Rx Window 2 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_jn1dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						
      
						mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.JoinAcceptDelay1);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
							  mib.Param.JoinAcceptDelay1 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Join Accept Delay between the end of the Tx and the Join Rx Window 1 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_jn2dl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t delay;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.JoinAcceptDelay2);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            delay = strtol((const char *)argv[0], NULL, 0);
            if(delay>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
							  mib.Param.JoinAcceptDelay2 = delay;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Join Accept Delay between the end of the Tx and the Join Rx Window 2 in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_njm_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
            if (lora_config_otaa_get() == LORA_ENABLE)											
					  {
							status=1;
						}
						else
						{
							status=0;							
						}	
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", status);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
					  ret = LWAN_SUCCESS;
					  write_config_in_flash_status=1;
					  atcmd[0] = '\0';
					
            if(status==0)
            {
                lora_config_otaa_set(LORA_DISABLE);								                
            }
						else if(status==1)
						{
							  lora_config_otaa_set(LORA_ENABLE);
						}
            else
            {            
                write_config_in_flash_status=0;							
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Network Join Mode. (0: ABP, 1: OTAA)\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_nwkid_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	  uint8_t length;
    uint8_t buf[4];
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_NET_ID;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%02X %02X %02X %02X\r\n", (uint8_t)(mib.Param.NetID>>24&0xff),(uint8_t)(mib.Param.NetID>>16&0xff),(uint8_t)(mib.Param.NetID>>8&0xff),(uint8_t)(mib.Param.NetID&0xff));

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            length = hex2bin((const char *)argv[0], buf, 4);
            if(length==4)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
								mib.Type = MIB_NET_ID;
							  mib.Param.NetID = buf[0] << 24 | buf[1] << 16 | buf[2] <<8 | buf[3];
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Network ID\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_fcu_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t counter;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_UPLINK_COUNTER;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.UpLinkCounter);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            counter = strtol((const char *)argv[0], NULL, 0);
            if(counter>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_UPLINK_COUNTER;
							  mib.Param.UpLinkCounter = counter;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
									if(join_mothod==0)
									{
										save_devNonce_buff();
										HAL_Delay(50);
									}
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Frame Counter Uplink\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_fcd_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t counter;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_DOWNLINK_COUNTER;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)mib.Param.DownLinkCounter);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            counter = strtol((const char *)argv[0], NULL, 0);
            if(counter>0)
            {
								MibRequestConfirm_t mib;								
							  LoRaMacStatus_t status;
							
								mib.Type = MIB_DOWNLINK_COUNTER;
							  mib.Param.DownLinkCounter = counter;
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
            }
            else
            {                
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Frame Counter Downlink\r\n");
					break;
				}
        default: break;
    }

    return ret;
}
	
static int at_class_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_DEVICE_CLASS;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%c\r\n", 'A' + mib.Param.Class);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;            
					
					  MibRequestConfirm_t mib;						
            LoRaMacStatus_t status;
					
						mib.Type = MIB_DEVICE_CLASS;
					
            switch (*argv[0])
						{
							case 'A':
							case 'B':
							case 'C':
								/* assume CLASS_A == 0, CLASS_B == 1, etc, which is the case for now */
								mib.Param.Class = (DeviceClass_t)(*argv[0] - 'A');
								status=LoRaMacMibSetRequestConfirm(&mib);
							
							  if(status==LORAMAC_STATUS_OK)
								{
									ret = LWAN_SUCCESS;
									write_config_in_flash_status=1;
								}
								else
									ret = LWAN_BUSY_ERROR;
								
							  atcmd[0] = '\0';
								break;
							default:
								return LWAN_PARAM_ERROR;
						} 

            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the Device Class\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_join_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS; 
          atcmd[0] = '\0';					
          LORA_Join();
          break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Join network\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_njs_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;
					 
					  MibRequestConfirm_t mib;						

						mib.Type = MIB_NETWORK_JOINED;
					  LoRaMacMibGetRequestConfirm(&mib);											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", mib.Param.IsNetworkJoined);
            break;
        }
				
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the join status\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_recv_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS; 
						if(atrecve_flag==0)
						{							
							MW_LOG(TS_OFF, VLEVEL_M, "%d:",ReceivedDataPort);
 				
						  if (ReceivedDataSize)
							{
								MW_LOG(TS_OFF, VLEVEL_M, "%s", ReceivedData);
								ReceivedDataSize = 0;
							}							
						}		
					  snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Print last received data in raw format\r\n");
					break;
				}
								
        default: break;
    }

    return ret;		
}

static int at_recvb_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
        case QUERY_CMD: {
            ret = LWAN_SUCCESS; 
						if(atrecve_flag==0)
						{							
							MW_LOG(TS_OFF, VLEVEL_M, "%d:",ReceivedDataPort);
 				
							for (uint8_t i = 0; i < ReceivedDataSize; i++)
							{
								MW_LOG(TS_OFF, VLEVEL_M, "%02x", ReceivedData[i]);
							}
							ReceivedDataSize = 0;
						}		
					  snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");										
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Print last received data in binary format (with hexadecimal values)\r\n");
					break;
				}
								
        default: break;
    }

    return ret;		
}

static int at_ver_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;					 											

            snprintf((char *)atcmd, ATCMD_SIZE, "%s %s\r\n",band_string,AT_VERSION_STRING);
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get current image version and Frequency Band\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_cfm_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode=0,trials=7,increment_switch=0;
	 
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d,%d\r\n", lora_config_reqack_get(),confirmed_uplink_retransmission_nbtrials,confirmed_uplink_counter_retransmission_increment_switch);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
					
            ret = LWAN_SUCCESS;
					  atcmd[0] = '\0';
					
					  if(argc==1)
						{
							mode = strtol((const char *)argv[0], NULL, 0);
							trials=7;
			        increment_switch=0;
							write_config_in_flash_status=1;
						}
						else if(argc==3)
						{
							mode = strtol((const char *)argv[0], NULL, 0);
							trials=strtol((const char *)argv[1], NULL, 0);
			        increment_switch=strtol((const char *)argv[2], NULL, 0);
							write_config_in_flash_status=1;
						}											  
						
						if(mode>1 || trials>7 || increment_switch>1)
						{
							write_config_in_flash_status=0;
							return LWAN_PARAM_ERROR;
						}
						
						if(mode==0)
						{
							lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
							confirmed_uplink_retransmission_nbtrials=trials;
							confirmed_uplink_counter_retransmission_increment_switch=0;
						}
						else if(mode==1)
						{
							lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
							confirmed_uplink_retransmission_nbtrials=trials;
							confirmed_uplink_counter_retransmission_increment_switch=increment_switch;
						}
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the confirmation mode (0-1)\r\n");
					break;
				}
        default: break;
    }
    
    return ret;
}

static int at_snr_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;					 											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", lora_config_snr_get());
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the SNR of the last received packet\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_rssi_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case QUERY_CMD:
        {
            ret = LWAN_SUCCESS;					 											
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", lora_config_rssi_get());
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the RSSI of the last received packet\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_tdc_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t time=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)APP_TX_DUTYCYCLE);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            time = strtol((const char *)argv[0], NULL, 0);
            if(time>=5000)
            {
                APP_TX_DUTYCYCLE=time;
							  
                ret = LWAN_SUCCESS;
							  write_config_in_flash_status=1;
							  atcmd[0] = '\0';
            }
            else
            {
                MW_LOG(TS_OFF, VLEVEL_M, "TDC setting needs to be high than 4000ms\n\r");
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the application data transmission interval in ms\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_port_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t port=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", lora_config_application_port_get());

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            port = strtol((const char *)argv[0], NULL, 0);
            if(port<=255)
            {
                lora_config_application_port_set(port);
							  
                ret = LWAN_SUCCESS;
							  write_config_in_flash_status=1;
							  atcmd[0] = '\0';
            }
            else
            {              
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the application port\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_pword_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            switch(password_len)
						{	
							case 2:
							{
								snprintf((char *)atcmd, ATCMD_SIZE, "%02X%02X\r\n",password_get[0],password_get[1]);
                break;								
							}		
							case 3:
							{
								snprintf((char *)atcmd, ATCMD_SIZE, "%02X%02X%02X\r\n",password_get[0],password_get[1],password_get[2]);
                break;								
							}	
							case 4:
							{
								snprintf((char *)atcmd, ATCMD_SIZE, "%02X%02X%02X%02X\r\n",password_get[0],password_get[1],password_get[2],password_get[3]);
                break;								
							}	
							case 5:
							{
								snprintf((char *)atcmd, ATCMD_SIZE, "%02X%02X%02X%02X%02X\r\n",
									       password_get[0],password_get[1],password_get[2],password_get[3],password_get[4]);
                break;								
							}	
							case 6:
							{
								snprintf((char *)atcmd, ATCMD_SIZE, "%02X%02X%02X%02X%02X%02X\r\n",
									       password_get[0],password_get[1],password_get[2],password_get[3],password_get[4],password_get[5]);
                break;								
							}		
							case 7:
							{
								snprintf((char *)atcmd, ATCMD_SIZE, "%02X%02X%02X%02X%02X%02X%2X\r\n",
									       password_get[0],password_get[1],password_get[2],password_get[3],password_get[4],password_get[5],password_get[6]);
                break;								
							}								
							case 8:
							{
								snprintf((char *)atcmd, ATCMD_SIZE, "%02X%02X%02X%02X%02X%02X%02X%02X\r\n", 
												password_get[0],password_get[1],password_get[2],password_get[3],password_get[4],password_get[5],password_get[6],password_get[7]);
                break;								
							}	
							default:
							break;							
						}
					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
					  uint8_t length;
					  uint8_t buf[10];
					
						length = hex2bin((const char *)argv[0], buf, 8);    
					
						if((length >= 2)&&(length <= 8))
					  {
							for(uint8_t i=0;i<8;i++)
							{
								password_get[i]=0x00;
							}
							
							password_len= length;
							
							for(uint8_t j=0;j<length;j++)
							{
								password_get[j]=buf[j];
							}
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Set password,2 to 8 bytes\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_chs_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint32_t freq=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)customize_freq1_get());

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            freq = strtol((const char *)argv[0], NULL, 0);
            
					  if((100000000<freq && freq<999999999) || freq==0)
						{
							customize_freq1_set(freq);
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Frequency (Unit: Hz) for Single Channel Mode\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
static int at_che_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;

		uint8_t fre;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;					 
					 
            snprintf((char *)atcmd, ATCMD_SIZE, "%u\r\n", (unsigned int)customize_set8channel_get());

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            fre = strtol((const char *)argv[0], NULL, 0);
            
						#if defined ( REGION_CN470 )
						if((fre>12)||((fre>=1)&&(fre<=10)))
						{
							fre=11;
							snprintf((char *)atcmd, ATCMD_SIZE, "Error Subband, must be 0 or 11,12\r\n");	
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");	
						}
						#elif defined ( REGION_US915 )
						if(fre>8)
						{
							fre=1;
							snprintf((char *)atcmd, ATCMD_SIZE, "Error Subband, must be 0 ~ 8\r\n");
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");	
						}	
						#elif defined ( REGION_AU915 )
						if(fre>8)
						{
							fre=1;
							snprintf((char *)atcmd, ATCMD_SIZE, "Error Subband, must be 0 ~ 8\r\n");		
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");	
						}
						#else
						fre=0;
						#endif
						
						customize_set8channel_set(fre);
							
						ret= LWAN_SUCCESS;
            write_config_in_flash_status=1;
						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set eight channels mode,Only for US915,AU915,CN470\r\n");
					break;
				}
        default: break;
    }

    return ret;
}	
#endif

static int at_pdta_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR;
	  uint16_t start_page=0,end_page=0;
	  uint8_t status;
	
		switch(opt) {
			
			case SET_CMD: {
					if(argc < 1) break;
					
					start_page = strtol((const char *)argv[0], NULL, 0);
					end_page = strtol((const char *)argv[1], NULL, 0);
				
					if(start_page<=end_page && start_page<417 && end_page<417 && start_page>0 && end_page>0)
					{
						is_PDTA_command=1;
						ret = LWAN_SUCCESS;
						atcmd[0] = '\0';
						
						if(sleep_status==1)
						{
							status=read_data_on_flash_buff(start_page,end_page);
						}
						else
						{
							if(LORA_JoinStatus () == LORA_SET)
							{
								if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
								{
									return LWAN_BUSY_ERROR;
								}
								else
								{
									if(work_mode==0)
										MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events when read sensor data\r");
									else
										MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx and CITEMP events when read sensor data\r");
								}
							}
							else
							{
								MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events when read sensor data\r");
							}
							
							TimerStop(&MacStateCheckTimer);
							TimerStop(&TxDelayedTimer);
							TimerStop(&AckTimeoutTimer);

							TimerStop(&RxWindowTimer1);
							TimerStop(&RxWindowTimer2);
							TimerStop(&TxTimer);
							TimerStop(&ReJoinTimer);
							TimerStop( &CalibrationUTCTimer);
							TimerStop( &sht20tempcompTimer);
							TimerStop( &exttempcompTimer);
							status=read_data_on_flash_buff(start_page,end_page);
						}
						
						if(status==1)
						{
							status=0;
							
							if(sleep_status==0)
							{
									TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
									TimerStart(&TxTimer);
								
									if(LORA_JoinStatus () == LORA_SET)
									{				
										if(work_mode==0)
										{
											MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");				    
										}
										else
										{
											MW_LOG(TS_OFF, VLEVEL_M, "Start Tx and CITEMP events\r");
											if((comp_temp1_sht20!=-40)&&(comp_temp2_sht20!=125)&&(work_mode==1)&&(collection_second==0))
											{
												TimerStart( &sht20tempcompTimer);
											}
											TimerStart( &exttempcompTimer);
										}
										
									}
									else
									{
										if(FDR_status==0)
										{
											MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");
											TimerStart(&TxDelayedTimer);
										}
									}
							}
						}
						is_PDTA_command=0;
					}
					else
						ret= LWAN_PARAM_ERROR;
				 
					break;
			}
			default: break;
	}

	return ret;		
}

static int at_pldta_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR;
	  uint32_t address=0;
	  uint16_t num=0;
	  uint8_t status;
	
		switch(opt) {
			
			case SET_CMD: {
					if(argc < 1) break;
					
					num = strtol((const char *)argv[0], NULL, 0);
									
					if(num<=3328)
					{
						is_PDTA_command=1;
						ret = LWAN_SUCCESS;
						atcmd[0] = '\0';
						
						address=write_address;
	
						for(uint16_t i=0;i<num;i++)
						{
							address-=16;
							if(address<=FLASH_SENSOR_DATA_START_ADDR)
							{
								address=FLASH_SENSOR_DATA_END_ADDR;
							}
						}
	
						if(sleep_status==1)
						{
							status=read_data_on_flash_buff_last_sets_data(address,num);
						}
						else
						{
							if(LORA_JoinStatus () == LORA_SET)
							{
								if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
								{
									return LWAN_BUSY_ERROR;
								}
								else
								{
									if(work_mode==0)
										MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events when read sensor data\r");
									else
										MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx and CITEMP events when read sensor data\r");
								}
							}
							else
							{
								MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events when read sensor data\r");
							}
							
							TimerStop(&MacStateCheckTimer);
							TimerStop(&TxDelayedTimer);
							TimerStop(&AckTimeoutTimer);

							TimerStop(&RxWindowTimer1);
							TimerStop(&RxWindowTimer2);
							TimerStop(&TxTimer);
							TimerStop(&ReJoinTimer);
							TimerStop( &CalibrationUTCTimer);
							TimerStop( &sht20tempcompTimer);
							TimerStop( &exttempcompTimer);
							status=read_data_on_flash_buff_last_sets_data(address,num);
						}
						
						if(status==1)
						{
							status=0;
							
							if(sleep_status==0)
							{
									TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
									TimerStart(&TxTimer);
								
									if(LORA_JoinStatus () == LORA_SET)
									{				
										if(work_mode==0)
										{
											MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");				    
										}
										else
										{
											MW_LOG(TS_OFF, VLEVEL_M, "Start Tx and CITEMP events\r");
											if((comp_temp1_sht20!=-40)&&(comp_temp2_sht20!=125)&&(work_mode==1)&&(collection_second==0))
											{
												TimerStart( &sht20tempcompTimer);
											}
											TimerStart( &exttempcompTimer);
										}										
									}
									else
									{
										if(FDR_status==0)
										{
											MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");
											TimerStart(&TxDelayedTimer);
										}
									}
							}
						}
						is_PDTA_command=0;
					}
					else
						ret= LWAN_PARAM_ERROR;
				 
					break;
			}
			default: break;
	}

	return ret;		
}

static int at_clrdta_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {		
				 
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS; 
					atcmd[0] = '\0';
					
					if(sleep_status==0)
					{
						if(LORA_JoinStatus () == LORA_SET)
						{
							if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
							{
								return LWAN_BUSY_ERROR;
							}
							else
							{
								if(work_mode==0)
									MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events,Please wait for the erase to complete\r");
								else
									MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx and CITEMP events,Please wait for the erase to complete\r");
							}
						}
						else
						{
							MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events,Please wait for the erase to complete\r");
						}
					}
					
					MW_LOG(TS_OFF, VLEVEL_M, "Clear all stored sensor data...\r");
					
					TimerStop(&MacStateCheckTimer);
					TimerStop(&TxDelayedTimer);
					TimerStop(&AckTimeoutTimer);

					TimerStop(&RxWindowTimer1);
					TimerStop(&RxWindowTimer2);
					TimerStop(&TxTimer);
					TimerStop(&ReJoinTimer);
					TimerStop( &CalibrationUTCTimer);
					TimerStop( &sht20tempcompTimer);
					TimerStop( &exttempcompTimer);
					
					FLASH_erase_all_sensor_data_storage(FLASH_SENSOR_DATA_START_ADDR);
					write_address=FLASH_SENSOR_DATA_START_ADDR;
					HAL_Delay(1000);
					
						if(sleep_status==0)
						{
								TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
								TimerStart(&TxTimer);
							
								if(LORA_JoinStatus () == LORA_SET)
								{
									if(work_mode==0)
									{
										MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");
									}
									else
									{
										MW_LOG(TS_OFF, VLEVEL_M, "Start Tx and CITEMP events\r");
										if((comp_temp1_sht20!=-40)&&(comp_temp2_sht20!=125)&&(work_mode==1)&&(collection_second==0))
										{
										  TimerStart( &sht20tempcompTimer);
										}
										TimerStart( &exttempcompTimer);
									}
								}
								else
								{
									if(FDR_status==0)
									{
										MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");
										TimerStart(&TxDelayedTimer);
									}
								}
							
						}
					
          break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Clear the storage, record position back to 1st\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_sleep_func(int opt, int argc, char *argv[])
{
	  int ret = LWAN_PARAM_ERROR; 
    switch(opt) {		
		 case QUERY_CMD: {
				ret = LWAN_SUCCESS;					 
			 
				snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", sleep_status);

			 break;
		}
				 
    case EXECUTE_CMD:
        {
          ret = LWAN_SUCCESS; 
					
					*((uint8_t *)(0x2000F00A))=0xA8;
					
					TimerStop(&MacStateCheckTimer);
					TimerStop(&TxDelayedTimer);
					TimerStop(&AckTimeoutTimer);

					TimerStop(&RxWindowTimer1);
					TimerStop(&RxWindowTimer2);
					TimerStop(&TxTimer);
					TimerStop(&ReJoinTimer);
					TimerStop( &CalibrationUTCTimer);
					TimerStop( &sht20tempcompTimer);
					TimerStop( &exttempcompTimer);
					TimerStop( &DS18B20IDTimer );
					TimerStop( &DownlinkDetectTimeoutTimer);
					TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
					TimerStop(&RedLEDAlarmTimer1);
					
					if((Ext==4)||(Ext==8)||(Ext==0x0e))
					{
////////						gpio_init(GPIO_Exit_PORT, GPIO_Exit_PIN, GPIO_MODE_ANALOG);
////////						gpio_config_interrupt(GPIO_Exit_PORT, GPIO_Exit_PIN , GPIO_INTR_NONE);
////////						gpio_config_stop3_wakeup(GPIO_Exit_PORT, GPIO_Exit_PIN ,false,GPIO_LEVEL_HIGH);	
////////						Power_contrl(0);
					}							
					sleep_status=1;
					
					snprintf((char *)atcmd, ATCMD_SIZE, "SLEEP\r\n");
					
          break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Set sleep mode\r\n");
					break;
				}
     default: break;
    }
    
    return ret;
}

static int at_ext_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode=1;
    uint16_t value_temp=0;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
						if(Ext==4)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", Ext,exitintmode);
            }
						else if(Ext==6)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", Ext,power_time);
            }			
						else if(Ext==8)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", Ext,exitmode);
            }
						else if((Ext==11)&&(s31f_ext==1))
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", Ext,s31f_ext);
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", Ext);
						}
						break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
						if(argc==2)
						{
							value_temp = strtol((const char *)argv[1], NULL, 0);
					  }
						
					  if((mode<12)||(mode==0x0e))
						{
							Ext=mode;
//////////							if((Ext==4)&&(value_temp<=3))
//////////							{
//////////								exitintmode = value_temp;
//////////								Power_contrl(1);
//////////							}
//////////							else if(Ext==6)
//////////							{
//////////								power_time = value_temp;
//////////								Power_contrl(0);
//////////							}
//////////              else if((Ext==8)&&(value_temp<=1))
//////////							{
//////////								exitmode = value_temp;
//////////								Power_contrl(1);
//////////							}
//////////              else if(Ext==0x0e)
//////////							{	
//////////								Power_contrl(1);
//////////							}
//////////							else if((Ext==11)&&(value_temp<=1))
//////////							{
//////////								s31f_ext=value_temp;
//////////								Power_contrl(0);
//////////							}
//////////							else
//////////							{
//////////								Power_contrl(0);
//////////							}
//////////							Interrupt_mode(Ext);
//////////							ret = LWAN_SUCCESS;
//////////							write_config_in_flash_status=1;
//////////							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set external sensor model\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_bat_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint16_t bat;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					  bat=battery_voltage_measurement();
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", bat);

					 break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get the current battery voltage in mV\r\n");
					break;
				}               
        default: break;
    }

    return ret;
}

static int at_cfg_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR; 
    switch(opt) {
    case EXECUTE_CMD:		
		      {
          ret = LWAN_SUCCESS; 
					atcmd[0] = '\0';
					
					if(sleep_status==0)
					{
						if(LORA_JoinStatus () == LORA_SET)
						{
							if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
							{
								return LWAN_BUSY_ERROR;
							}
							else
							{
								if(work_mode==0)
									MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events,Please wait for the erase to complete\r");
								else
									MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx and CITEMP events,Please wait for the erase to complete\r");
							}
						}
						else
						{
							MW_LOG(TS_OFF, VLEVEL_M, "Stop Tx events,Please wait for the erase to complete\r");
						}
					}
					
//					MW_LOG(TS_OFF, VLEVEL_M, "Clear all stored sensor data...\r");
					
					TimerStop(&MacStateCheckTimer);
					TimerStop(&TxDelayedTimer);
					TimerStop(&AckTimeoutTimer);

					TimerStop(&RxWindowTimer1);
					TimerStop(&RxWindowTimer2);
					TimerStop(&TxTimer);
					TimerStop(&ReJoinTimer);
					TimerStop( &CalibrationUTCTimer);
					TimerStop( &sht20tempcompTimer);
					TimerStop( &exttempcompTimer);
					
					atrecve_flag=1;						
					for (uint8_t num = 0; num < AT_TABLE_SIZE; num++)
					{							
						if(g_at_table[num].fn(QUERY_CMD, 0, 0)==LWAN_SUCCESS)
						{
							MW_LOG(TS_OFF, VLEVEL_M, "AT%s=",g_at_table[num].cmd);
							if(strcmp(g_at_table[num].cmd,AT_RECVB)==0)
							{
								atrecve_flag=0;
								g_at_table[num].fn(QUERY_CMD, 0, 0);
								atrecve_flag=1;
							}
              MW_LOG(TS_OFF, VLEVEL_M, "%s",atcmd);							
						}
						atcmd_index = 0;
						memset(atcmd, 0xff, ATCMD_SIZE);
					}    
					atrecve_flag=0;
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");
						
					HAL_Delay(1000);
					
						if(sleep_status==0)
						{
								TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
								TimerStart(&TxTimer);
								if(LORA_JoinStatus () == LORA_SET)
								{
									if(work_mode==0)
									{
										MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");
									}
									else
									{
										MW_LOG(TS_OFF, VLEVEL_M, "Start Tx and CITEMP events\r");
										if((comp_temp1_sht20!=-40)&&(comp_temp2_sht20!=125)&&(work_mode==1)&&(collection_second==0))
										{
										  TimerStart( &sht20tempcompTimer);
										}
										TimerStart( &exttempcompTimer);
									}
								}
								else
								{
									if(FDR_status==0)
									{
										MW_LOG(TS_OFF, VLEVEL_M, "Start Tx events\r");
										TimerStart(&TxDelayedTimer);
									}
								}
							
						}
					
          break;
        }							
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Print all configurations\r\n");
					break;
				}
     default: break;
    }
    
    return ret;    
}

static int at_wmod_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode=0;
    uint16_t time=0,value_temp=0;
    short value1=0,value2=0;
	
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
						if(work_mode==0)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", work_mode);
            }
						else if(work_mode==1)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d,%d,%d\r\n", work_mode,collection_second,comp_temp1_exttemp,comp_temp2_exttemp);
            }
						else if(work_mode==2)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d,%d\r\n", work_mode,collection_second,temp_change);
            }
						else if(work_mode==3)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d,%d,%d,%d,%d,%d\r\n", work_mode,external_sensor_type_for_work_mode3,external_sensor_sample_interval_for_work_mode3,external_sensor_sample_sum_for_work_mode3,comp_temp1_exttemp,comp_temp2_exttemp,temp_alarm_switch_for_work_mode3);
            }						
					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;

						mode = strtol((const char *)argv[0], NULL, 0);		
					
						if((argc>=3) && mode<3)
						{
							time = strtol((const char *)argv[1], NULL, 0); 
							value1 = strtol((const char *)argv[2], NULL, 10);
							if(argc==4)
							{
								value2 = strtol((const char *)argv[3], NULL, 10);
							}
						}
						
						if(time>255 && mode<3)
						{
							time=60;					
						}
						
					  if(mode<=2)
						{
							work_mode=mode;
							if(work_mode==0)
							{
								collection_second=0; 
								comp_temp1_exttemp=0;
								comp_temp2_exttemp=0;	
								temp_change=0;								
								TimerStop( &sht20tempcompTimer);
								TimerStop( &exttempcompTimer);
							}
							else if((work_mode==1)&&(argc==1))
							{
								collection_second=0; 
								comp_temp1_exttemp=0;
								comp_temp2_exttemp=0;	
								temp_change=0;								
								TimerStop( &sht20tempcompTimer);
								TimerStop( &exttempcompTimer);
							}							
							else if((work_mode==1)&&(argc==4))
							{
								collection_second=(uint8_t)time;
                if(collection_second<60)
								{
									collection_second=60;
								}
	
								comp_temp1_exttemp=value1;
								comp_temp2_exttemp=value2;
                TimerStop( &sht20tempcompTimer);								
								TimerStop( &exttempcompTimer);																	
							}
							else if((work_mode==2)&&(argc==3))
							{
								collection_second=(uint8_t)time; 
								if(collection_second<60)
								{
									collection_second=60;
								}
								
								temp_change=(uint8_t)value1;
								TimerStop( &sht20tempcompTimer);
								TimerStop( &exttempcompTimer);																	
							}
							ret = LWAN_SUCCESS;	
							write_config_in_flash_status=1;
							snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");
						}
						else if(mode==3)
						{
							work_mode=mode;
							
							if(argc==7)
							{
								value_temp = strtol((const char *)argv[1], NULL, 0);
								
								if(value_temp>4 || value_temp==0)
								{
									value_temp=1;
								}
								external_sensor_type_for_work_mode3=value_temp;							
								
								value_temp = strtol((const char *)argv[2], NULL, 0);
								if(value_temp<60)
								{
									value_temp=60;
								}
								external_sensor_sample_interval_for_work_mode3=value_temp;									
								
								value_temp = strtol((const char *)argv[3], NULL, 0);
								if(value_temp<1)
								{
									value_temp=20;
								}
								
								if(value_temp>60)
								{
									value_temp=60;
								}
								
								external_sensor_sample_sum_for_work_mode3=value_temp;	
								
								value1 = strtol((const char *)argv[4], NULL, 10);
								value2 = strtol((const char *)argv[5], NULL, 10);
								temp_alarm_switch_for_work_mode3 = strtol((const char *)argv[6], NULL, 0);
								
								comp_temp1_exttemp=value1;
								comp_temp2_exttemp=value2;
								
								ret = LWAN_SUCCESS;
								write_config_in_flash_status=1;
							  snprintf((char *)atcmd, ATCMD_SIZE, "Attention:Take effect after ATZ\r\n");
								
								TimerStop( &TxTimer);
								TimerStop( &sht20tempcompTimer);
								TimerStop( &exttempcompTimer);
							}
						}
						else
						{
							ret= LWAN_PARAM_ERROR;
            }
						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Work Mode\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_artemp_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", comp_temp1_sht20,comp_temp2_sht20);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            short  value1 = strtol((const char *)argv[0], NULL, 10);
            short  value2= strtol((const char *)argv[1], NULL, 10);
            
            if(value1>value2||value1<-40||value2>125)
            {
                ret = LWAN_PARAM_ERROR;
            }
            else
            {               
								comp_temp1_sht20=value1;
								comp_temp2_sht20=value2;
                ret = LWAN_SUCCESS;   
                write_config_in_flash_status=1;							
                atcmd[0] = '\0';
            }			
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the internal Temperature sensor alarm range\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_citemp_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint16_t interval;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", collection_interval);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            interval = strtol((const char *)argv[0], NULL, 0);
            
					  if(interval<=65535)
						{
							collection_interval=interval;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the internal Temperature sensor collection interval in min\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_setcnt_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {       
        case SET_CMD: {
            if(argc < 1) break;
					
            count = strtol((const char *)argv[0], NULL, 0);				
						ret = LWAN_SUCCESS;
						atcmd[0] = '\0';           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Set count value\r\n");
					break;
				}
        default: break;
    }

    return ret;	
}

static int at_intmod1_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	  uint16_t dlay=0;
    uint8_t value=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", inmode,inmode_delay);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            value = strtol((const char *)argv[0], NULL, 0);
					  if(argc==2)
						{
							dlay= strtol((const char *)argv[1], NULL, 0);
						}
						
						if (value<=4)
            {
								inmode=value;
							  inmode_delay=dlay;
							  GPIO_EXTI8_IoInit(inmode);
                ret = LWAN_SUCCESS;
							  write_config_in_flash_status=1;
							  atcmd[0] = '\0';
            }
            else
            {
                ret = LWAN_PARAM_ERROR;
            }
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set the trigger interrupt mode of PA8(0:Disable,1:falling or rising,2:falling,3:rising)\r\n");
					break;
				}
        default: break;
    }

    return ret;		
}

#if defined ( REGION_AU915 ) || defined ( REGION_AS923 )
static int at_dwellt_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", dwelltime);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
            
					  if(status<2)
						{
							dwelltime=status;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set UplinkDwellTime\r\n");
					break;
				}
        default: break;
    }

    return ret;
}
#endif

static int at_rjtdc_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint16_t interval;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", REJOIN_TX_DUTYCYCLE);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            interval = strtol((const char *)argv[0], NULL, 0);
            
					  if(interval>0 && interval<=65535)
						{
							REJOIN_TX_DUTYCYCLE=interval;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the ReJoin data transmission interval in min\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_rpl_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t level;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", response_level);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            level = strtol((const char *)argv[0], NULL, 0);
            
					  if(level<=4)
						{
							response_level=level;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set response level\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_timestamp_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
						struct tm localtime;
						SysTime_t sysTimeCurrent = { 0 };
						
						sysTimeCurrent=SysTimeGet();	
						SysTimeLocalTime( sysTimeCurrent.Seconds, &localtime );                     
            
            snprintf((char *)atcmd, ATCMD_SIZE, "systime= %d/%d/%d %02d:%02d:%02d (%u)\n\r",localtime.tm_year+1900,localtime.tm_mon+1,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,(unsigned int)sysTimeCurrent.Seconds);
            break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            uint32_t timestamp;
            SysTime_t sysTime = { 0 };
            
            timestamp=strtoul((const char *)argv[0], NULL, 0);
            
            if (timestamp > 1640044800) {              
							sysTime.Seconds=timestamp;
		          SysTimeSet( sysTime );	
              ret = LWAN_SUCCESS;   
              atcmd[0] = '\0';							
            }
						else
							ret = LWAN_PARAM_ERROR;
            
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set UNIX timestamp in second\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_leapsec_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t second;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", currentLeapSecond);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            second = strtol((const char *)argv[0], NULL, 0);
            
					  if(second<=255)
						{
							currentLeapSecond=second;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set Leap Second\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_syncmod_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", time_synchronization_method);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
            
					  if(mode<2)
						{
							time_synchronization_method=mode;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or Set time synchronization method\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_synctdc_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t interval;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", time_synchronization_interval);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            interval = strtol((const char *)argv[0], NULL, 0);
            
					  if(interval>0 && interval<=255)
						{
							time_synchronization_interval=interval;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set time synchronization interval in day\r\n");
					break;
				}
        default: break;
    }

    return ret;
}
	
static int at_pid_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", pid_flag);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
            
					  if(mode<2)
						{
							pid_flag=mode;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the PID\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_uoa_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t value1=0;
	  uint16_t value2=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
					  if(valid_flag==0)
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", valid_flag);
						}
						else
						{
							snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", valid_flag,valid_time);
						}

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            value1 = strtol((const char *)argv[0], NULL, 0);
					  if(argc==2)
						{
							value2 = strtol((const char *)argv[1], NULL, 0);
						}
						
					  if((value1<=1)&&(value2<=65535))
						{
							if(value1==0)
							{
							 valid_flag=0;
							 valid_time=0;
							}
							else
							{
								valid_flag=value1;
								valid_time=value2;	
                exit_send_flag=0;								
							}								
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the active duration of the PIR sensor\r\n");
					break;
				}
        default: break;
    }

    return ret;	
}

static int at_downlink_detect_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	  uint8_t status;
	  uint16_t timeout1=0,timeout2=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d,%d\r\n", downlink_detect_switch,unconfirmed_uplink_change_to_confirmed_uplink_timeout,downlink_detect_timeout);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
            timeout1 = strtol((const char *)argv[1], NULL, 0);
					  timeout2 = strtol((const char *)argv[2], NULL, 0);
					
					  if(status<2)
						{
							downlink_detect_switch=status;
							
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							return LWAN_PARAM_ERROR;

						if(timeout1>0 && timeout1<=65535 && timeout2>0 && timeout2<=65535)
						{
							unconfirmed_uplink_change_to_confirmed_uplink_timeout=timeout1;
							downlink_detect_timeout=timeout2;
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
						{	
              write_config_in_flash_status=0;							
							return LWAN_PARAM_ERROR;	
						}
	
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the downlink detection\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_setmaxnbtrans_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;

	  uint8_t trials,increment_switch;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d,%d\r\n", LinkADR_NbTrans_retransmission_nbtrials,LinkADR_NbTrans_uplink_counter_retransmission_increment_switch);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            trials = strtol((const char *)argv[0], NULL, 0);
            increment_switch = strtol((const char *)argv[1], NULL, 0);				  
					
						if(trials<1 || trials>15 || increment_switch>1)
						{
							return LWAN_PARAM_ERROR;
						}

						LinkADR_NbTrans_retransmission_nbtrials=trials;
						LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=increment_switch;

						ret = LWAN_SUCCESS;
						write_config_in_flash_status=1;
						atcmd[0] = '\0';

            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the max nbtrans in LinkADR\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_uuid_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    
    uint8_t buf[8];
    
    switch(opt) {
                 
        case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            
            uint32_t unique_id[2];            
						unique_id[0] = HAL_GetUIDw0();
						unique_id[1] = HAL_GetUIDw1();
					
            snprintf((char *)atcmd, ATCMD_SIZE, "%08X%08X\r\n", (unsigned int)unique_id[0],(unsigned int)unique_id[1]);
            
					  break;
        }
                
        case SET_CMD: {
            if(argc < 1) break;
            
            uint8_t length = hex2bin((const char *)argv[0], buf, 8);
            if (length == 8) {                                    
                    uuid1_in_sflash=buf[0]<<24|buf[1]<<16|buf[2]<<8|buf[3];
                    uuid2_in_sflash=buf[4]<<24|buf[5]<<16|buf[6]<<8|buf[7];
                    
                    atcmd[0] = '\0';
                    ret = LWAN_SUCCESS; 
                    write_key_in_flash_status=1;							
            }
            else if(length==6)
            {
                if(buf[0]==0x66&&buf[1]==0x66&&buf[2]==0x66&&buf[3]==0x66&&buf[4]==0x66&&buf[5]==0x66)
                {
                    uint32_t id[2];
										id[0] = HAL_GetUIDw0();
										id[1] = HAL_GetUIDw1();
                    uuid1_in_sflash=id[0]^id[1];
                    uuid2_in_sflash=uuid1_in_sflash&id[1];
                    atcmd[0] = '\0';
                    ret = LWAN_SUCCESS;
									  write_key_in_flash_status=1;
                }
            }
						else 
              ret = LWAN_PARAM_ERROR;
						
            break;
        }
				
				case DESC_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "Get the device Unique ID\r\n");
            break;
        }
								
        default: break;
    }

    return ret;
}

static int at_rxdatatest_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
	  uint8_t length;
    uint8_t buff[20];
        
    switch(opt) {
			
        case SET_CMD: {
            if(argc < 1) break;
            
						length=rxdata_change_hex((const char *)argv[0], buff);
					
			   		lora_AppData_t rxappData;
					  rxappData.Port = 2;
            rxappData.BuffSize = length;
						rxappData.Buff=buff;
            LoRaMainCallbacks->LORA_RxData( &rxappData );
						ret = LWAN_SUCCESS;
						snprintf((char *)atcmd, ATCMD_SIZE, "\r\n");
            break;
        }
				
        default: break;
    }
    
    return ret;	
}

static int at_pnackmd_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", pnackmd_switch);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
            
					  if(mode<2)
						{
							pnackmd_switch=mode;
							
							if(pnackmd_switch==1)
							{
								lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
								confirmed_uplink_retransmission_nbtrials=0;
								confirmed_uplink_counter_retransmission_increment_switch=0;
							}
				
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Mode for sending data for which acknowledgment was not received\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_getsensorvalue_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t status=0;
		
    switch(opt) {
        
        case SET_CMD: {
            if(argc < 1) break;
            
            status = strtol((const char *)argv[0], NULL, 0);
            
					  if(status<2)
						{
              if(status==0)
							{		
								if(( LORA_JoinStatus () == LORA_SET)&&(( LoRaMacState & 0x00000001 ) == 0x00000001))
								{
									return LWAN_BUSY_ERROR;
								}									
								sensor_t bsp_sensor_data_buff;								
								
								atcmd[0] = '\0';
													
								BSP_sensor_Read( &bsp_sensor_data_buff,1);
							
								ret = LWAN_SUCCESS;
							}
							else if(status==1)
							{
								if ( LORA_JoinStatus () != LORA_SET)
								{
									/*Not joined, try again later*/
									return LWAN_NO_NET_JOINED;
								}
							
								message_flags=1;
								is_time_to_send=1;
								atcmd[0] = '\0';
								ret = LWAN_SUCCESS;								
							}
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get current sensor value\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_extplogic_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", extpower_logic);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
            
					  if(mode<2)
						{
							extpower_logic=mode;							
				
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set ext power logic\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_ledalarm_func(int opt, int argc, char *argv[])
{
    int ret = LWAN_PARAM_ERROR;
    uint8_t mode;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", led_alarm_switch);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            mode = strtol((const char *)argv[0], NULL, 0);
            
					  if(mode<2)
						{
							led_alarm_switch=mode;							
				
							ret = LWAN_SUCCESS;
							write_config_in_flash_status=1;
							atcmd[0] = '\0';
						}
						else
							ret= LWAN_PARAM_ERROR;
           
						if(mode==0)
						{
							TimerStop(&RedLEDAlarmTimer1);
						}
						
            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set led alarm\r\n");
					break;
				}
        default: break;
    }

    return ret;
}

static int at_devnonce_func(int opt, int argc, char *argv[])	
{
    int ret = LWAN_PARAM_ERROR;
    uint16_t value=0;
    
    switch(opt) {
         case QUERY_CMD: {
            ret = LWAN_SUCCESS;
            snprintf((char *)atcmd, ATCMD_SIZE, "%d\r\n", LoRaMacDevNonce);

					 break;
        }
        
        case SET_CMD: {
            if(argc < 1) break;
            
            value = strtol((const char *)argv[0], NULL, 0);
						LoRaMacDevNonce=value;
						save_devNonce_buff();
						HAL_Delay(50);
            ret = LWAN_SUCCESS;
						atcmd[0] = '\0';

            break;
        }
								
				case DESC_CMD: {
					ret = LWAN_SUCCESS;
					snprintf((char *)atcmd, ATCMD_SIZE, "Get or set the LoRaMacDevNonce\r\n");
					break;
				}
        default: break;
    }

    return ret;			
}

static char circBuffer[CIRC_BUFF_SIZE];
static char command[CMD_SIZE];
static unsigned i = 0;
static uint32_t widx = 0;
static uint32_t ridx = 0;
static uint32_t charCount = 0;
static uint32_t circBuffOverflow = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Parse a command and process it
  * @param  cmd The command
  */
static void parse_cmd(const char *cmd);

/**
  * @brief  Print a string corresponding to an ATEerror_t
  * @param  error_type The AT error code
  */
static void com_error(ATEerror_t error_type);

/**
  * @brief  CMD_GetChar callback from ADV_TRACE
  * @param  rxChar th char received
  * @param  size
  * @param  error
  */
static void CMD_GetChar(uint8_t *rxChar, uint16_t size, uint8_t error);

/**
  * @brief  CNotifies the upper layer that a character has been received
  */
static void (*NotifyCb)(void);

/**
  * @brief  Remove backspace and its preceding character in the Command string
  * @param  cmd string to process
  * @retval 0 when OK, otherwise error
  */
static int32_t CMD_ProcessBackSpace(char *cmd);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
void CMD_Init(void (*CmdProcessNotify)(void))
{
  /* USER CODE BEGIN CMD_Init_1 */

  /* USER CODE END CMD_Init_1 */
  UTIL_ADV_TRACE_StartRxProcess(CMD_GetChar);
  /* register call back*/
  if (CmdProcessNotify != NULL)
  {
    NotifyCb = CmdProcessNotify;
  }
  widx = 0;
  ridx = 0;
  charCount = 0;
  i = 0;
  circBuffOverflow = 0;
  /* USER CODE BEGIN CMD_Init_2 */

  /* USER CODE END CMD_Init_2 */
}

void CMD_Process(void)
{
  /* USER CODE BEGIN CMD_Process_1 */

  /* USER CODE END CMD_Process_1 */
  /* Process all commands */
  if (circBuffOverflow == 1)
  {
    com_error(AT_TEST_PARAM_OVERFLOW);
    /*Full flush in case of overflow */
    UTILS_ENTER_CRITICAL_SECTION();
    ridx = widx;
    charCount = 0;
    circBuffOverflow = 0;
    UTILS_EXIT_CRITICAL_SECTION();
    i = 0;
  }

  while (charCount != 0)
  {
#if 0 /* echo On    */
    AT_PPRINTF("%c", circBuffer[ridx]);
#endif /* 0 */

    if (circBuffer[ridx] == AT_ERROR_RX_CHAR)
    {
      ridx++;
      if (ridx == CIRC_BUFF_SIZE)
      {
        ridx = 0;
      }
      UTILS_ENTER_CRITICAL_SECTION();
      charCount--;
      UTILS_EXIT_CRITICAL_SECTION();
      com_error(AT_RX_ERROR);
      i = 0;
    }
    else if ((circBuffer[ridx] == '\r') || (circBuffer[ridx] == '\n'))
    {
      ridx++;
      if (ridx == CIRC_BUFF_SIZE)
      {
        ridx = 0;
      }
      UTILS_ENTER_CRITICAL_SECTION();
      charCount--;
      UTILS_EXIT_CRITICAL_SECTION();

      if (i != 0)
      {
        command[i] = '\0';
        UTILS_ENTER_CRITICAL_SECTION();
        CMD_ProcessBackSpace(command);
        UTILS_EXIT_CRITICAL_SECTION();
        parse_cmd(command);
        i = 0;
      }
    }
    else if (i == (CMD_SIZE - 1))
    {
      i = 0;
      com_error(AT_TEST_PARAM_OVERFLOW);
    }
    else
    {
      command[i++] = circBuffer[ridx++];
      if (ridx == CIRC_BUFF_SIZE)
      {
        ridx = 0;
      }
      UTILS_ENTER_CRITICAL_SECTION();
      charCount--;
      UTILS_EXIT_CRITICAL_SECTION();
    }
  }
  /* USER CODE BEGIN CMD_Process_2 */

  /* USER CODE END CMD_Process_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
static int32_t CMD_ProcessBackSpace(char *cmd)
{
  /* USER CODE BEGIN CMD_ProcessBackSpace_1 */

  /* USER CODE END CMD_ProcessBackSpace_1 */
  uint32_t i = 0;
  uint32_t bs_cnt = 0;
  uint32_t cmd_len = 0;
  /*get command length and number of backspace*/
  while (cmd[cmd_len] != '\0')
  {
    if (cmd[cmd_len] == '\b')
    {
      bs_cnt++;
    }
    cmd_len++;
  }
  /*for every backspace, remove backspace and its preceding character*/
  for (i = 0; i < bs_cnt; i++)
  {
    int32_t curs = 0;
    int32_t j = 0;

    /*set cursor to backspace*/
    while (cmd[curs] != '\b')
    {
      curs++;
    }
    if (curs > 0)
    {
      for (j = curs - 1; j < cmd_len - 2; j++)
      {
        cmd[j] = cmd[j + 2];
      }
      cmd[j++] = '\0';
      cmd[j++] = '\0';
      cmd_len -= 2;
    }
    else
    {
      return -1;
    }
  }
  return 0;
  /* USER CODE BEGIN CMD_ProcessBackSpace_2 */

  /* USER CODE END CMD_ProcessBackSpace_2 */
}

static void CMD_GetChar(uint8_t *rxChar, uint16_t size, uint8_t error)
{
  /* USER CODE BEGIN CMD_GetChar_1 */

  /* USER CODE END CMD_GetChar_1 */
  charCount++;
  if (charCount == (CIRC_BUFF_SIZE + 1))
  {
    circBuffOverflow = 1;
    charCount--;
  }
  else
  {
    circBuffer[widx++] = *rxChar;
    if (widx == CIRC_BUFF_SIZE)
    {
      widx = 0;
    }
  }

  if (NotifyCb != NULL)
  {
    NotifyCb();
  }
  /* USER CODE BEGIN CMD_GetChar_2 */

  /* USER CODE END CMD_GetChar_2 */
}

static void parse_cmd(const char *cmd)
{
	  uint8_t pwflag=0;
    char *ptr = NULL;
		int argc = 0;
		int index = 0;
		char *argv[ARGC_LIMIT];
		int ret = LWAN_ERROR;
		uint8_t *rxcmd = (uint8_t *)(cmd + 2);
		int16_t rxcmd_index = atcmd_index - 2;

//parse_flag=1;

    if(parse_flag==1)
    {
			g_atcmd_processing = true;
			
			if(cmd[0] != 'A' || cmd[1] != 'T')
					goto at_end;
			
			if((cmd[0] == 'A')&&(cmd[1] == 'T')&&(cmd[2] == '?')&&(cmd[3] == '\0'))  //AT?
			{
        for (uint8_t num = 0; num < AT_TABLE_SIZE; num++)
        {							
          if(g_at_table[num].fn(DESC_CMD, 0, 0)==LWAN_SUCCESS)
          {
				  	MW_LOG(TS_OFF, VLEVEL_M, "AT%s: ",g_at_table[num].cmd);
            MW_LOG(TS_OFF, VLEVEL_M, "%s", atcmd);  
          }
					HAL_Delay(50);
          atcmd_index = 0;
          memset((char *)cmd, 0xff, ATCMD_SIZE);
         }	
				 snprintf((char *)cmd, ATCMD_SIZE, "\r\n");
				 ret=LWAN_SUCCESS;			
			}
			else
			{

			for (index = 0; index < AT_TABLE_SIZE; index++) {
					int cmd_len = strlen(g_at_table[index].cmd);
				if (!strncmp((const char *)(rxcmd), g_at_table[index].cmd, cmd_len)) {
					ptr = (char *)rxcmd + cmd_len;					
					break;
				}
			}

		if (index >= AT_TABLE_SIZE || !g_at_table[index].fn)
					goto at_end;

    if (ptr[0] == '\0') {
			ret = g_at_table[index].fn(EXECUTE_CMD, argc, argv);
		}  else if (ptr[0] == ' ') {
					argv[argc++] = ptr;
			ret = g_at_table[index].fn(EXECUTE_CMD, argc, argv);
		} 
		else if( (ptr[0] == '?') && (ptr[1] == '\0')) {
				ret = g_at_table[index].fn(DESC_CMD, argc, argv);				
		}else if ((ptr[0] == '=') && (ptr[1] == '?') && (ptr[2] == '\0')) {
					ret = g_at_table[index].fn(QUERY_CMD, argc, argv);
		} else if (ptr[0] == '=') {
			ptr += 1;
					
					char *str = strtok((char *)ptr, ",");
					while(str) {
							argv[argc++] = str;
							str = strtok((char *)NULL, ",");
					}

			ret = g_at_table[index].fn(SET_CMD, argc, argv);
					
			if(ret==LWAN_SUCCESS && write_key_in_flash_status==1)
			{
				write_key_in_flash_status=0;				
				Flash_store_key();
			}
			
			if(ret==LWAN_SUCCESS && write_config_in_flash_status==1)
			{
				write_config_in_flash_status=0;				
				Flash_Store_Config();
			}
			
		} else {
			ret = LWAN_ERROR;
		}
	 }
	}
	else
	{
		if(at_PASSWORD_comp((char *) cmd)==1)
		{
			pwflag=1;
			ret = LWAN_SUCCESS;
		}
		else
		{
			pwflag=1;
		}
			
		index = 0;
	}			
	at_end:
   if(pwflag==0)
   {	
			if (LWAN_ERROR == ret)
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", "AT_ERROR");
			else if(LWAN_PARAM_ERROR == ret)
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", "AT_PARAM_ERROR");
			else if(LWAN_BUSY_ERROR == ret) 
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", "AT_BUSY_ERROR");
			else if(LWAN_NO_NET_JOINED == ret) 
					snprintf((char *)atcmd, ATCMD_SIZE, "\r\n%s\r\n", "AT_NO_NET_JOINED");
			

			MW_LOG(TS_OFF, VLEVEL_M, "%s", atcmd);  	
	 
			if(ret==LWAN_SUCCESS)
			{
				MW_LOG(TS_OFF, VLEVEL_M, "\r\nOK\r\n");
			}	
    }   
	 
    atcmd_index = 0;
    memset(atcmd, 0xff, ATCMD_SIZE);
    g_atcmd_processing = false;        
    return;
}

uint8_t password_comp(uint8_t scan_word[],uint8_t set_word[],uint8_t scan_lens,uint8_t set_len)
{
	uint8_t leng=0;
	for(uint8_t i=0;i<scan_lens;i++)
	{
		if(scan_word[i]==set_word[i])
		{
			leng++;
		}
	}
	
	if((leng==set_len)&&(scan_lens==set_len))
	{
		return 1;
	}
	else
	{
		return 0;		
	}
}

static uint8_t at_PASSWORD_comp(char *argv)
{
	uint8_t scan_len=0;	
  uint8_t buf[10];
					
	scan_len=hex2bin((const char *)argv, buf, 8);   

	if(password_comp(password_get,buf,scan_len,password_len)==1)
	{
	  TimerInit( &ATcommandsTimer, OnTimerATcommandsEvent );
    TimerSetValue(  &ATcommandsTimer, 300000);//timeout=5 min
    TimerStart( &ATcommandsTimer );
    parse_flag=1;
    atcmd_index = 0;
    memset(atcmd, 0xff, ATCMD_SIZE);
	  MW_LOG(TS_OFF, VLEVEL_M,"Correct Password\r\n");
		return 1;
	}
	else
  {
    parse_flag=0;
    atcmd_index = 0;
    memset(atcmd, 0xff, ATCMD_SIZE);
		MW_LOG(TS_OFF, VLEVEL_M,"Incorrect Password\n\r");
		return 0;
  }
}

static void OnTimerATcommandsEvent(void *context)
{
  parse_flag=0;
  TimerStop( &ATcommandsTimer );
}

static void com_error(ATEerror_t error_type)
{
  /* USER CODE BEGIN com_error_1 */

  /* USER CODE END com_error_1 */
  if (error_type > AT_MAX)
  {
    error_type = AT_MAX;
  }
  AT_PPRINTF(ATError_description[error_type]);
  /* USER CODE BEGIN com_error_2 */

  /* USER CODE END com_error_2 */
}

/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
