/******************************************************************************
  * @file    lora.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   lora API to drive the lora state Machine
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "LoRaMac.h"
#include "lora_app.h"
#include "lora-test.h"
#include "flash_eraseprogram.h"
#include "version.h"
#include "mw_log_conf.h"
#include "flash_if.h"
#include "utilities.h"

/*
product_id
1:LTC2
2:LHT65
3:LSN50
4:LDDS75
5:LSE01
6:LDDS20
7:RS485-LN
*/
uint8_t password_len;
uint8_t password_get[8];
char *band_string="";
uint8_t product_id=2;
uint8_t current_fre_band=0;//AS923=2
uint8_t current_firmware_ver=10;//ver=1.0 region_info

uint8_t product_id_read_in_flash=0;
uint8_t fre_band_read_in_flash=0;//AS923=1 read in eeprom
uint8_t firmware_ver_read_in_flash=0;//ver=1.3

uint8_t inmode,inmode2,inmode3;
uint16_t inmode_delay,inmode2_delay,inmode3_delay;

uint32_t dl_channel_freq[8];
bool down_check;
uint8_t decrypt_flag=0;
bool join_mothod=1;
extern uint32_t APP_TX_DUTYCYCLE;
extern uint16_t REJOIN_TX_DUTYCYCLE;
extern uint8_t response_level;

extern bool rejoin_status;
extern uint8_t currentLeapSecond;
extern uint8_t time_synchronization_method;
extern uint8_t time_synchronization_interval;

extern bool JoinReq_NbTrails_over;
extern bool unconfirmed_downlink_data_ans_status,confirmed_downlink_data_ans_status;

static void new_firmware_update(void);

extern uint8_t Ext;
bool FDR_status=0;
uint8_t txp_power=0;
extern uint8_t dwelltime;

extern uint8_t s31f_ext;
extern uint8_t valid_flag;
extern uint16_t valid_time;
extern uint8_t work_mode;
extern uint16_t collection_interval;
extern short comp_temp1_sht20,comp_temp2_sht20;
extern short comp_temp1_exttemp,comp_temp2_exttemp;
extern uint8_t temp_alarm_switch_for_work_mode3;
extern uint8_t temp_change;
extern uint8_t collection_second;

extern LoRaMacRegion_t LoRaMacRegion;
extern uint16_t power_time;
extern uint8_t exitintmode;
extern uint8_t exitmode;
extern uint8_t pid_flag;
uint8_t RX2DR_setting_status;
extern uint8_t pnackmd_switch;

extern uint8_t downlink_detect_switch;
extern uint16_t downlink_detect_timeout;

extern uint8_t confirmed_uplink_counter_retransmission_increment_switch;
extern uint8_t confirmed_uplink_retransmission_nbtrials;

extern uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch;
extern uint8_t LinkADR_NbTrans_retransmission_nbtrials;
extern uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout;

uint32_t uuid1_in_sflash,uuid2_in_sflash;

uint8_t device_UUID_error_status=0;

uint8_t lora_packet_send_complete_status=0;

extern uint8_t extpower_logic;

extern bool sleep_status;

extern uint8_t led_alarm_switch;

extern uint8_t external_sensor_type_for_work_mode3;
extern uint16_t external_sensor_sample_interval_for_work_mode3;
extern uint8_t external_sensor_sample_sum_for_work_mode3;

#define HEX16(X)  X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7],X[8],X[9], X[10],X[11], X[12],X[13], X[14],X[15]
#define HEX8(X)   X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7]
 /**
   * Lora Configuration
   */
 typedef struct
 {
   LoraState_t otaa;        /*< ENABLE if over the air activation, DISABLE otherwise */
   LoraState_t duty_cycle;  /*< ENABLE if dutycyle is on, DISABLE otherwise */
   uint8_t DevEui[8];           /*< Device EUI */
	 uint32_t DevAddr;
   uint8_t AppEui[8];           /*< Application EUI */
   uint8_t AppKey[16];          /*< Application Key */
   uint8_t NwkSKey[16];         /*< Network Session Key */
   uint8_t AppSKey[16];         /*< Application Session Key */
   int16_t Rssi;                /*< Rssi of the received packet */
   uint8_t Snr;                 /*< Snr of the received packet */
   uint8_t application_port;    /*< Application port we will receive to */
   LoraConfirm_t ReqAck;      /*< ENABLE if acknowledge is requested */
   McpsConfirm_t *McpsConfirm;  /*< pointer to the confirm structure */
   int8_t TxDatarate;
 } lora_configuration_t;
 
/**
   * Lora customize Configuration
   */
typedef struct 
{
	uint32_t freq1;
	uint8_t  set8channel;
}customize_configuration_t;

static customize_configuration_t customize_config=
{
	.freq1=0,
  .set8channel=0
};

uint32_t customize_freq1_get(void)
{
	return customize_config.freq1;
}

void customize_freq1_set(uint32_t Freq)
{
	customize_config.freq1=Freq;
}

uint32_t customize_set8channel_get(void)
{
	return customize_config.set8channel;
}

void customize_set8channel_set(uint8_t Freq)
{
	customize_config.set8channel=Freq;
}

static lora_configuration_t lora_config = 
{
  .otaa = ((OVER_THE_AIR_ACTIVATION == 0) ? LORA_DISABLE : LORA_ENABLE),
#if defined( REGION_EU868 )
  .duty_cycle = LORA_ENABLE,
#else
  .duty_cycle = LORA_DISABLE,
#endif
	.DevAddr = LORAWAN_DEVICE_ADDRESS,
  .DevEui = LORAWAN_DEVICE_EUI,
  .AppEui = LORAWAN_APPLICATION_EUI,
  .AppKey = LORAWAN_APPLICATION_KEY,
  .NwkSKey = LORAWAN_NWKSKEY,
  .AppSKey = LORAWAN_APPSKEY,
  .Rssi = 0,
  .Snr = 0,
  .ReqAck = LORAWAN_UNCONFIRMED_MSG,
  .McpsConfirm = NULL,
  .TxDatarate = 0
};


/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in ms

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

//#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

//#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 ) 

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

//#endif

#endif

static MlmeReqJoin_t JoinParameters;


//static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

/*
 * Defines the LoRa parameters at Init
 */
static LoRaParam_t* LoRaParamInit;
static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
static MibRequestConfirm_t mibReq;

LoRaMainCallback_t *LoRaMainCallbacks;
/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
	  lora_packet_send_complete_status=1;
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
  lora_AppData_t AppData;
  
	rejoin_status=0;

	unconfirmed_downlink_data_ans_status=0;
	confirmed_downlink_data_ans_status=0;
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    if (certif_running() == true )
    {
      certif_DownLinkIncrement( );
    }

    if( mcpsIndication->RxData == true )
    {
      switch( mcpsIndication->Port )
      {
        case CERTIF_PORT:
          certif_rx( mcpsIndication, &JoinParameters );
          break;
        default:
          
					if(mcpsIndication->McpsIndication==MCPS_UNCONFIRMED && response_level==1)
					{
						unconfirmed_downlink_data_ans_status=1;
					}
					else if(mcpsIndication->McpsIndication==MCPS_CONFIRMED && ((response_level==2) || (response_level==4 )))
					{
						confirmed_downlink_data_ans_status=1;
					}
          AppData.Port = mcpsIndication->Port;
          AppData.BuffSize = mcpsIndication->BufferSize;
          AppData.Buff = mcpsIndication->Buffer;
          lora_config.Rssi = mcpsIndication->Rssi;
          lora_config.Snr  = mcpsIndication->Snr;
          LoRaMainCallbacks->LORA_RxData( &AppData );
          break;
      }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
              LoRaMainCallbacks->LORA_HasJoined();
            }
            else
            {
                JoinReq_NbTrails_over=1;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if (certif_running() == true )
                {
                     certif_linkCheck( mlmeConfirm);
                }
            }
            break;
        }
        default:
            break;
    }
}
/**
 *  lora Init
 */
void LORA_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParam )
{
  /* init the Tx Duty Cycle*/
  LoRaParamInit = LoRaParam;
  
  /* init the main call backs*/
  LoRaMainCallbacks = callbacks;
  
#if (STATIC_DEVICE_EUI != 1)
  LoRaMainCallbacks->BoardGetUniqueId( lora_config.DevEui );  
#endif

#if (STATIC_DEVICE_ADDRESS != 1)
  // Random seed initialization
//  srand1( LoRaMainCallbacks->BoardGetRandomSeed( ) );
//  // Choose a random device address
//  DevAddr = randr( 0, 0x01FFFFFF );
#endif
	
	Flash_read_key();
	Flash_Read_Config();
	
	if(UID_COMP()==0)
	{
		device_UUID_error_status=1;
		MW_LOG(TS_OFF, VLEVEL_M, "Invalid credentials,the device goes into low power mode\r\n");
	}

  #if defined LB_LS
	MW_LOG(TS_OFF, VLEVEL_M, "\n\rDragino SN50v3-LS-LA66S Device\n\r");
	HAL_Delay(20);
	#else	
	MW_LOG(TS_OFF, VLEVEL_M, "\n\rDragino SN50v3-LB-LA66S Device\n\r");
	HAL_Delay(20);
	#endif
	MW_LOG(TS_OFF, VLEVEL_M, "Image Version: "AT_VERSION_STRING"\n\r");
	HAL_Delay(20);
	MW_LOG(TS_OFF, VLEVEL_M, "LoRaWan Stack: "AT_LoRaWan_VERSION_STRING"\n\r");
  HAL_Delay(20);	
	MW_LOG(TS_OFF, VLEVEL_M, "Frequency Band: ");
	HAL_Delay(20);
	
	region_printf();
  new_firmware_update();
	key_printf();
	MW_LOG(TS_OFF, VLEVEL_M, "Enter Password to Active AT Commands\n\n\r");
  MW_LOG(TS_OFF, VLEVEL_M, "\n\rUse AT+DEBUG to see more debug info\n\r");
	HAL_Delay(40);
	
	lora_config.otaa = LORA_ENABLE;
			
  LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
  LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
  LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
  LoRaMacCallbacks.GetBatteryLevel = LoRaMainCallbacks->BoardGetBatteryLevel;
#if defined( REGION_AS923 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
#elif defined( REGION_AU915 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
#elif defined( REGION_CN470 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
#elif defined( REGION_CN779 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
#elif defined( REGION_EU433 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );
 #elif defined( REGION_IN865 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
#elif defined( REGION_EU868 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
#elif defined( REGION_KR920 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
#elif defined( REGION_US915 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
#elif defined( REGION_RU864 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_RU864 );
#elif defined( REGION_KZ865 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KZ865 );
#elif defined( REGION_MA869 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_MA869 );	
#else
    #error "Please define a region in the compiler options."
#endif

#ifdef REGION_AS923_JAPAN
  #ifndef REGION_AS923
    #error "REGION_AS923_JAPAN cannot be used without REGION_AS923; Please add macro REGION_AS923 to preprocessor."
  #endif
#endif

  mibReq.Type = MIB_ADR;
  mibReq.Param.AdrEnable = LoRaParamInit->AdrEnable;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_PUBLIC_NETWORK;
  mibReq.Param.EnablePublicNetwork = LoRaParamInit->EnablePublicNetwork;
  LoRaMacMibSetRequestConfirm( &mibReq );
          
  mibReq.Type = MIB_DEVICE_CLASS;
  mibReq.Param.Class= CLASS_A;
  LoRaMacMibSetRequestConfirm( &mibReq );

  lora_config.TxDatarate = LoRaParamInit->TxDatarate;
	
	/*
	 *0x00,AT+FDR
	 *0x12,FDR and join network
	 *0x13,backup Ext,TDC,add new configuration and join network
	 */
	uint8_t status=(*((uint8_t *) FLASH_USER_START_ADDR_CONFIG ));

	   if(status==0x00 || status==0x12 || status==0x13)// !=0,Automatic join network
			{
				if(status==0x00 || status==0x12)//FDR
				{
					
					lora_config.duty_cycle = LORA_DISABLE;
					lora_config.application_port=2;
				
					#if defined( REGION_US915 )|| defined ( REGION_AU915 )
					customize_config.set8channel=0;
					#elif defined( REGION_CN470 )
					customize_config.set8channel=11;
					#else
					customize_config.set8channel=0;
					#endif
					customize_config.freq1=0;

					inmode=2;						
					if(password_default(password_get)==0)  //default 123456
					{
						password_len=3;
						password_get[0]=18;
						password_get[1]=52;
						password_get[2]=86;		
					}
					else
					{
						password_len=3;
					}
					APP_TX_DUTYCYCLE=1200000;
					FDR_status=1;

					#if defined( REGION_AS923 )	|| defined( REGION_AU915 )
					dwelltime=1;
					#endif
					REJOIN_TX_DUTYCYCLE=30;//min
					response_level=0;
					currentLeapSecond=18;//As of June 2017, the GPS time is 18seconds ahead of UTC time.
					time_synchronization_interval=1;//1day
					time_synchronization_method=1;//0 via downlink,1 via devicetimereq
					pid_flag=0;
					pnackmd_switch=0;
					downlink_detect_switch=1;
					downlink_detect_timeout=2880;
					
					confirmed_uplink_retransmission_nbtrials=7;
					confirmed_uplink_counter_retransmission_increment_switch=0;
					
					LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=0;
					LinkADR_NbTrans_retransmission_nbtrials=1;
					
					unconfirmed_uplink_change_to_confirmed_uplink_timeout=1440;			
			  }
				
				if(status==0x00 || status==0x12 || status==0x13)//new firmware add new config,v1.0 now
				{
					inmode=2;						
					APP_TX_DUTYCYCLE=1200000;
				}		

				Flash_Store_Config();
				Flash_Read_Config();
				
				if(pnackmd_switch==1)
				{
					lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
					confirmed_uplink_retransmission_nbtrials=0;
					confirmed_uplink_counter_retransmission_increment_switch=0;
				}
				
				if(status==0x12 || status==0x13)
				{
					FDR_status=0;
					LORA_Join();
				}
				else
					MW_LOG(TS_OFF, VLEVEL_M, "Please set the parameters or reset Device to apply change\n\r");				  				
			}
			else 
			{					
				Flash_Read_Config();
				if(time_synchronization_interval==0)
				{
					time_synchronization_interval=1;//1day
				}
				
				if(collection_interval==0)
				{
					collection_interval=1;
				}
				
				if(REJOIN_TX_DUTYCYCLE==0)
				{
					REJOIN_TX_DUTYCYCLE=20;
				}
				
				if(downlink_detect_timeout==0)
				{
					downlink_detect_timeout=2880;
				}
				
				if(unconfirmed_uplink_change_to_confirmed_uplink_timeout==0)
				{
					unconfirmed_uplink_change_to_confirmed_uplink_timeout=1440;
				}
				
				if(LinkADR_NbTrans_retransmission_nbtrials==0)
				{
					LinkADR_NbTrans_retransmission_nbtrials=1;
				}
				
				if(pnackmd_switch==1)
				{
					lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
					confirmed_uplink_retransmission_nbtrials=0;
					confirmed_uplink_counter_retransmission_increment_switch=0;
				}
				
				if(APP_TX_DUTYCYCLE>=4000)
				{
					if(*((uint8_t *)(0x2000F00A))!=0xA8)
					{
					  LORA_Join();
					}
					else
						sleep_status=1;
				}
			}
}

void region_printf(void)
{
	current_firmware_ver=155;//1.0
/*just change current_firm_ver*/
#if defined( REGION_AS923 )
	#if defined(REGION_AS923_JAPAN)
	MW_LOG(TS_OFF, VLEVEL_M, "AS923-JP\n\r");
	current_fre_band=1;
	band_string="AS923-JP";
	#elif defined( AS923_2 )
	MW_LOG(TS_OFF, VLEVEL_M, "AS923-2\n\r");
	current_fre_band=14;	
	band_string="AS923-2";
	#elif defined( AS923_3 )
	MW_LOG(TS_OFF, VLEVEL_M, "AS923-3\n\r");
	current_fre_band=15;
  band_string="AS923-3";	
	#elif defined( AS923_4 )
	MW_LOG(TS_OFF, VLEVEL_M, "AS923-4\n\r");
	current_fre_band=16;
  band_string="AS923-4";		
	#else
  MW_LOG(TS_OFF, VLEVEL_M, "AS923\n\r");
	current_fre_band=2;
	band_string="AS923";
	#endif
#elif defined( REGION_AU915 )
  MW_LOG(TS_OFF, VLEVEL_M, "AU915\n\r");
	current_fre_band=3;
	band_string="AU915";
#elif defined( REGION_CN470 )
  MW_LOG(TS_OFF, VLEVEL_M, "CN470\n\r");
	current_fre_band=4;
	band_string="CN470";
#elif defined( REGION_CN779 )
  MW_LOG(TS_OFF, VLEVEL_M, "CN779\n\r");
	current_fre_band=5;
	band_string="CN779";
#elif defined( REGION_EU433 )
  MW_LOG(TS_OFF, VLEVEL_M, "EU433\n\r");
	current_fre_band=6;
	band_string="EU433";
#elif defined( REGION_IN865 )
  MW_LOG(TS_OFF, VLEVEL_M, "IN865\n\r");
	current_fre_band=7;
	band_string="IN865";
#elif defined( REGION_EU868 )
  MW_LOG(TS_OFF, VLEVEL_M, "EU868\n\r");
	current_fre_band=8;
	band_string="EU868";
#elif defined( REGION_KR920 )
  MW_LOG(TS_OFF, VLEVEL_M, "KR920\n\r");
	current_fre_band=9;
	band_string="KR920";
#elif defined( REGION_US915 )
  MW_LOG(TS_OFF, VLEVEL_M, "US915\n\r");
	current_fre_band=10;
	band_string="US915";
#elif defined( REGION_RU864 )
  MW_LOG(TS_OFF, VLEVEL_M, "RU864\n\r");	
	current_fre_band=11;
	band_string="RU864";
#elif defined( REGION_KZ865 )
  MW_LOG(TS_OFF, VLEVEL_M, "KZ865\n\r");	
	current_fre_band=12;
	band_string="KZ865";
#elif defined( REGION_MA869 )
  MW_LOG(TS_OFF, VLEVEL_M, "MA869\n\r");	
	current_fre_band=13;
	band_string="MA869";
#else
    #error "Please define a region in the compiler options."
#endif
}

void key_printf(void)
{
  MW_LOG(TS_OFF, VLEVEL_M, "DevEui= %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX8(lora_config.DevEui));
}

void LORA_Join( void)
{
	if(device_UUID_error_status==0)
	{
		if (lora_config.otaa == LORA_ENABLE)
		{
			MlmeReq_t mlmeReq;
		
			mlmeReq.Type = MLME_JOIN;
			mlmeReq.Req.Join.DevEui = lora_config.DevEui;
			mlmeReq.Req.Join.AppEui = lora_config.AppEui;
			mlmeReq.Req.Join.AppKey = lora_config.AppKey;
			mlmeReq.Req.Join.NbTrials = LoRaParamInit->NbTrials;
		
			JoinParameters = mlmeReq.Req.Join;

			LoRaMacMlmeRequest( &mlmeReq );
		}
		else
		{
			mibReq.Type = MIB_NET_ID;
			mibReq.Param.NetID = LORAWAN_NETWORK_ID;
			LoRaMacMibSetRequestConfirm( &mibReq );

			mibReq.Type = MIB_DEV_ADDR;
			mibReq.Param.DevAddr = lora_config.DevAddr;
			LoRaMacMibSetRequestConfirm( &mibReq );

			mibReq.Type = MIB_NWK_SKEY;
			mibReq.Param.NwkSKey = lora_config.NwkSKey;
			LoRaMacMibSetRequestConfirm( &mibReq );

			mibReq.Type = MIB_APP_SKEY;
			mibReq.Param.AppSKey = lora_config.AppSKey;
			LoRaMacMibSetRequestConfirm( &mibReq );

			mibReq.Type = MIB_NETWORK_JOINED;
			mibReq.Param.IsNetworkJoined = true;
			LoRaMacMibSetRequestConfirm( &mibReq );

			LoRaMainCallbacks->LORA_HasJoined();
		}
  }
}

LoraFlagStatus LORA_JoinStatus( void)
{
  MibRequestConfirm_t mibReq;

  mibReq.Type = MIB_NETWORK_JOINED;
  
  LoRaMacMibGetRequestConfirm( &mibReq );

  if( mibReq.Param.IsNetworkJoined == true )
  {
    return LORA_SET;
  }
  else
  {
    return LORA_RESET;
  }
}

LoraErrorStatus LORA_send(lora_AppData_t* AppData, LoraConfirm_t IsTxConfirmed)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
	  uint8_t status=0;
  
    /*if certification test are on going, application data is not sent*/
    if (certif_running() == true)
    {
      return LORA_ERROR;
    }
    
		status=LoRaMacQueryTxPossible( AppData->BuffSize, &txInfo );
		
		if(work_mode==3)
		{
			while( LoRaMacQueryTxPossible( AppData->BuffSize, &txInfo ) != LORAMAC_STATUS_OK );
			status=LORAMAC_STATUS_OK;
		}
		
    if( status != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = lora_config_tx_datarate_get() ;
    }
    else
    {
        if( IsTxConfirmed == LORAWAN_UNCONFIRMED_MSG )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppData->Port;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppData->BuffSize;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData->Buff;
            mcpsReq.Req.Unconfirmed.Datarate = lora_config_tx_datarate_get() ;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppData->Port;
            mcpsReq.Req.Confirmed.fBufferSize = AppData->BuffSize;
            mcpsReq.Req.Confirmed.fBuffer = AppData->Buff;
					
					  if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
						{
							  mcpsReq.Req.Confirmed.NbTrials =1;
						}
						else
						{						
                mcpsReq.Req.Confirmed.NbTrials = confirmed_uplink_retransmission_nbtrials+1;
						}
            mcpsReq.Req.Confirmed.Datarate = lora_config_tx_datarate_get() ;
        }
    }
    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return LORA_SUCCESS;
    }
    return LORA_ERROR;
}  

LoraErrorStatus LORA_RequestClass( DeviceClass_t newClass )
{
  LoraErrorStatus Errorstatus = LORA_SUCCESS;
  MibRequestConfirm_t mibReq;
  DeviceClass_t currentClass;
  
  mibReq.Type = MIB_DEVICE_CLASS;
  LoRaMacMibGetRequestConfirm( &mibReq );
  
  currentClass = mibReq.Param.Class;
  /*attempt to swicth only if class update*/
  if (currentClass != newClass)
  {
    switch (newClass)
    {
      case CLASS_A:
      {
        if (currentClass == CLASS_A)
        {
          mibReq.Param.Class = CLASS_A;
          if( LoRaMacMibSetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
          {
          /*switch is instantanuous*/
            LoRaMainCallbacks->LORA_ConfirmClass(CLASS_A);
          }
          else
          {
            Errorstatus = LORA_ERROR;
          }
        }
        break;
      }
      case CLASS_C:
      {
        if (currentClass != CLASS_A)
        {
          Errorstatus = LORA_ERROR;
        }
        /*switch is instantanuous*/
        mibReq.Param.Class = CLASS_C;
        if( LoRaMacMibSetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
        {
          LoRaMainCallbacks->LORA_ConfirmClass(CLASS_C);
        }
        else
        {
            Errorstatus = LORA_ERROR;
        }
        break;
      }
      default:
        break;
    } 
  }
  return Errorstatus;
}

void LORA_GetCurrentClass( DeviceClass_t *currentClass )
{
  MibRequestConfirm_t mibReq;
  
  mibReq.Type = MIB_DEVICE_CLASS;
  LoRaMacMibGetRequestConfirm( &mibReq );
  
  *currentClass = mibReq.Param.Class;
}


void lora_config_otaa_set(LoraState_t otaa)
{
  lora_config.otaa = otaa;
}


LoraState_t lora_config_otaa_get(void)
{
  return lora_config.otaa;
}

void lora_config_duty_cycle_set(LoraState_t duty_cycle)
{
  lora_config.duty_cycle = duty_cycle;
	#if defined( REGION_EU868 )
  LoRaMacTestSetDutyCycleOn((duty_cycle == LORA_ENABLE) ? 1 : 0);
	#endif
}

LoraState_t lora_config_duty_cycle_get(void)
{
  return lora_config.duty_cycle;
}

void lora_config_deveui_get(uint8_t *deveui)
{
  memcpy1(deveui, lora_config.DevEui, sizeof(lora_config.DevEui));
}

void lora_config_deveui_set(uint8_t deveui[8])
{
  memcpy1(lora_config.DevEui, deveui, sizeof(lora_config.DevEui));
}

void lora_config_appeui_get(uint8_t *appeui)
{
  memcpy1(appeui, lora_config.AppEui, sizeof(lora_config.AppEui));
}

void lora_config_appeui_set(uint8_t appeui[8])
{
  memcpy1(lora_config.AppEui, appeui, sizeof(lora_config.AppEui));
}

void lora_config_appkey_get(uint8_t *appkey)
{
  memcpy1(appkey, lora_config.AppKey, sizeof(lora_config.AppKey));
}

void lora_config_appkey_set(uint8_t appkey[16])
{
  memcpy1(lora_config.AppKey, appkey, sizeof(lora_config.AppKey));
}

uint32_t lora_config_devaddr_get(void)
{
	return lora_config.DevAddr;
}

void lora_config_devaddr_set(uint32_t devaddr)
{
	lora_config.DevAddr=devaddr;
}

void lora_config_appskey_get(uint8_t *appskey)
{
  memcpy1(appskey, lora_config.AppSKey, sizeof(lora_config.AppSKey));
}

void lora_config_appskey_set(uint8_t appskey[16])
{
  memcpy1(lora_config.AppSKey, appskey, sizeof(lora_config.AppSKey));
}

void lora_config_nwkskey_get(uint8_t *nwkskey)
{
  memcpy1(nwkskey, lora_config.NwkSKey, sizeof(lora_config.NwkSKey));
}

void lora_config_nwkskey_set(uint8_t nwkskey[16])
{
  memcpy1(lora_config.NwkSKey, nwkskey, sizeof(lora_config.NwkSKey));
}

void lora_config_reqack_set(LoraConfirm_t reqack)
{
  lora_config.ReqAck = reqack;
}

LoraConfirm_t lora_config_reqack_get(void)
{
  return lora_config.ReqAck;
}

int8_t lora_config_snr_get(void)
{
  return lora_config.Snr;
}

void lora_config_application_port_set(uint8_t application_port)
{
	lora_config.application_port=application_port;
}

uint8_t lora_config_application_port_get(void )
{
	return lora_config.application_port;
}

int16_t lora_config_rssi_get(void)
{
  return lora_config.Rssi;
}

void lora_config_tx_datarate_set(int8_t TxDataRate)
{
  lora_config.TxDatarate =TxDataRate;
}

int8_t lora_config_tx_datarate_get(void )
{
  return lora_config.TxDatarate;
}

LoraState_t lora_config_isack_get(void)
{
  if (lora_config.McpsConfirm == NULL)
  {
    return LORA_DISABLE;
  }
  else
  {
    return (lora_config.McpsConfirm->AckReceived ? LORA_ENABLE : LORA_DISABLE);
  }
}

void Flash_store_key(void)
{
	  uint8_t store_key_in_flash[128];
	  
//    while(print_isdone()==0);
	
    for(uint8_t i=0,j=0;i<8;i++,j++)
    {
     store_key_in_flash[i]= lora_config.DevEui[j];
    }
        
    for(uint8_t i=8,j=0;i<16;i++,j++)
    {
      store_key_in_flash[i]=lora_config.AppEui[j];
    }
    
    for(uint8_t i=16,j=0;i<32;i++,j++)
    {
      store_key_in_flash[i]=lora_config.AppKey[j];
    }
    
    for(uint8_t i=32,j=24;i<36;i++,j-=8)
    {
      store_key_in_flash[i]=(lora_config.DevAddr>>j)&0xff;
    }
        
    for(uint8_t i=36,j=0;i<52;i++,j++)
    {
      store_key_in_flash[i]=lora_config.NwkSKey[j];
    }
       
    for(uint8_t i=52,j=0;i<68;i++,j++)
    {
      store_key_in_flash[i]=lora_config.AppSKey[j];
    }        
    
    store_key_in_flash[68]=(uuid1_in_sflash>>24)&0xFF;
    store_key_in_flash[69]=(uuid1_in_sflash>>16)&0xFF;
    store_key_in_flash[70]=(uuid1_in_sflash>>8)&0xFF;
    store_key_in_flash[71]=uuid1_in_sflash&0xFF;

    store_key_in_flash[72]=(uuid2_in_sflash>>24)&0xFF;
    store_key_in_flash[73]=(uuid2_in_sflash>>16)&0xFF;
    store_key_in_flash[74]=(uuid2_in_sflash>>8)&0xFF;
    store_key_in_flash[75]=uuid2_in_sflash&0xFF;
		
		__disable_irq();
//		flash_erase_page(FLASH_USER_START_ADDR_KEY);
		FLASH_IF_Erase((void *)FLASH_USER_START_ADDR_KEY, FLASH_PAGE_SIZE);
		HAL_Delay(5);
//		if(flash_program_bytes(FLASH_USER_START_ADDR_KEY,store_key_in_flash,128)==ERRNO_FLASH_SEC_ERROR)
		if(FLASH_IF_Write((void *)FLASH_USER_START_ADDR_KEY, store_key_in_flash, 128) ==FLASH_IF_ERROR)
		{
			MW_LOG(TS_OFF, VLEVEL_M, "write key error\r\n");
		}
		__enable_irq();
}

void Flash_read_key(void)
{
	  uint8_t read_key_in_flash[128];
	
//	  while(print_isdone()==0);
	
	  for (uint8_t i = 0; i < 128; i++)
    {
      read_key_in_flash[i]=(*((uint8_t *) (FLASH_USER_START_ADDR_KEY + i)));      
    }
		
		for(uint8_t i=0,j=0;i<8;i++,j++)
    {
      lora_config.DevEui[j]=read_key_in_flash[i];
    }   
    
    for(uint8_t i=8,j=0;i<16;i++,j++)
    {
      lora_config.AppEui[j]=read_key_in_flash[i];
    }
    
    for(uint8_t i=16,j=0;i<32;i++,j++)
    {
      lora_config.AppKey[j]=read_key_in_flash[i];
    }
    
    lora_config.DevAddr = read_key_in_flash[32] << 24 | read_key_in_flash[33] << 16 | read_key_in_flash[34] <<8 | read_key_in_flash[35];
    
    for(uint8_t i=36,j=0;i<52;i++,j++)
    {
      lora_config.NwkSKey[j]=read_key_in_flash[i];
    }
    
    for(uint8_t i=52,j=0;i<68;i++,j++)
    {
      lora_config.AppSKey[j]=read_key_in_flash[i];
    }    
    
    uuid1_in_sflash=read_key_in_flash[68]<<24|read_key_in_flash[69]<<16|read_key_in_flash[70]<<8|read_key_in_flash[71];
    uuid2_in_sflash=read_key_in_flash[72]<<24|read_key_in_flash[73]<<16|read_key_in_flash[74]<<8|read_key_in_flash[75];
}

void Flash_Store_Config(void)
{
	uint8_t store_config_in_flash[128];
	
//	while(print_isdone()==0);
	
	store_config_in_flash[0]=0x11;//FDR_status
	
	store_config_in_flash[1]=product_id;
	store_config_in_flash[2]=current_fre_band;
	store_config_in_flash[3]=current_firmware_ver;
	
	MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[4]=mib.Param.AdrEnable;
	
	mib.Type = MIB_CHANNELS_TX_POWER;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[5]=mib.Param.ChannelsTxPower;
	
  store_config_in_flash[6]=lora_config.TxDatarate;
	
	store_config_in_flash[7]=lora_config.duty_cycle;	

	 mib.Type = MIB_PUBLIC_NETWORK;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[8]=mib.Param.EnablePublicNetwork;
	
	store_config_in_flash[9]=lora_config.otaa;
	
	mib.Type = MIB_DEVICE_CLASS;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[10]=mib.Param.Class;//0:CLASS A
	
	store_config_in_flash[11]=lora_config.ReqAck;
	
	mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[12]=mib.Param.Rx2Channel.Frequency>>24 & 0xFF;
	store_config_in_flash[13]=mib.Param.Rx2Channel.Frequency>>16 & 0xFF;
	store_config_in_flash[14]=mib.Param.Rx2Channel.Frequency>>8 & 0xFF;
	store_config_in_flash[15]=mib.Param.Rx2Channel.Frequency & 0xFF;
	
	if((join_mothod==0)||(RX2DR_setting_status==1))
	{
		RX2DR_setting_status=0;
		mib.Type = MIB_RX2_CHANNEL;
		status = LoRaMacMibGetRequestConfirm(&mib);
		if(status!=LORAMAC_STATUS_OK)
		{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
		store_config_in_flash[16]=mib.Param.Rx2Channel.Datarate;
  }
	else
	{
		mib.Type = MIB_RX2_DEFAULT_CHANNEL;
		status = LoRaMacMibGetRequestConfirm(&mib);
		store_config_in_flash[16]=mib.Param.Rx2DefaultChannel.Datarate;
	}	
	
	mib.Type = MIB_RECEIVE_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[17]=mib.Param.ReceiveDelay1>>8 & 0xFF;
	store_config_in_flash[18]=mib.Param.ReceiveDelay1 & 0xFF;
	
	mib.Type = MIB_RECEIVE_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[19]=mib.Param.ReceiveDelay2>>8 & 0xFF;
	store_config_in_flash[20]=mib.Param.ReceiveDelay2 & 0xFF;
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[21]=mib.Param.JoinAcceptDelay1>>8 & 0xFF;
	store_config_in_flash[22]=mib.Param.JoinAcceptDelay1 & 0xFF;
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[23]=mib.Param.JoinAcceptDelay2>>8 & 0xFF;
	store_config_in_flash[24]=mib.Param.JoinAcceptDelay2 & 0xFF;
	
	mib.Type = MIB_NET_ID;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{MW_LOG(TS_OFF, VLEVEL_M, "LORAMAC STATUS ERROR\n\r");}

	store_config_in_flash[25]=mib.Param.NetID>>24 & 0xFF;
	store_config_in_flash[26]=mib.Param.NetID>>16 & 0xFF;
	store_config_in_flash[27]=mib.Param.NetID>>8 & 0xFF;
	store_config_in_flash[28]=mib.Param.NetID & 0xFF;

	store_config_in_flash[29]=APP_TX_DUTYCYCLE>>24 & 0xFF;
	store_config_in_flash[30]=APP_TX_DUTYCYCLE>>16 & 0xFF;
	store_config_in_flash[31]=APP_TX_DUTYCYCLE>>8 & 0xFF;
	store_config_in_flash[32]=APP_TX_DUTYCYCLE & 0xFF;
	
	store_config_in_flash[33]=lora_config.application_port;
	
	store_config_in_flash[34]=password_get[0];
	store_config_in_flash[35]=password_get[1];
	store_config_in_flash[36]=password_get[2];
	store_config_in_flash[37]=password_get[3];
	store_config_in_flash[38]=password_get[4];
	store_config_in_flash[39]=password_get[5];
	store_config_in_flash[40]=password_get[6];
	store_config_in_flash[41]=password_get[7];
	store_config_in_flash[42]=password_len;
	
	store_config_in_flash[43]=customize_config.freq1>>24 & 0xFF;
	store_config_in_flash[44]=customize_config.freq1>>16 & 0xFF;
	store_config_in_flash[45]=customize_config.freq1>>8 & 0xFF;
	store_config_in_flash[46]=customize_config.freq1 & 0xFF;
	
	store_config_in_flash[47]=customize_config.set8channel; 

	store_config_in_flash[51]=REJOIN_TX_DUTYCYCLE>>8 & 0xFF;
	store_config_in_flash[52]=REJOIN_TX_DUTYCYCLE & 0xFF;
	
	store_config_in_flash[53]=response_level;
	store_config_in_flash[54]=dwelltime;
	
	store_config_in_flash[55]=currentLeapSecond;
	store_config_in_flash[56]=time_synchronization_method;
	store_config_in_flash[57]=time_synchronization_interval;
	store_config_in_flash[58]=pid_flag;
	
	store_config_in_flash[59]=downlink_detect_switch;
	
	store_config_in_flash[60]=downlink_detect_timeout>>8 & 0xFF;
	store_config_in_flash[61]=downlink_detect_timeout & 0xFF;
	store_config_in_flash[62]=confirmed_uplink_retransmission_nbtrials;
	store_config_in_flash[63]=confirmed_uplink_counter_retransmission_increment_switch;
	store_config_in_flash[64]=LinkADR_NbTrans_retransmission_nbtrials;
	store_config_in_flash[65]=LinkADR_NbTrans_uplink_counter_retransmission_increment_switch;
	store_config_in_flash[66]=unconfirmed_uplink_change_to_confirmed_uplink_timeout>>8 & 0xFF;
	store_config_in_flash[67]=unconfirmed_uplink_change_to_confirmed_uplink_timeout & 0xFF;
	store_config_in_flash[68]=pnackmd_switch;
	store_config_in_flash[69]=extpower_logic;
	store_config_in_flash[70]=power_time>>8 & 0xFF;
	store_config_in_flash[71]=power_time & 0xFF;
	store_config_in_flash[72]=exitintmode;
	store_config_in_flash[73]=exitmode;
	store_config_in_flash[74]=comp_temp1_exttemp>>8 & 0xFF;
	store_config_in_flash[75]=comp_temp1_exttemp & 0xFF;	
	store_config_in_flash[76]=comp_temp2_exttemp>>8 & 0xFF;	
	store_config_in_flash[77]=comp_temp2_exttemp & 0xFF;
  store_config_in_flash[78]=temp_change;
  store_config_in_flash[79]=collection_second;	
	store_config_in_flash[80]=led_alarm_switch;
	store_config_in_flash[81]=external_sensor_type_for_work_mode3;
  store_config_in_flash[82]=external_sensor_sample_interval_for_work_mode3>>8 & 0xFF;	
	store_config_in_flash[83]=external_sensor_sample_interval_for_work_mode3 & 0xFF;	
	store_config_in_flash[84]=external_sensor_sample_sum_for_work_mode3;
	store_config_in_flash[85]=temp_alarm_switch_for_work_mode3;
	store_config_in_flash[86]=valid_flag;
	store_config_in_flash[87]=valid_time>>8 & 0xFF;
	store_config_in_flash[88]=valid_time;
	store_config_in_flash[89]=s31f_ext;

  store_config_in_flash[90]=inmode;
	store_config_in_flash[91]=inmode_delay>>8 & 0xFF;
	store_config_in_flash[92]=inmode_delay;
	
	__disable_irq();
//	flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
  FLASH_IF_Erase((void *)FLASH_USER_START_ADDR_CONFIG, FLASH_PAGE_SIZE);
	HAL_Delay(5);
//	if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,store_config_in_flash,128)==ERRNO_FLASH_SEC_ERROR)
  if(FLASH_IF_Write((void *)FLASH_USER_START_ADDR_CONFIG, store_config_in_flash, 128) ==FLASH_IF_ERROR)
	{
		MW_LOG(TS_OFF, VLEVEL_M, "write config error\r\n");
	}
	__enable_irq();
}
	
void Flash_Read_Config(void)
{
	uint8_t read_config_in_flash[128];
	
//	while(print_isdone()==0);
	
	for(int i=0;i<128;i++)
	{
	  read_config_in_flash[i]=(*((uint8_t *) (FLASH_USER_START_ADDR_CONFIG + i)));
	}
	
	product_id_read_in_flash=read_config_in_flash[1];
	fre_band_read_in_flash=read_config_in_flash[2];
	firmware_ver_read_in_flash=read_config_in_flash[3];
	
	MibRequestConfirm_t mib;
	
  mib.Type = MIB_ADR;
	mib.Param.AdrEnable=read_config_in_flash[4];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_CHANNELS_TX_POWER;
	mib.Param.ChannelsTxPower=read_config_in_flash[5];
	LoRaMacMibSetRequestConfirm( &mib );
	txp_power=read_config_in_flash[5];
	
	lora_config.TxDatarate=read_config_in_flash[6];
	
	if(read_config_in_flash[7]==0x01)
	{
		lora_config.duty_cycle=LORA_ENABLE;
	}
	else
	{
		lora_config.duty_cycle=LORA_DISABLE;
	}
	
  #if defined( REGION_EU868 )		
	LoRaMacTestSetDutyCycleOn((lora_config.duty_cycle == LORA_ENABLE) ? 1 : 0);
	#endif
		
	mib.Type = MIB_PUBLIC_NETWORK;
	mib.Param.EnablePublicNetwork=read_config_in_flash[8];
	LoRaMacMibSetRequestConfirm( &mib );
	
	if(read_config_in_flash[9]==0x01)
	{
		join_mothod=1;
		lora_config.otaa=LORA_ENABLE;
	}
	else
	{
		join_mothod=0;
		lora_config.otaa=LORA_DISABLE;
	}
		
	mib.Type = MIB_DEVICE_CLASS;
	mib.Param.Class=(DeviceClass_t)(read_config_in_flash[10]);
	LoRaMacMibSetRequestConfirm( &mib );
		
	if(read_config_in_flash[11]==0x01)
	{
		lora_config.ReqAck=LORAWAN_CONFIRMED_MSG;
	}
	else
	{
		lora_config.ReqAck=LORAWAN_UNCONFIRMED_MSG;
	}
	
	mib.Type = MIB_RX2_CHANNEL;
	mib.Param.Rx2Channel.Frequency=read_config_in_flash[12]<<24 | read_config_in_flash[13]<<16 | read_config_in_flash[14]<<8 | read_config_in_flash[15];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_RX2_CHANNEL;
	mib.Param.Rx2Channel.Datarate=read_config_in_flash[16];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
	mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ read_config_in_flash[12]<<24 | read_config_in_flash[13]<<16 | read_config_in_flash[14]<<8 | read_config_in_flash[15], read_config_in_flash[16] };
	LoRaMacMibSetRequestConfirm( &mibReq );
	
  mib.Type = MIB_RECEIVE_DELAY_1;
	mib.Param.ReceiveDelay1=read_config_in_flash[17]<<8 | read_config_in_flash[18];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_RECEIVE_DELAY_2;
	mib.Param.ReceiveDelay2=read_config_in_flash[19]<<8 | read_config_in_flash[20];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
	mib.Param.JoinAcceptDelay1=read_config_in_flash[21]<<8 | read_config_in_flash[22];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
	mib.Param.JoinAcceptDelay2=read_config_in_flash[23]<<8 | read_config_in_flash[24];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_NET_ID;
	mib.Param.NetID=read_config_in_flash[25]<<24 | read_config_in_flash[26]<<16 | read_config_in_flash[27]<<8 | read_config_in_flash[28];
	LoRaMacMibSetRequestConfirm( &mib );
	
	APP_TX_DUTYCYCLE=read_config_in_flash[29]<<24 | read_config_in_flash[30]<<16 | read_config_in_flash[31]<<8 | read_config_in_flash[32];
	
	lora_config.application_port=read_config_in_flash[33];
	
	password_get[0]=read_config_in_flash[34];
	password_get[1]=read_config_in_flash[35];
	password_get[2]=read_config_in_flash[36];
	password_get[3]=read_config_in_flash[37];
	password_get[4]=read_config_in_flash[38];
	password_get[5]=read_config_in_flash[39];
	password_get[6]=read_config_in_flash[40];
	password_get[7]=read_config_in_flash[41];
  password_len=read_config_in_flash[42];
	
	customize_config.freq1=read_config_in_flash[43]<<24 | read_config_in_flash[44]<<16 | read_config_in_flash[45]<<8 | read_config_in_flash[46];
	
	customize_config.set8channel=read_config_in_flash[47];

	REJOIN_TX_DUTYCYCLE=read_config_in_flash[51]<<8 | read_config_in_flash[52];
	
	response_level=read_config_in_flash[53];
	
	dwelltime=read_config_in_flash[54];
	
	currentLeapSecond=read_config_in_flash[55];
	
	time_synchronization_method=read_config_in_flash[56];
	
	time_synchronization_interval=read_config_in_flash[57];
	
	pid_flag=read_config_in_flash[58];
	
	downlink_detect_switch=read_config_in_flash[59];
	
	downlink_detect_timeout=read_config_in_flash[60]<<8 | read_config_in_flash[61];
	
	confirmed_uplink_retransmission_nbtrials=read_config_in_flash[62];
	
	confirmed_uplink_counter_retransmission_increment_switch=read_config_in_flash[63];
	
	LinkADR_NbTrans_retransmission_nbtrials=read_config_in_flash[64];
	LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=read_config_in_flash[65];
	
	unconfirmed_uplink_change_to_confirmed_uplink_timeout=read_config_in_flash[66]<<8 | read_config_in_flash[67];
	
	pnackmd_switch=read_config_in_flash[68];
	
	extpower_logic=read_config_in_flash[69];
	
	power_time=read_config_in_flash[70]<<8 | read_config_in_flash[71];
	
	exitintmode=read_config_in_flash[72];
	
	exitmode=read_config_in_flash[73];

	comp_temp1_exttemp=read_config_in_flash[74]<<8 | read_config_in_flash[75];
	
	if(comp_temp1_exttemp<0)
  comp_temp1_exttemp=-1.0*(~comp_temp1_exttemp+1);
	
	comp_temp2_exttemp=read_config_in_flash[76]<<8 | read_config_in_flash[77];
	
	if(comp_temp2_exttemp<0)
  comp_temp2_exttemp=-1.0*(~comp_temp2_exttemp+1);
	
  temp_change=read_config_in_flash[78];
	
  collection_second=read_config_in_flash[79];	
	
	led_alarm_switch=read_config_in_flash[80];	
	
	external_sensor_type_for_work_mode3=read_config_in_flash[81];
	
	external_sensor_sample_interval_for_work_mode3=read_config_in_flash[82]<<8 | read_config_in_flash[83];
	
	external_sensor_sample_sum_for_work_mode3=read_config_in_flash[84];
	
	temp_alarm_switch_for_work_mode3=read_config_in_flash[85];
	
	valid_flag=read_config_in_flash[86];
	
	valid_time=read_config_in_flash[87]<<8 | read_config_in_flash[88];
	
	s31f_ext=read_config_in_flash[89];
	
	inmode=read_config_in_flash[90];
	
	inmode_delay=read_config_in_flash[91]<<8 | read_config_in_flash[92];
}

static void new_firmware_update(void)
{
	uint8_t status[128]={0x12};

	status[0]=(*((uint8_t *) FLASH_USER_START_ADDR_CONFIG ));
	
	if(status[0]!=0x00)
	{
		if((product_id!=product_id_read_in_flash) || (current_fre_band!=fre_band_read_in_flash))
		{
			memset(status, 0x00, 128);
			status[0]=0x12;
	
//			while(print_isdone()==0);
			__disable_irq();
//			flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
			FLASH_IF_Erase((void *)FLASH_USER_START_ADDR_CONFIG, FLASH_PAGE_SIZE);
			HAL_Delay(5);
//			if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
			if(FLASH_IF_Write((void *)FLASH_USER_START_ADDR_CONFIG, status, 128) ==FLASH_IF_ERROR)	
			{
				MW_LOG(TS_OFF, VLEVEL_M, "write config error\r\n");
			}
			__enable_irq();
		}
		else if((current_firmware_ver!=firmware_ver_read_in_flash))
		{
			memset(status, 0x00, 128);
			status[0]=0x13;
			
//			while(print_isdone()==0);
			__disable_irq();
			//add new configuration
//			flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
			FLASH_IF_Erase((void *)FLASH_USER_START_ADDR_CONFIG, FLASH_PAGE_SIZE);
			HAL_Delay(5);
//			if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
			if(FLASH_IF_Write((void *)FLASH_USER_START_ADDR_CONFIG, status, 128) ==FLASH_IF_ERROR)
			{
				MW_LOG(TS_OFF, VLEVEL_M, "write config error\r\n");
			}
			__enable_irq();
		}
  }
}

uint8_t Uplink_data_adaptive_rate(lora_AppData_t* AppData)
{
	LoRaMacTxInfo_t txInfo;
	if( LoRaMacQueryTxPossible( AppData->BuffSize, &txInfo ) != LORAMAC_STATUS_OK )
	{
		return 0;
	}
	else
		return 1;
}

uint8_t UID_COMP(void)
{
	uint32_t unique_id[2];
	
	unique_id[0] = HAL_GetUIDw0();
	unique_id[1] = HAL_GetUIDw1();
	
	unique_id[0]=unique_id[0]^unique_id[1];
	unique_id[1]=unique_id[0]&unique_id[1];
	
	if(unique_id[0]!=uuid1_in_sflash||unique_id[1]!=uuid2_in_sflash)
	{
		return 0;
	}
	else
		return 1;
}

uint8_t password_default(uint8_t* pawo)
{
	uint8_t retemp=1;
	
	if( (lora_config.AppSKey[0]==0xff)&&(lora_config.AppSKey[1]==0xff)
		&&(lora_config.AppSKey[2]==0xff)&&(lora_config.AppSKey[3]==0xff)
	  &&(lora_config.AppSKey[4]==0xff)&&(lora_config.AppSKey[5]==0xff)
	  &&(lora_config.AppSKey[6]==0xff)&&(lora_config.AppSKey[7]==0xff)
		&&(lora_config.AppSKey[8]==0xff)&&(lora_config.AppSKey[9]==0xff)
		&&(lora_config.AppSKey[10]==0xff)&&(lora_config.AppSKey[11]==0xff)
	  &&(lora_config.AppSKey[12]==0xff)&&(lora_config.AppSKey[13]==0xff)
	  &&(lora_config.AppSKey[14]==0xff)&&(lora_config.AppSKey[15]==0xff))
	{
		retemp=0;
	}
	else
	{
		for(uint8_t i=0,j=13;j<16;i++,j++)
		{
			pawo[i]=lora_config.AppSKey[j];
		}
	}
	
	return retemp;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
