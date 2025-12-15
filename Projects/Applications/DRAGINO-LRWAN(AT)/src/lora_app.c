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
#include "log.h"
#include "tremo_system.h"
#include "tremo_flash.h"
#include "stdlib.h"
#include "tremo_delay.h"
#include "LoRaMacTest.h"

/*
product_id
0:LA66_ST
1:LTC2
2:LHT65
3:LSN50
4:LDDS75
5:LSE01
6:LDDS20
7:RS485-LN
254:PS_LB
*/

#if defined( REGION_AS923 )
	#if defined(REGION_AS923_JAPAN)
	#define Firm_FQ 1
	#elif defined( AS923_2 )
	#define Firm_FQ 14		
	#elif defined( AS923_3 )
	#define Firm_FQ 15	
	#elif defined( AS923_4 )
	#define Firm_FQ 16	
	#else
	#define Firm_FQ 2	
	#endif
#elif defined( REGION_AU915 )
	#define Firm_FQ 3	
#elif defined( REGION_CN470 )
	#define Firm_FQ 4	
#elif defined( REGION_CN779 )
	#define Firm_FQ 5	
#elif defined( REGION_EU433 )
	#define Firm_FQ 6	
#elif defined( REGION_IN865 )
	#define Firm_FQ 7	
#elif defined( REGION_EU868 )
	#define Firm_FQ 8	
#elif defined( REGION_KR920 )
	#define Firm_FQ 9	
#elif defined( REGION_US915 )
	#define Firm_FQ 10	
#elif defined( REGION_RU864 )
	#define Firm_FQ 11	
#elif defined( REGION_KZ865 )
	#define Firm_FQ 12	
#elif defined( REGION_MA869 )
	#define Firm_FQ 13	
#else
	#define Firm_FQ 255
#endif

uint8_t product_id=0x1C;
uint8_t fire_version=0;
uint8_t current_fre_band=0;
uint8_t product_id_read_in_flash=0;
uint8_t fre_band_read_in_flash=0;
uint8_t firmware_ver_read_in_flash=0;
uint8_t workmode;
uint8_t pwm_timer;
uint16_t power_5v_time;
char *band_string="";
bool FDR_status=0;
bool down_check;
bool joined_flags=0;
uint8_t RX2DR_setting_status;
uint32_t uuid1_in_sflash,uuid2_in_sflash;
uint8_t device_UUID_error_status=0;
uint8_t password_len;
uint8_t password_get[8];
uint8_t inmode,inmode2,inmode3;
uint16_t inmode_delay,inmode2_delay,inmode3_delay;
uint8_t decrypt_flag=0;

extern float GapValue;
extern bool mac_response_flag;
extern bool rejoin_status;
extern uint32_t APP_TX_DUTYCYCLE;
extern uint16_t REJOIN_TX_DUTYCYCLE;
extern uint8_t response_level;

extern bool JoinReq_NbTrails_over;
extern bool unconfirmed_downlink_data_ans_status,confirmed_downlink_data_ans_status;

extern bool joined_finish;
extern uint8_t dwelltime;
extern uint8_t symbtime1_value;
extern uint8_t flag1;
extern uint8_t symbtime2_value;
extern uint8_t flag2;
extern uint8_t downlink_detect_switch;
extern uint16_t downlink_detect_timeout;

extern uint8_t confirmed_uplink_counter_retransmission_increment_switch;
extern uint8_t confirmed_uplink_retransmission_nbtrials;

extern uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch;
extern uint8_t LinkADR_NbTrans_retransmission_nbtrials;
extern uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout;

extern bool print_isdone(void);

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
	
	if(UID_COMP()==0)
	{
		device_UUID_error_status=1;
		LOG_PRINTF(LL_DEBUG,"Invalid credentials,the device goes into low power mode\r\n");
	}

	#if defined LB_LS
  LOG_PRINTF(LL_DEBUG,"\n\rDragino SN50_v3-LS Device\n\r");
	#else	
	LOG_PRINTF(LL_DEBUG,"\n\rDragino SN50_v3-LB Device\n\r");
	#endif
	LOG_PRINTF(LL_DEBUG,"Image Version: "AT_VERSION_STRING"\n\r");
	LOG_PRINTF(LL_DEBUG,"LoRaWan Stack: "AT_LoRaWan_VERSION_STRING"\n\r");	
	LOG_PRINTF(LL_DEBUG,"Frequency Band: ");
	
	region_printf();
	key_printf();
	LOG_PRINTF(LL_DEBUG,"Enter Password to Active AT Commands\n\r");
  LOG_PRINTF(LL_DEBUG,"\n\rUse AT+DEBUG to see more debug info\r\n\r\n");
	
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
				
	uint8_t status=(*((uint8_t *) FLASH_USER_START_ADDR_CONFIG ));

	if((status==0x00)||(status==0xff))// !=0,Automatic join network
	{
		fdr_config();	
    LOG_PRINTF(LL_DEBUG,"\r\nPlease set the parameters or reset Device to apply change\n\r");							
	}
	else if(status==0x12)
	{
		fdr_config();						
		system_reset();				
	}					
  else 
  {				
		Flash_Read_Config();		
		LORA_Join();
	}
}

void fdr_config(void)
{			
	lora_config.duty_cycle = LORA_DISABLE;
	lora_config.application_port=2;

	#if defined( REGION_CN470 )
	customize_config.set8channel=11;
	#endif
				
	#if defined( REGION_AS923 )	|| defined( REGION_AU915 )
	dwelltime=1;
	#endif
  			
	workmode=1;	
  power_5v_time=500;	
	inmode=2;		
	inmode2=2;		
	inmode3=2;
  GapValue=400.0;	
	APP_TX_DUTYCYCLE=1200000;
	REJOIN_TX_DUTYCYCLE=20;//min
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
	
	downlink_detect_switch=1;
	unconfirmed_uplink_change_to_confirmed_uplink_timeout=1440;
	downlink_detect_timeout=2880;
	confirmed_uplink_retransmission_nbtrials=7;
	confirmed_uplink_counter_retransmission_increment_switch=0;			
	LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=0;
	LinkADR_NbTrans_retransmission_nbtrials=1;	
	
	joined_flags=1;	
	Flash_Store_Config();
	Flash_Read_Config();
}

void region_printf(void)
{
#if defined( REGION_AS923 )
	#if defined(REGION_AS923_JAPAN)
	LOG_PRINTF(LL_DEBUG,"AS923-JP\n\r");
	band_string="AS923-JP";	
	#elif defined( AS923_2 )
	LOG_PRINTF(LL_DEBUG,"AS923-2\n\r");
	band_string="AS923-2";	
	#elif defined( AS923_3 )
	LOG_PRINTF(LL_DEBUG,"AS923-3\n\r");
	band_string="AS923-3";
	#elif defined( AS923_4 )
	LOG_PRINTF(LL_DEBUG,"AS923-4\n\r");	
	band_string="AS923-4";
	#else
  LOG_PRINTF(LL_DEBUG,"AS923\n\r");
	band_string="AS923";
	#endif
#elif defined( REGION_AU915 )
  LOG_PRINTF(LL_DEBUG,"AU915\n\r");
	band_string="AU915";
#elif defined( REGION_CN470 )
  LOG_PRINTF(LL_DEBUG,"CN470\n\r");
  band_string="CN470";
#elif defined( REGION_CN779 )
  LOG_PRINTF(LL_DEBUG,"CN779\n\r");
	band_string="CN779";
#elif defined( REGION_EU433 )
  LOG_PRINTF(LL_DEBUG,"EU433\n\r");
	band_string="EU433";
#elif defined( REGION_IN865 )
  LOG_PRINTF(LL_DEBUG,"IN865\n\r");
	band_string="IN865";
#elif defined( REGION_EU868 )
  LOG_PRINTF(LL_DEBUG,"EU868\n\r");
	band_string="EU868";
#elif defined( REGION_KR920 )
  LOG_PRINTF(LL_DEBUG,"KR920\n\r");
	band_string="KR920";
#elif defined( REGION_US915 )
  LOG_PRINTF(LL_DEBUG,"US915\n\r");
	band_string="US915";
#elif defined( REGION_RU864 )
  LOG_PRINTF(LL_DEBUG,"RU864\n\r");
	band_string="RU864";	
#elif defined( REGION_KZ865 )
  LOG_PRINTF(LL_DEBUG,"KZ865\n\r");
	band_string="KZ865";	
#elif defined( REGION_MA869 )
  LOG_PRINTF(LL_DEBUG,"MA869\n\r");	
	band_string="MA869";
#else
    #error "Please define a region in the compiler options."
#endif
}

void key_printf(void)
{
  LOG_PRINTF(LL_DEBUG,"DevEui= %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX8(lora_config.DevEui));
}

void LORA_Join( void)
{
	joined_finish=0;	
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
  
    /*if certification test are on going, application data is not sent*/
    if (certif_running() == true)
    {
      return LORA_ERROR;
    }
    
    if( LoRaMacQueryTxPossible( AppData->BuffSize, &txInfo ) != LORAMAC_STATUS_OK )
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
  LoRaMacTestSetDutyCycleOn((duty_cycle == LORA_ENABLE) ? 1 : 0);
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
	  
    while(print_isdone()==0);
	
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
		flash_erase_page(FLASH_USER_START_ADDR_KEY);
	  delay_ms(5);		
		if(flash_program_bytes(FLASH_USER_START_ADDR_KEY,store_key_in_flash,128)==ERRNO_FLASH_SEC_ERROR)
		{
			LOG_PRINTF(LL_DEBUG,"write key error\r\n");
		}
		__enable_irq();
}

void Flash_read_key(void)
{
	  uint8_t read_key_in_flash[128];
	
	  while(print_isdone()==0);
	
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
	
	while(print_isdone()==0);
	
	store_config_in_flash[0]=0x11;//FDR_status
	store_config_in_flash[1]=product_id;	
	store_config_in_flash[2]=current_fre_band;
	store_config_in_flash[3]=fire_version;
	
	MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[4]=mib.Param.AdrEnable;
	
	mib.Type = MIB_CHANNELS_TX_POWER;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[5]=mib.Param.ChannelsTxPower;
	
  store_config_in_flash[6]=lora_config.TxDatarate;
	
	store_config_in_flash[7]=lora_config.duty_cycle;	

	 mib.Type = MIB_PUBLIC_NETWORK;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[8]=mib.Param.EnablePublicNetwork;
	
	store_config_in_flash[9]=lora_config.otaa;

	mib.Type = MIB_DEVICE_CLASS;
	status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[10]=mib.Param.Class;//0:CLASS A
	
	store_config_in_flash[11]=lora_config.ReqAck;
	
	mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[12]=mib.Param.Rx2Channel.Frequency>>24 & 0xFF;
	store_config_in_flash[13]=mib.Param.Rx2Channel.Frequency>>16 & 0xFF;
	store_config_in_flash[14]=mib.Param.Rx2Channel.Frequency>>8 & 0xFF;
	store_config_in_flash[15]=mib.Param.Rx2Channel.Frequency & 0xFF;
	
	if(RX2DR_setting_status==1)
	{
		RX2DR_setting_status=0;
		mib.Type = MIB_RX2_CHANNEL;
		status = LoRaMacMibGetRequestConfirm(&mib);
		if(status!=LORAMAC_STATUS_OK)
		{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
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
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[17]=mib.Param.ReceiveDelay1>>8 & 0xFF;
	store_config_in_flash[18]=mib.Param.ReceiveDelay1 & 0xFF;
	
	mib.Type = MIB_RECEIVE_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[19]=mib.Param.ReceiveDelay2>>8 & 0xFF;
	store_config_in_flash[20]=mib.Param.ReceiveDelay2 & 0xFF;
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[21]=mib.Param.JoinAcceptDelay1>>8 & 0xFF;
	store_config_in_flash[22]=mib.Param.JoinAcceptDelay1 & 0xFF;
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}
	store_config_in_flash[23]=mib.Param.JoinAcceptDelay2>>8 & 0xFF;
	store_config_in_flash[24]=mib.Param.JoinAcceptDelay2 & 0xFF;
	
	mib.Type = MIB_NET_ID;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{LOG_PRINTF(LL_DEBUG,"LORAMAC STATUS ERROR\n\r");}

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
													 
	store_config_in_flash[48]=down_check;
													 
	store_config_in_flash[49]=decrypt_flag;
	
	store_config_in_flash[50]=symbtime1_value;
													 
	store_config_in_flash[51]=flag1;
													 
	store_config_in_flash[52]=symbtime2_value;
													 
	store_config_in_flash[53]=flag2;
	
	store_config_in_flash[54]=inmode;
													 
	store_config_in_flash[55]=REJOIN_TX_DUTYCYCLE>>8 & 0xFF;
	store_config_in_flash[56]=REJOIN_TX_DUTYCYCLE & 0xFF;
	
	store_config_in_flash[57]=response_level;
	store_config_in_flash[58]=dwelltime;
			
	store_config_in_flash[59]=downlink_detect_switch;
	
	store_config_in_flash[60]=downlink_detect_timeout>>8 & 0xFF;
	store_config_in_flash[61]=downlink_detect_timeout & 0xFF;
	store_config_in_flash[62]=confirmed_uplink_retransmission_nbtrials;
	store_config_in_flash[63]=confirmed_uplink_counter_retransmission_increment_switch;
	store_config_in_flash[64]=LinkADR_NbTrans_retransmission_nbtrials;
	store_config_in_flash[65]=LinkADR_NbTrans_uplink_counter_retransmission_increment_switch;
	store_config_in_flash[66]=unconfirmed_uplink_change_to_confirmed_uplink_timeout>>8 & 0xFF;
	store_config_in_flash[67]=unconfirmed_uplink_change_to_confirmed_uplink_timeout & 0xFF;
	
	store_config_in_flash[68]=mac_response_flag;
	
	store_config_in_flash[69]=power_5v_time>>8 & 0xFF;
	store_config_in_flash[70]=power_5v_time;
	
  store_config_in_flash[71]=workmode;
  store_config_in_flash[72]=inmode2;
  store_config_in_flash[73]=inmode3;

	store_config_in_flash[74]=(int)(GapValue*10)>>24;
	store_config_in_flash[75]=(int)(GapValue*10)>>16;
	store_config_in_flash[76]=(int)(GapValue*10)>>8;
	store_config_in_flash[77]=(int)(GapValue*10);
	
	store_config_in_flash[78]=pwm_timer;
	
	store_config_in_flash[79]=inmode_delay>>8 & 0xFF;
	store_config_in_flash[80]=inmode_delay;

	store_config_in_flash[81]=inmode2_delay>>8 & 0xFF;
	store_config_in_flash[82]=inmode2_delay;
	
	store_config_in_flash[83]=inmode3_delay>>8 & 0xFF;
	store_config_in_flash[84]=inmode3_delay;
	
	__disable_irq();	
	flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
	delay_ms(5);	
	if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,store_config_in_flash,128)==ERRNO_FLASH_SEC_ERROR)
	{
		LOG_PRINTF(LL_DEBUG,"write config error\r\n");
	}
	__enable_irq();
}
	
void Flash_Read_Config(void)
{
	uint8_t read_config_in_flash[128];
	
	while(print_isdone()==0);
	
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
	
	lora_config.TxDatarate=read_config_in_flash[6];
	
	if(read_config_in_flash[7]==0x01)
	{
		lora_config.duty_cycle=LORA_ENABLE;
	}
	else
	{
		lora_config.duty_cycle=LORA_DISABLE;
	}
	
	LoRaMacTestSetDutyCycleOn((lora_config.duty_cycle == LORA_ENABLE) ? 1 : 0);
		
	mib.Type = MIB_PUBLIC_NETWORK;
	mib.Param.EnablePublicNetwork=read_config_in_flash[8];
	LoRaMacMibSetRequestConfirm( &mib );
	
	if(read_config_in_flash[9]==0x01)
	{
		lora_config.otaa=LORA_ENABLE;
	}
	else
	{
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
											 
	down_check=read_config_in_flash[48];
													 
	decrypt_flag=read_config_in_flash[49];
	
	symbtime1_value=read_config_in_flash[50];
													 
	flag1=read_config_in_flash[51];
													 
	symbtime2_value=read_config_in_flash[52];
													 
	flag2=read_config_in_flash[53];
	
	inmode=read_config_in_flash[54];

	REJOIN_TX_DUTYCYCLE=read_config_in_flash[55]<<8 | read_config_in_flash[56];
	
	response_level=read_config_in_flash[57];
	
	dwelltime=read_config_in_flash[58];

	downlink_detect_switch=read_config_in_flash[59];
	
	downlink_detect_timeout=read_config_in_flash[60]<<8 | read_config_in_flash[61];
	
	confirmed_uplink_retransmission_nbtrials=read_config_in_flash[62];
	
	confirmed_uplink_counter_retransmission_increment_switch=read_config_in_flash[63];
	
	LinkADR_NbTrans_retransmission_nbtrials=read_config_in_flash[64];
	LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=read_config_in_flash[65];
	
	unconfirmed_uplink_change_to_confirmed_uplink_timeout=read_config_in_flash[66]<<8 | read_config_in_flash[67];

	mac_response_flag=read_config_in_flash[68];

	power_5v_time=read_config_in_flash[69]<<8 | read_config_in_flash[70];
	
	workmode=read_config_in_flash[71];
  inmode2=read_config_in_flash[72];
  inmode3=read_config_in_flash[73];
	
	GapValue=(float)((read_config_in_flash[74]<<24 | read_config_in_flash[75]<<16 | read_config_in_flash[76]<<8 | read_config_in_flash[77])/10.0);
	
	pwm_timer=read_config_in_flash[78];
	
	inmode_delay=read_config_in_flash[79]<<8 | read_config_in_flash[80];
	
	inmode2_delay=read_config_in_flash[81]<<8 | read_config_in_flash[82];
	
	inmode3_delay=read_config_in_flash[83]<<8 | read_config_in_flash[84];
}

uint8_t string_touint(void)
{
	char *p;	
	uint8_t chanum=0;	
	uint8_t versi;
	char version[8]="";
	p=AT_VERSION_STRING;
	
	while(*p++!='\0')
	{
  if(*p>='0'&&*p<='9')
   {
		 version[chanum++]=*p;
	 }		 
	}
	versi=atoi(version);
	
	return versi;
}

void new_firmware_update(void)
{
	fire_version = string_touint();	
	current_fre_band = Firm_FQ;
	
	if((fire_version!=firmware_ver_read_in_flash)||(current_fre_band!=fre_band_read_in_flash)||(product_id!=product_id_read_in_flash))
	{
		uint8_t status[128]={0};			
		memset(status, 0x00, 128);
		
		status[0]=0x12;	
		status[1]=product_id;
		status[2]=current_fre_band;	
		status[3]=fire_version;	
		while(print_isdone()==0);	
    __disable_irq();		
		flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
		delay_ms(5);		
		if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
		{
			LOG_PRINTF(LL_DEBUG,"write config error\r\n");
		}	
		__enable_irq();
		
		delay_ms(50);		
		system_reset();			
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
	system_get_chip_id(unique_id);
	
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
