/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "app_lorawan.h"
#include "gpio.h"
#include "flash_eraseprogram.h"
#include "lora_app.h"
#include "bsp.h"
#include <stdio.h>
#include <string.h>
#include "timer.h"
#include "radio.h"
#include "stm32_systime.h"
#include "mw_log_conf.h"
#include "iwdg.h"
#include "stm32_lpm.h"
#include "lora_command.h"
#include "sht3x.h"
#include "adc_if.h"
#include "version.h"
#include "flash_if.h"
#include "radio.h"
#include "sys_app.h"
#include "sys_app.h"
#include "zb25vq32.h"
#include "lora_app.h"

extern SysTime_t LastTxdoneTime;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*!
 * Defines the application data transmission duty cycle. 60s, value in [ms].
 */
uint32_t APP_TX_DUTYCYCLE=300000;//ms
static uint8_t DS18B20_ID_temp[1][8];
extern uint8_t DS18B20_ID[1][8];

void LoraStartdelay1(void);
static uint8_t status1;
static void Onexdelay1TimerEvent(void *context);
static TimerEvent_t exdelay1Timer;
/*
 *Poll Message Flag: 1: This message is a poll message from server, 0: means this is a normal uplink.
 *Data Record Flag: 1: this message is stored in Flash,0: This message is not stored in flash
 *UTC Set time OK: 1: Set time ok,0: N/A
 *UTC Set Time Request:1: Request server downlink UTC time, 0 : N/A
 */
bool Poll_Message_Flag=0,Data_Record_Flag=0,UTC_Accept=0,UTC_Request=1;
static uint32_t ServerSetTDC;
static uint32_t timestamp1,timestamp2;
uint8_t TDC_flag=0;
void read_sensor_data(uint32_t timestamp,uint8_t dev,uint16_t bat,float temp20,float hum20,float extsen,uint16_t rev);
static uint8_t bsp_sensor_data[16];
uint8_t Ext=0x01;//sensor type,0x01=ds18b20
uint8_t work_mode=0;
bool wakeup_flag=1;
uint16_t collection_interval=1;//1min

__IO bool ble_sleep_flags=0;
__IO bool ble_sleep_command=0;
__IO uint32_t write_address;
__IO uint32_t write_address10;
bool sleep_status=0;//AT+SLEEP
bool is_lora_joined=0;
__IO uint8_t is_time_to_send=0;
__IO bool is_activity_flag=0;
__IO bool is_time_to_IWDG_Refresh=0;
__IO uint16_t IWDG_Refresh_times=0;

uint32_t IWDG_Refresh_times_total_time=0;
uint32_t txdone_detection_timeout=0;

bool mac_response_flag=0;
bool exit_send_flag=0;
bool exti_flag=0;
bool exti_flag2;
bool level_status=0;
bool exit_fr=0;
extern bool debug_flags;
static uint8_t is_RetrievalTimer_running=0;

uint8_t is_PDTA_command=0;
uint8_t is_PLDTA_command=0;

extern uint8_t inmode,inmode2,inmode3;
extern uint16_t inmode_delay,inmode2_delay,inmode3_delay;
extern uint16_t LoRaMacDevNonce;
extern bool join_mothod;
extern uint32_t UpLinkCounter;
extern uint8_t dwelltime;
uint8_t atz_flags=0;
uint16_t REJOIN_TX_DUTYCYCLE=20;//min
bool join_network=0;
bool MAC_COMMAND_ANS_status=0;
uint8_t response_level=0;
bool joined_finish=0;
bool uplink_message_data_status=0;
bool is_there_data=0;
bool rejoin_status=0;
bool rejoin_keep_status=0;
bool is_time_to_linkcheck=0;
bool is_time_to_rejoin=0;
bool JoinReq_NbTrails_over=0;
bool unconfirmed_downlink_data_ans_status=0,confirmed_downlink_data_ans_status=0;
float collection_temp_sht20=0;
//float collection_temp_sht20_record=10;
float collection_temp_exttemp=0,collection_temp_exttemp_record=10;
bool is_time_to_temp_comp=0;
bool is_time_to_exttemp_cmp=0;
bool beyond_status=0;
bool is_time_to_send_beyond_data;
static void sht20_temp_comp(void);
static void ext_temp_comp(void);
short comp_temp1_sht20=-40,comp_temp2_sht20=125;
short comp_temp1_exttemp=0,comp_temp2_exttemp=0;
uint8_t temp_alarm_switch_for_work_mode3=0;
uint8_t is_time_to_send_temp_alarm_for_work_mode3=0;
static float last_uplink_temp=-100.0;
uint8_t temp_change;
uint8_t collection_second;
bool is_time_to_send_ds18b20_id=0;//pid=1
__IO uint8_t EnterLowPowerStopModeStatus=0,EnterLowPowerStopMode_error_times=0;
extern uint8_t lora_packet_send_complete_status;
uint8_t ack_data_store_status=0;
uint8_t is_lora_joined_keep_status=0;
bool is_time_to_LED_alarm=0;
uint8_t led_alarm_switch=0;
uint8_t collection_count_for_work_mode3=0;

uint8_t temp_record_for_work_mode3[239];
uint8_t temp_record_for_work_mode3_size=0;

uint8_t external_sensor_type_for_work_mode3=0;
uint16_t external_sensor_sample_interval_for_work_mode3=60;
uint8_t external_sensor_sample_sum_for_work_mode3=20;

extern bool ser_ack_record,ser_ack_record2;
uint32_t no_recv_ack_addr_record[22]={0};//max=242/11

bool send_no_recv_ack_data_keep_status=0;
uint8_t ack_status_in_flash_set[8]={0,0,0,0,0,0,0x55,0x66};

uint8_t currentLeapSecond=0;
uint8_t time_synchronization_method=0;
uint8_t time_synchronization_interval=0;

uint8_t downlink_detect_switch=0;
uint16_t downlink_detect_timeout=0;
uint8_t downlink_received_status=0;
uint8_t LoRaMacState_error_times=0;

uint8_t confirmed_uplink_counter_retransmission_increment_switch=0;
uint8_t confirmed_uplink_retransmission_nbtrials=0;

uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=0;
uint8_t LinkADR_NbTrans_retransmission_nbtrials=0;

uint8_t unconfirmed_uplink_change_to_confirmed_uplink_status=0;
uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout=0;

bool payload_oversize=0;

//uint8_t Ext4_record_status;
//bool Ext4_is_time_to_comp_record_status;	
uint8_t pid_flag;
uint8_t pnackmd_switch;

#define BCD_TO_HEX2(bcd) ((((bcd)>>4)*10)+((bcd)&0x0F))

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            200
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           242
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

uint8_t user_key_exti_flag=0;

uint32_t batteryLevel_mV;
uint8_t user_key_duration=0;
uint8_t payloadlens=0;
void user_key_event(void);

extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;
extern bool rx2_flags;
extern uint32_t LoRaMacState;
extern uint8_t exitintmode;
extern uint8_t exitmode;
extern uint16_t power_time;
uint32_t count;
extern uint8_t s31f_ext;
extern bool connect_flags_temp;
extern bool FDR_status;
extern bool message_flags;
extern uint8_t valid_flag;
extern uint16_t valid_time;
uint8_t extpower_logic;

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* LoRa endNode send request*/
static void Send( void );
static void Send_device_status(void);

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

static void StartIWDGRefresh(TxEventType_t EventType);
static void LoraStartRejoin(TxEventType_t EventType);
static void StartRetrieval(TxEventType_t EventType);
static void StartCalibrationUTCtime(TxEventType_t EventType);
static void Startsht20tempcomptime(TxEventType_t EventType);
void Startexttempcomptime(TxEventType_t EventType);
static void StartDS18B20ID(TxEventType_t EventType);
static void StartDownlinkDetect(TxEventType_t EventType);
static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(TxEventType_t EventType);
static void StartReplyNoRecvAckData(TxEventType_t EventType);
static void StartRedLEDAlarm1(TxEventType_t EventType);
static void LoraStartCheckBLE(void);

TimerEvent_t CheckBLETimesTimer;
TimerEvent_t ValidtimeTimer;
TimerEvent_t TxTimer;
TimerEvent_t downlinkLedTimer;
TimerEvent_t downlinkLedTimer;
TimerEvent_t NetworkJoinedLedTimer;
TimerEvent_t PressButtonTimesLedTimer;
TimerEvent_t PressButtonTimeoutTimer;
TimerEvent_t IWDGRefreshTimer;
TimerEvent_t ReJoinTimer;
TimerEvent_t RetrievalTimer;
TimerEvent_t CalibrationUTCTimer;
TimerEvent_t sht20tempcompTimer;
TimerEvent_t exttempcompTimer;
TimerEvent_t DS18B20IDTimer;
TimerEvent_t DownlinkDetectTimeoutTimer;
TimerEvent_t UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer;
TimerEvent_t ReplyNoRecvAckDataTimer;
TimerEvent_t RedLEDAlarmTimer1;
TimerEvent_t RedLEDAlarmTimer2;
TimerEvent_t DelayresetTimeoutTimer;

bool is_time_to_find_timestamp1_on_flash=0;
bool is_time_to_reply_retrieval_request=0;
bool is_time_to_send_retrieval_data=0;
__IO uint32_t find_timestamp_addr;
__IO uint32_t find_ack_status_addr;

uint32_t find_timestamp_addr_record,find_ack_status_addr_record;
bool is_time_to_send_no_recv_ack_data=0;

uint8_t retrieval_uplink_time=0;
bool is_time_to_reply_downlink=0;
bool is_alarm_setting=0;
static uint8_t downlink_command_buffer[15];
static uint8_t downlink_command_buffersize=0;

bool joined_led=0,join_led_timeout_status=0;
uint8_t press_button_times=0;//Press the button times in a row fast
uint8_t OnPressButtonTimeout_status=0;
extern TimerEvent_t TxDelayedTimer;

extern void printf_joinmessage(void);

__IO uint8_t IsRxCmdReceived=0;

/* tx timer callback function*/
static void OnTxTimerEvent( void *context );

void OndownlinkLedEvent(void *context);
void OnNetworkJoinedLedEvent(void *context);
void OnPressButtonTimesLedEvent(void *context);
void OnPressButtonTimeoutEvent(void *context);
static void OnIWDGRefreshTimeoutEvent(void *context);
static void OnReJoinTimerEvent( void *context );
static void OnRetrievalTimeoutEvent( void *context );
static void find_timestamp1_on_flash(void);//find the address where timestamp is stored
static void Reply_to_retrieval_request(void);
static void OnCalibrationUTCTimeoutEvent( void *context );
static void onsht20tempcompTimeoutEvent( void *context );
static void onexttempcompTimeoutEvent( void *context );
static void OnDownlinkDetectTimeoutEvent( void *context );
static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void *context );
void OnDS18B20IDTimeoutEvent(void *context);
bool comp_ds18b20_id(void);
static void Reply_to_not_recv_ack_data(void);
static void OnReplyNoRecvAckDataTimeoutEvent( void *context );
static void OnRedLEDAlarm1TimeoutEvent( void *context );
static void OnRedLEDAlarm2TimeoutEvent( void *context );
static void OnValidtimeTimeoutEvent( void *context );
static void OnDelayresetTimeoutEvent( void *context );
void OnCheckBLETimesEvent(void *context);

uint8_t HW_GetBatteryLevel( void ); 
uint16_t HW_GetTemperatureLevel( void );	
void HW_GetUniqueId( uint8_t *id );
uint32_t HW_GetRandomSeed( void );
void board_init();
void uart_log_init(uint32_t baudrate);
void send_exti(void);

static void CmdProcessNotify(void);
	
/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetTemperatureLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LORA_RxData,
                                               LORA_HasJoined,
                                               LORA_ConfirmClass};

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

	SystemApp_Init();
	
	iwdg_init();
	
	CMD_Init(CmdProcessNotify);

  if(FLASH_IF_Init(NULL) != FLASH_IF_OK)
  {
    Error_Handler();
  }

  if(__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
  {
		*((uint8_t *)(0x2000F00A))=0x11;//0x11:Activation Mode, 0xA8:sleep
	
		for(uint16_t i=0;i<3328;i++)
		{
			*((unsigned int *)(SRAM_SENSOR_DATA_STORE_ACK_START_ADDR+i)) =0xEE;
		}
  }	
	
	Flash_read_key();	
	Flash_Read_Config();
	
	uint8_t ZB25VQ32_devid[3];
  uint8_t ZB25VQ32_data[256];
	
	MX_SPI1_Init();
	ZB25VQ32_ReleaseDeepPowerDown();
	ZB25VQ32_ReadID(ZB25VQ32_devid);
//	APP_PPRINTF("ZB25VQ32 devid = %02x,%02x,%02x\r\n", ZB25VQ32_devid[0],ZB25VQ32_devid[1],ZB25VQ32_devid[2]);

	for(uint32_t i = 0; i<256; i++)
	{
    ZB25VQ32_data[i] = 0x12;
	}
	
	ZB25VQ32_WriteEnable();
	uint8_t ret=0;

//////	ZB25VQ32_ReadStatusRegister(devid);
//////	APP_PPRINTF("StatusRegister = %02x\r\n", devid[0]);
	
//  ZB25VQ32_SectorErase(0);//0,1,2...1023
//  	
//	HAL_Delay(100);
//	
//	ZB25VQ32_WriteDisable();
//	HAL_Delay(10);
//	ZB25VQ32_WriteEnable();
//	ret = ZB25VQ32_PageProgram(ZB25VQ32_data,0x00 + 256*2,256);
//	APP_PPRINTF("ret = %02x\r\n", ret);
//	HAL_Delay(10);

//	
//	
//	
//	for(uint32_t i = 0; i<256; i++)
//	{
//    ZB25VQ32_data[i] = 0;
//	}
//	
//  ret = ZB25VQ32_FastRead(ZB25VQ32_data,0x00 + 256*2,256);
//	APP_PPRINTF("ret = %02x\r\n", ret);
//	
//  HAL_Delay(10);
//	for(uint32_t i = 0; i<256; i++)
//	{
//		APP_PPRINTF("%02x ", ZB25VQ32_data[i]);
//		if(i%16==0)
//		{
//			APP_PPRINTF("\r\n");
//		}
//	}

	ZB25VQ32_WriteDisable();
	ZB25VQ32_EnterDeepPowerDown();

	LoraStartCheckBLE();
	BSP_sensor_Init();

	display_sht31_message();
	
	write_address=FLASH_SENSOR_DATA_START_ADDR;
	write_address=find_addr_first_after_systemreset(write_address);

	write_address10=FLASH_LoRaMacDevNonce_START_ADDR;
	write_address10=find_LoRaMacDevNonce_addr_first_after_systemreset(write_address10);	
	
//	MW_LOG(TS_OFF, VLEVEL_M, "W10:%08X,%d\r\n",write_address10,join_mothod);
  if(join_mothod==1)
	{			
		read_lastet_count(write_address10,&LoRaMacDevNonce);
	  if(LoRaMacDevNonce==0)
		{
			LoRaMacDevNonce=1;
		}
	}

	if(*((uint8_t *)(0x2000FE0B))==0xAA)
	{
		if(join_mothod==0)
		{
			UpLinkCounter=(*((uint8_t *)(0x2000FE06)))<<24|(*((uint8_t *)(0x2000FE07)))<<16|(*((uint8_t *)(0x2000FE08)))<<8|(*((uint8_t *)(0x2000FE09)));
		}
	}
				
	StartIWDGRefresh(TX_ON_EVENT);		
	
	if(debug_flags==1)
	{
		MW_LOG(TS_OFF, VLEVEL_M, "dragino_CCU6_ota\r\n");
	}
	
//	MW_LOG(TS_OFF, VLEVEL_M, "sleep_status  %d %d \r\n",sleep_status,*((uint8_t *)(0x2000F00A)));
	
	/* Configure the Lora Stack*/
	LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
	
	TimerInit( &ReplyNoRecvAckDataTimer, OnReplyNoRecvAckDataTimeoutEvent );
	TimerInit( &sht20tempcompTimer, onsht20tempcompTimeoutEvent );
	TimerInit( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer, UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent );
	TimerInit( &DownlinkDetectTimeoutTimer, OnDownlinkDetectTimeoutEvent );			
	TimerInit( &RetrievalTimer, OnRetrievalTimeoutEvent );
	TimerInit( &downlinkLedTimer, OndownlinkLedEvent );
	TimerInit( &RedLEDAlarmTimer1, OnRedLEDAlarm1TimeoutEvent );
	TimerInit( &RedLEDAlarmTimer2, OnRedLEDAlarm2TimeoutEvent );
	TimerInit( &ValidtimeTimer, OnValidtimeTimeoutEvent );
	TimerInit( &TxTimer, OnTxTimerEvent );
  LoraStartdelay1();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 			
		CMD_Process();
		
	  send_exti();			
		
		if(ble_sleep_command==1)
		{			
			HAL_Delay(50);					
			MW_LOG(TS_OFF, VLEVEL_M,"AT+PWRM2\r\n");
			HAL_Delay(100);										
			ble_sleep_command=0;
			ble_sleep_flags=1;
		}
								
		if(joined_led==1)
		{
			joined_led=0;
      join_network=1;
			HAL_GPIO_WritePin(LED_RGB_PORT, LED_RED_PIN, 0);
			HAL_GPIO_WritePin(LED_RGB_PORT, LED_GREEN_PIN, 0);
			HAL_GPIO_WritePin(LED_RGB_PORT, LED_BLUE_PIN, 0);
			
			TimerInit( &NetworkJoinedLedTimer, OnNetworkJoinedLedEvent );
			TimerSetValue( &NetworkJoinedLedTimer, 5000);
			HAL_GPIO_WritePin(LED_RGB_PORT, LED_GREEN_PIN, 1);
			TimerStart( &NetworkJoinedLedTimer );
		}
		
		if(is_lora_joined==1)
		{
			MW_LOG(TS_OFF, VLEVEL_M, "JOINED\n\r");
      rejoin_keep_status=0;
	
			if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
			{
				printf_joinmessage();
			}			
			
      TimerStop(&ReJoinTimer);
			is_lora_joined=0;
			
			joined_led=1;
		}
		
    if(join_led_timeout_status==1)	
		{
      join_led_timeout_status=0;	
						
			StartCalibrationUTCtime(TX_ON_EVENT);
			HAL_Delay(5);
			
			if(downlink_detect_switch==1)
			{
				if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
				{
					StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(TX_ON_EVENT);
					HAL_Delay(5);
				}
				StartDownlinkDetect(TX_ON_EVENT);
				HAL_Delay(5);
			}
			
		 LoraStartTx( TX_ON_TIMER);   		
		}
		
		if((is_time_to_temp_comp==1) && (comp_temp1_sht20!=-40)&&(comp_temp2_sht20!=125))
		{
			is_time_to_temp_comp=0;
			sht20_temp_comp();
		}

		if((is_time_to_exttemp_cmp==1) && (work_mode>=1))
		{
			is_time_to_exttemp_cmp=0;
			if((Ext==0x01)||(Ext==0x02)||(Ext==0x09)||(Ext==10)||(Ext==11)||work_mode==3)
			{
				ext_temp_comp();
			}
		}
		
		if(is_time_to_send_beyond_data==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				is_time_to_send_beyond_data=0;
				Send();
			}
		}
		
		if(time_synchronization_method==1)
		{
			if(payload_oversize==1)
			{
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					LoRaMacState_error_times=0;
					MlmeReq_t mlmeReq;
					mlmeReq.Type = MLME_DEVICE_TIME;
					LoRaMacMlmeRequest( &mlmeReq );
					payload_oversize=0;
					
					#if defined ( REGION_US915 ) || defined ( REGION_AU915 ) || defined ( REGION_AS923 )
					UTC_Request=0;//only send once 
					#endif
					
					AppData.Buff[0]=0x00;
					AppData.BuffSize=1;
					AppData.Port = 4;
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				}
			}
	  }

		if(atz_flags==1)
		{
			LoRaMacState_error_times=0;
			HAL_Delay(500);
			AppData.Buff[0]=0x11;
			AppData.BuffSize=1;
			AppData.Port = 4;
			LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			atz_flags++;
		}
		else if((atz_flags==2)&&(( LoRaMacState & 0x00000001 ) != 0x00000001))
		{
			NVIC_SystemReset();
		}
		#ifdef REGION_US915
		if(MAC_COMMAND_ANS_status==1)
		{		
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
          LoRaMacState_error_times=0;          
					MibRequestConfirm_t mib;
			
			    mib.Type=MIB_CHANNELS_DATARATE;
			    LoRaMacMibGetRequestConfirm(&mib);										
					
				  if(mib.Param.ChannelsDatarate==0)
			    {
						MAC_COMMAND_ANS_status=0;
						AppData.Buff[0]=0x00;
						AppData.BuffSize=1;
	          AppData.Port = 4;
	          LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				  }
				}		  
		}
		#elif defined( REGION_AS923 )	|| defined( REGION_AU915 )		
		if((MAC_COMMAND_ANS_status==1)&&(dwelltime==1))
		{		
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{ 
		    LoRaMacState_error_times=0;
				MibRequestConfirm_t mib;
		
				mib.Type=MIB_CHANNELS_DATARATE;
				LoRaMacMibGetRequestConfirm(&mib);
				
				MAC_COMMAND_ANS_status=0;
				
				if(mib.Param.ChannelsDatarate==2)
				{
					AppData.Buff[0]=0x00;
					AppData.BuffSize=1;
					AppData.Port = 4;							
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);					
				}
			}		  
		}
		#endif
		
		if((MAC_COMMAND_ANS_status==1 && response_level==3) 
		|| (unconfirmed_downlink_data_ans_status==1 && response_level==1 && is_there_data==1 ) 
		|| (confirmed_downlink_data_ans_status==1 && response_level==2 && is_there_data==1 )
		||(((MAC_COMMAND_ANS_status==1)||(confirmed_downlink_data_ans_status==1&&is_there_data==1))&&(response_level==4)))
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				MAC_COMMAND_ANS_status=0;
				unconfirmed_downlink_data_ans_status=0;
				confirmed_downlink_data_ans_status=0;
				is_there_data=0;
				AppData.Buff[0]=0x00;
				AppData.BuffSize=1;
				AppData.Port = 4;
	      LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			}
		}
		
		if((uplink_message_data_status==1)&&(( LoRaMacState & 0x00000001 ) != 0x00000001) &&(( LoRaMacState & 0x00000010 ) != 0x00000010))
	  {		
			if(time_synchronization_method==1 && payload_oversize==0)//0 via downlink,1 via devicetimereq mac command
			{
				if(UTC_Request==1)
				{						
					MlmeReq_t mlmeReq;
					mlmeReq.Type = MLME_DEVICE_TIME;
					LoRaMacMlmeRequest( &mlmeReq );
				 }
			}				
			Send_device_status();			
			uplink_message_data_status=0;
		}	
					
		if(is_time_to_send==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				if(time_synchronization_method==1 && payload_oversize==0)//0 via downlink,1 via devicetimereq mac command
				{
					if(UTC_Request==1)
					{						
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_DEVICE_TIME;
						LoRaMacMlmeRequest( &mlmeReq );
				  }
				}				
				Send();
				is_time_to_send=0;
      }
		}
		
		if(is_time_to_reply_downlink==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				is_time_to_reply_downlink=0;
				
				for(int i=0;i<downlink_command_buffersize;i++)
				{
					AppData.Buff[i]=downlink_command_buffer[i];
				}
				
				AppData.BuffSize=downlink_command_buffersize;
				if(is_alarm_setting==1)
				{
					is_alarm_setting=0;
					AppData.Port = 8;
				}
				else
				{
					AppData.Port = 12;
				}
	      LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			}
		}
		
		find_timestamp1_on_flash();
		
		Reply_to_retrieval_request();
		
		if(is_time_to_rejoin==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				unconfirmed_uplink_change_to_confirmed_uplink_status=0;
			  is_time_to_rejoin=0;
			  LORA_Join();
			}
		}
		
		if(JoinReq_NbTrails_over==1)
		{
			JoinReq_NbTrails_over=0;
			
			rejoin_keep_status=1;
			
			if(REJOIN_TX_DUTYCYCLE==0)
			{
				REJOIN_TX_DUTYCYCLE=20;
			}
			LoraStartRejoin(TX_ON_EVENT);
		}
		
		if(rejoin_status==1 && sleep_status==0)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				rejoin_keep_status=1;
				LoRaMacState_error_times=0;
				unconfirmed_uplink_change_to_confirmed_uplink_status=0;
				
				TimerStop( &CalibrationUTCTimer);
				TimerStop( &sht20tempcompTimer);
				TimerStop( &exttempcompTimer);
				TimerStop( &DownlinkDetectTimeoutTimer);
				TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
			  LORA_Join();
			}
		}
	
		if(downlink_received_status==1 && LORA_JoinStatus () == LORA_SET && downlink_detect_switch==1 && downlink_detect_timeout>0 && unconfirmed_uplink_change_to_confirmed_uplink_timeout>0 && sleep_status==0)
		{
			downlink_received_status=0;
			unconfirmed_uplink_change_to_confirmed_uplink_status=0;
			TimerSetValue( &DownlinkDetectTimeoutTimer,  downlink_detect_timeout*60000); 
      TimerStart( &DownlinkDetectTimeoutTimer);
			
			if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
			{
				TimerSetValue( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,  unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
				TimerStart( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
			}
		}
		
		if(LoRaMacState_error_times>=5 && sleep_status==0)
		{
			LoRaMacState_error_times=0;
			HAL_Delay(100);
			NVIC_SystemReset();
		}
		
		if(is_time_to_IWDG_Refresh==1)
		{
			is_time_to_IWDG_Refresh=0;
			iwdg_reload();

			IWDG_Refresh_times_total_time = IWDG_Refresh_times*18;
			
			txdone_detection_timeout = (3*APP_TX_DUTYCYCLE)/1000;
			
			if((IWDG_Refresh_times_total_time>txdone_detection_timeout) && sleep_status==0 && FDR_status==0)
			{
			  if((LORA_JoinStatus () != LORA_SET)&&(IWDG_Refresh_times_total_time<(3*REJOIN_TX_DUTYCYCLE*60)))
				{
							
				}
				else
				{
					IWDG_Refresh_times=0;
					NVIC_SystemReset();
				}
			}
			
			IWDG_Refresh_times++;
			
			if(EnterLowPowerStopModeStatus==0)
			{
				EnterLowPowerStopMode_error_times++;
				if(EnterLowPowerStopMode_error_times>=10)
				{
					EnterLowPowerStopMode_error_times=0;
				}					
			}			
		}
		
		user_key_event();

    UTIL_LPM_EnterLowPower();
  }
	
  /* USER CODE END 3 */
}

static void LORA_HasJoined( void )
{	
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );

	is_lora_joined=1;
	
	is_lora_joined_keep_status=1;
}

static void Send( void )
{
	is_there_data=0;  
	
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }
	
	if(debug_flags==1)
	{
		message_flags=1;
	}	
	
  sensor_t bsp_sensor_data_buff;
	BSP_sensor_Read(&bsp_sensor_data_buff,message_flags);
	message_flags=0;
	
	uint8_t i = 0;

	AppData.Port = lora_config_application_port_get();	

	AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
	AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
	AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
	AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
	AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
	AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4);

	AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pa8<<7)|(bsp_sensor_data_buff.in1<<1)|(exti_flag&0x01);
	
//	if(bh1750flags==1)
//	{
//		AppData.Buff[i++] =(bsp_sensor_data_buff.illuminance)>>8;      
//		AppData.Buff[i++] =(bsp_sensor_data_buff.illuminance);
//		AppData.Buff[i++] = 0x00;   
//		AppData.Buff[i++] = 0x00;				
//	}	
//	else
//	{
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10)>>8;      
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10);
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10)>>8;   
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10);
//	}
	
  AppData.BuffSize = i;
	payloadlens=i;
	exti_flag=0;
	
  if(unconfirmed_uplink_change_to_confirmed_uplink_status==1)
  {
		LORA_send( &AppData, LORAWAN_CONFIRMED_MSG);
	}
	else
	{
		LORA_send( &AppData, lora_config_reqack_get());
	}
}

static void Send_device_status(void)
{
	uint8_t freq_band;
	uint8_t sub_band;	
	uint16_t version;
	
	is_there_data=0;		
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }

	#if defined( REGION_EU868 )
		freq_band=0x01;
	#elif defined( REGION_US915 )
		freq_band=0x02;
	#elif defined( REGION_IN865 )
		freq_band=0x03;
	#elif defined( REGION_AU915 )
		freq_band=0x04;
	#elif defined( REGION_KZ865 )
		freq_band=0x05;
	#elif defined( REGION_RU864 )
		freq_band=0x06;
	#elif defined( REGION_AS923 )
	  #if defined AS923_1
		freq_band=0x08;
	  #elif defined AS923_2
	  freq_band=0x09;
	  #elif defined AS923_3
	  freq_band=0x0A;
	  #elif defined AS923_4
	  freq_band=0x0F;
		#else
		freq_band=0x07;
		#endif
	#elif defined( REGION_MA869 )			
		freq_band=0x0B;
	#elif defined( REGION_KR920 )			
	  freq_band=0x0C;
	#elif defined( REGION_EU433 )
	  freq_band=0x0D;
	#elif defined( REGION_CN470 )
	  freq_band=0x0E;
	#else
    freq_band=0x00;
	#endif
	
	#if defined( REGION_US915 )	|| defined( REGION_AU915 ) || defined( REGION_CN470 )
	sub_band = customize_set8channel_get();
	#else
	sub_band = 0xff;
	#endif
	
	version=SN50v3_FIRMWARE_VERSION;		

	uint16_t battery_mv;
  battery_mv=battery_voltage_measurement();
  if(battery_mv>3750)
	{
	  battery_mv=battery_voltage_measurement();
	}	
	
	uint32_t i = 0;

  AppData.Port = 5;
	
	if(Ext==0x0e)
	{
		AppData.Buff[i++] = 0x1A;
	}
	else
	{
		AppData.Buff[i++] = 0x0B;
	}
	
	AppData.Buff[i++] = (version>>8)&0xff;
	AppData.Buff[i++] =  version&0xff;
	
	AppData.Buff[i++] = freq_band;
	AppData.Buff[i++] = sub_band;

  AppData.Buff[i++]= (battery_mv >>8);
	AppData.Buff[i++]=  battery_mv;
	
	AppData.BuffSize = i;
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);		   
}

static void LORA_RxData( lora_AppData_t *AppData )
{
	uint8_t downlink_config_store_in_flash=0;
  is_there_data=1;
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
	
  TimerSetValue(  &downlinkLedTimer, 200);
  HAL_GPIO_WritePin(LED_RGB_PORT, LED_RED_PIN, 1);
	HAL_GPIO_WritePin(LED_RGB_PORT, LED_BLUE_PIN, 1);

  TimerStart( &downlinkLedTimer );
	
	if(AppData->BuffSize<8)
	{
		MW_LOG(TS_OFF, VLEVEL_M, "Receive data\n\r");
		MW_LOG(TS_OFF, VLEVEL_M, "%d:",AppData->Port);
		 for (int i = 0; i < AppData->BuffSize; i++)
		{
			MW_LOG(TS_OFF, VLEVEL_M, "%02x", AppData->Buff[i]);
		}
		MW_LOG(TS_OFF, VLEVEL_M, "\n\r");
  }
  else
	{
		MW_LOG(TS_OFF, VLEVEL_M, "Run AT+RECVB=? to see detail\r\n");			
	}
      switch(AppData->Buff[0] & 0xff)
      {			
				case 1:
				{
					if( AppData->BuffSize == 4 )
					{
					    ServerSetTDC=( AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3] );//S
							if(ServerSetTDC<5)
							{
							  MW_LOG(TS_OFF, VLEVEL_M, "TDC setting needs to be high than 4s\n\r");
							}
							else
							{
							  TDC_flag=1;
			          APP_TX_DUTYCYCLE=ServerSetTDC*1000;
							}						
					}
					break;
				}
				
        case 4:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]==0xFF)  //---->ATZ
						{
							atz_flags=1;		
						}
						else if(AppData->Buff[1]==0xFE)  //---->AT+FDR
						{	
							uint8_t status[128]={0};
							memset(status, 0x00, 128);

							status[0]=0x12;
							__disable_irq();
//							flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
							FLASH_IF_Erase((void *)FLASH_USER_START_ADDR_CONFIG, FLASH_PAGE_SIZE);
							HAL_Delay(5);
//							if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
							if(FLASH_IF_Write((void *)FLASH_USER_START_ADDR_CONFIG, status, 128) ==FLASH_IF_ERROR)
							{
								MW_LOG(TS_OFF, VLEVEL_M, "write config error\r\n");
							}
							__enable_irq();
					
							atz_flags=1;													
						}
					}
					else if((AppData->BuffSize == 4)&&(AppData->Buff[1]==0xFF))
					{
						uint16_t dltime= (AppData->Buff[2]<<8) | AppData->Buff[3];
						TimerInit( &DelayresetTimeoutTimer, OnDelayresetTimeoutEvent );
						TimerSetValue( &DelayresetTimeoutTimer,  dltime*1000 );
						TimerStart( &DelayresetTimeoutTimer );				
					}
					break;
				}	
				
        case 5:
				{
					is_time_to_reply_downlink=1;
					
					if( AppData->BuffSize == 4 && pnackmd_switch==0 )
					{
						if(AppData->Buff[1]<2)
						{
							if(AppData->Buff[1]==0x01)
							{
								lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
								confirmed_uplink_retransmission_nbtrials=AppData->Buff[2];
		            confirmed_uplink_counter_retransmission_increment_switch=AppData->Buff[3];
							}
							else if(AppData->Buff[1]==0x00)
							{
								confirmed_uplink_retransmission_nbtrials=AppData->Buff[2];
		            confirmed_uplink_counter_retransmission_increment_switch=AppData->Buff[3];
								lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
							}
							downlink_config_store_in_flash=1;
					  }
				  }else if(AppData->BuffSize == 2 && pnackmd_switch==0)
					{
            if(AppData->Buff[1]==0x00)
						{
							lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
						}
						else
						{
							lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
						}
						
						confirmed_uplink_counter_retransmission_increment_switch=0;
						confirmed_uplink_retransmission_nbtrials=7;
						downlink_config_store_in_flash=1;
					}else
					{
						is_time_to_reply_downlink=0;
						AppData->BuffSize=0;
					}
					
					downlink_command_buffersize=AppData->BuffSize;
					for(int i=0;i<AppData->BuffSize;i++)
					{
						downlink_command_buffer[i]=AppData->Buff[i];
					}
					
					break;
				}
			
   case 0x06:
   {
			if(( AppData->BuffSize == 4 )||( AppData->BuffSize == 6 ))
			{
			  if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]<=0x04))   		  //---->AT+INTMOD1
				{
					if( AppData->BuffSize == 6)
					{
						inmode_delay=AppData->Buff[4]<<8 | AppData->Buff[5];
					}
					inmode=AppData->Buff[3];
					GPIO_EXTI8_IoInit(inmode);
					downlink_config_store_in_flash=1;		
				}			
//			  else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x01)&&(AppData->Buff[3]<=0x04))   		  //---->AT+INTMOD2
//				{
//					if( AppData->BuffSize == 6)
//					{
//						inmode2_delay=AppData->Buff[4]<<8 | AppData->Buff[5];
//					}
//					inmode2=AppData->Buff[3];
//					GPIO_EXTI4_IoInit(inmode);
//					downlink_config_store_in_flash=1;
//					rxpr_flags=1;		
//				}		
//			  if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x02)&&(AppData->Buff[3]<=0x04))   		  //---->AT+INTMOD3
//				{
//					if( AppData->BuffSize == 6)
//					{
//						inmode3_delay=AppData->Buff[4]<<8 | AppData->Buff[5];
//					}
//					inmode3=AppData->Buff[3];
//					GPIO_EXTI15_IoInit(inmode);
//					downlink_config_store_in_flash=1;
//					rxpr_flags=1;		
//				}						
		  }
			break;	
	  }	
	 
				case 7:
				{
					if( AppData->BuffSize == 2 )
					{
						#if defined( REGION_US915 )	|| defined( REGION_AU915 )
						if(AppData->Buff[1]<9)
						{
							customize_set8channel_set(AppData->Buff[1]);
							downlink_config_store_in_flash=1;
							atz_flags=1;
						}
						#elif defined( REGION_CN470 )
						if(AppData->Buff[1]<13)
						{
							customize_set8channel_set(AppData->Buff[1]);
							downlink_config_store_in_flash=1;
							atz_flags=1;
						}
						#endif
				  }
					break;
				}
				
				case 0x20:			
				{
					if( AppData->BuffSize == 2 )
					{		
						if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))    
						{
							if(AppData->Buff[1]==0x01)       //---->AT+NJM=1
							{
								lora_config_otaa_set(LORA_ENABLE);
							}
							else                             //---->AT+NJM=0
							{
								lora_config_otaa_set(LORA_DISABLE);							
							}
							downlink_config_store_in_flash=1;
							atz_flags=1;					
						}						 
					}
					break;				
				}	
				
				case 0x21:
				{
					if( (AppData->BuffSize == 2) && (AppData->Buff[1]<=4) )
					{
						response_level=( AppData->Buff[1] );//0~4					//---->AT+RPL
						downlink_config_store_in_flash=1;							
					}
					break;
				}		
				
				case 0x22:			
				{
					MibRequestConfirm_t mib;
					if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))   //---->AT+ADR=1
					{		
						mib.Type = MIB_ADR;
						mib.Param.AdrEnable =AppData->Buff[1];
						LoRaMacMibSetRequestConfirm( &mib );					
						downlink_config_store_in_flash=1;							
					}
					else if((AppData->BuffSize == 4 )&&(AppData->Buff[1]==0x00))   //---->AT+ADR=0
					{
						uint8_t downlink_data_rate=AppData->Buff[2];
						mib.Type = MIB_ADR;					
						mib.Param.AdrEnable = AppData->Buff[1];
						LoRaMacMibSetRequestConfirm( &mib );	
						
						#if defined(REGION_US915)
						if(downlink_data_rate>3)
						{
							downlink_data_rate=3;   
						}
						#elif defined(REGION_AS923) || defined(REGION_AU915)
						if(dwelltime==1)
						{
							if(downlink_data_rate>5)
							{
								downlink_data_rate=5;
							}else if(downlink_data_rate<2)
							{
								downlink_data_rate=2;
							}
						}
						#else
						if(downlink_data_rate>5)
						{
							downlink_data_rate=5;
						}
						#endif	
						
						lora_config_tx_datarate_set(downlink_data_rate) ;
						
						if(AppData->Buff[3]!=0xff)                //---->AT+TXP
						{
							mib.Type = MIB_CHANNELS_TX_POWER;						
							mib.Param.ChannelsTxPower=AppData->Buff[3];
							LoRaMacMibSetRequestConfirm( &mib );							
						}				
						downlink_config_store_in_flash=1;									
					 }
					 break;				
				}			
				
				case 0x23:			
				{
					if( AppData->BuffSize == 2 )
					{		
						lora_config_application_port_set(AppData->Buff[1]);    //---->AT+PORT
						downlink_config_store_in_flash=1;						 
					}
					break;					
				}		
				
				case 0x25:			
				{
				 #if defined( REGION_AS923 )	|| defined( REGION_AU915 )
				 if( AppData->BuffSize == 2 )
				 {				
					 if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))   //---->AT+DWELLT
					 {
						 dwelltime=AppData->Buff[1];
						 downlink_config_store_in_flash=1;
						 atz_flags=1;		
					 }						
				 }
				 #endif	
				 break;				
				}
				
				case 0x26:
				{
					if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))  
					{
						uplink_message_data_status=1;		
						UTC_Request=1;					
					}
          else if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x02)) 
					{
						is_time_to_send=1;
					}						
					else if( AppData->BuffSize == 3 )
					{
						uint16_t value;			
						
						value=( AppData->Buff[1]<<8 | AppData->Buff[2] );//1~65535
						
						if(value>0)
						{
							REJOIN_TX_DUTYCYCLE=value;
							downlink_config_store_in_flash=1;
						}					
					}
					break;
				}
				
				case 0x27:
				{
					if( AppData->BuffSize == 2 )
					{				
						currentLeapSecond=AppData->Buff[1];
						downlink_config_store_in_flash=1;					
					}
					break;
				}

				case 0x28:
				{
					if( AppData->BuffSize == 2 )
					{				
						time_synchronization_method=AppData->Buff[1];
						if(time_synchronization_method>1)
						{
							time_synchronization_method=1;
						}
						
						if(Ext==0x09||time_synchronization_method==1)
						{
							StartCalibrationUTCtime(TX_ON_EVENT);
						}
						
						downlink_config_store_in_flash=1;					
					}
					break;
				}
				
				case 0x29:
				{
					if( AppData->BuffSize == 2 )
					{						
						time_synchronization_interval=AppData->Buff[1];
						if(time_synchronization_interval==0)
						{
							time_synchronization_interval=10;
						}
						downlink_config_store_in_flash=1;					
					}
					break;
				}

				case 0x30:
				{
					SysTime_t sysTime = { 0 };
					SysTime_t sysTimeCurrent = { 0 };
					SysTime_t downlinkTime = { 0 };
				
					if( AppData->BuffSize == 6 )
					{				
						downlinkTime.Seconds = ( uint32_t )AppData->Buff[1]<<24;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[2]<<16;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[3]<<8;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[4];
						downlinkTime.SubSeconds = AppData->Buff[5];	
						downlinkTime.SubSeconds = ( int16_t )( ( ( int32_t )downlinkTime.SubSeconds * 1000 ) >> 8 );
						
						sysTime=downlinkTime;

						sysTimeCurrent = SysTimeGet( );
						sysTime = SysTimeAdd( sysTime, SysTimeSub( sysTimeCurrent, LastTxdoneTime ) );

						if(sysTime.Seconds>1611878400)//20210129 00:00:00
						{											
							SysTimeSet( sysTime );
							
							sysTimeCurrent=SysTimeGet();	
							
							UTC_Request=0;
							UTC_Accept=1;
							
							MW_LOG(TS_OFF, VLEVEL_M, "Set current timestamp=%u\r",(unsigned int)sysTimeCurrent.Seconds);
						}
						else
						{
							MW_LOG(TS_OFF, VLEVEL_M, "timestamp error\r");
						}
					}
					break;
				}
				
//				case 0x31:
//				{
//					if( AppData->BuffSize == 10 )
//					{
//							uint32_t value,time;
//							timestamp1=( AppData->Buff[1]<<24 | AppData->Buff[2]<<16 | AppData->Buff[3]<<8 | AppData->Buff[4]);
//							timestamp2=( AppData->Buff[5]<<24 | AppData->Buff[6]<<16 | AppData->Buff[7]<<8 | AppData->Buff[8]);
//							time=AppData->Buff[9];
//						
//							if(time<5)
//							{
//								retrieval_uplink_time=5;
//							}
//							else
//								retrieval_uplink_time=time;
//							
//							if(timestamp1>timestamp2)
//							{
//								value=timestamp1;
//								timestamp2=timestamp1;
//								timestamp1=value;
//							}
//							
//							if(is_RetrievalTimer_running==0)
//							{							
//							  is_time_to_find_timestamp1_on_flash=1;
//							}														
//					}
//					break;
//				}
				
				case 0x32:
				{
					if( AppData->BuffSize == 6 )
					{
            uint16_t value;						
						value=AppData->Buff[1];
						if(value<2)
						{
							downlink_detect_switch=value;
						}
						
            value=AppData->Buff[2]<<8|AppData->Buff[3];
						if(value>0)
						{
							unconfirmed_uplink_change_to_confirmed_uplink_timeout=value;
						}
						
            value=AppData->Buff[4]<<8|AppData->Buff[5];
						if(value>0)
						{
							downlink_detect_timeout=value;
						}
						
						if(downlink_detect_switch==0)
						{
							TimerStop(&DownlinkDetectTimeoutTimer);
							TimerStop(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
						}
						else
						{
							TimerSetValue(&DownlinkDetectTimeoutTimer,downlink_detect_timeout*60000);
							TimerStart(&DownlinkDetectTimeoutTimer);
							
							if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
							{
								TimerSetValue(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
								TimerStart(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
							}	
						}
						
						downlink_config_store_in_flash=1;					
					}
					break;
				}
				
				case 0x33:
				{
					if( AppData->BuffSize == 3 )
					{
						LinkADR_NbTrans_retransmission_nbtrials=AppData->Buff[1];
						LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=AppData->Buff[2];
						
						if(LinkADR_NbTrans_retransmission_nbtrials==0)
						{
							LinkADR_NbTrans_retransmission_nbtrials=1;
						}
						
						if(LinkADR_NbTrans_retransmission_nbtrials>15)
						{
							LinkADR_NbTrans_retransmission_nbtrials=15;
						}
						
						if(LinkADR_NbTrans_uplink_counter_retransmission_increment_switch>1)
						{
							LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=1;
						}
						downlink_config_store_in_flash=1;
					}
					break;
				}
				
//				case 0x34:
//				{
//					if( AppData->BuffSize == 2 )
//					{				
//						pnackmd_switch=AppData->Buff[1];
//						if(pnackmd_switch>1)
//						{
//							pnackmd_switch=1;
//						}
//						
//						if(pnackmd_switch==1)
//						{
//							lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
//							confirmed_uplink_retransmission_nbtrials=0;
//							confirmed_uplink_counter_retransmission_increment_switch=0;
//						}
//						
//						downlink_config_store_in_flash=1;					
//					}
//					break;
//				}
				
				default:
					break;
			}
	
	if(TDC_flag==1)
	{
		Flash_Store_Config();
		TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    TimerStart( &TxTimer);
		TimerStart( &IWDGRefreshTimer);				
		TDC_flag=0;
	}
	
	if(downlink_config_store_in_flash==1)
	{
		downlink_config_store_in_flash=0;
		Flash_Store_Config();
	}
}

void sht20_temp_comp(void)
{

}

static void ext_temp_comp(void)
{

}

static void OnTxTimerEvent( void *context )
{
	TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
	
	is_time_to_send=1;
	
	if((( LoRaMacState & 0x00000001 ) == 0x00000001)||(( LoRaMacState & 0x00000010 ) == 0x00000010))
	{
		LoRaMacState_error_times++;
	}
}

static void LoraStartTx(TxEventType_t EventType)
{
	void *my_context;
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
//    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent(&my_context);
  }
}

static void OnIWDGRefreshTimeoutEvent( void *context )
{
	TimerSetValue( &IWDGRefreshTimer,  18000);

  TimerStart( &IWDGRefreshTimer);
	
	if(is_PDTA_command==1 || is_PLDTA_command==1)
	{
		iwdg_reload();
	}
	else
	  is_time_to_IWDG_Refresh=1;
}

static void StartIWDGRefresh(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &IWDGRefreshTimer, OnIWDGRefreshTimeoutEvent );
    TimerSetValue( &IWDGRefreshTimer,  18000); 
		TimerStart( &IWDGRefreshTimer);
  }
}

static void OnValidtimeTimeoutEvent( void *context )
{
	TimerStop( &ValidtimeTimer );	
	exit_send_flag=0;
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  MW_LOG(TS_OFF, VLEVEL_M, "switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  LoRaMacState_error_times=0;
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

void send_exti(void)
{
	if(exti_flag==1)
	{
	  is_time_to_send=1;
	}
}

void user_key_event(void)
{
	if(user_key_exti_flag==1)
	{		
		user_key_exti_flag=0;
		
		if(OnPressButtonTimeout_status==0)
		{
			OnPressButtonTimeout_status=1;
			TimerInit( &PressButtonTimeoutTimer, OnPressButtonTimeoutEvent );
			TimerSetValue( &PressButtonTimeoutTimer, 5000);
			TimerStart( &PressButtonTimeoutTimer );
		}
		
		press_button_times++;
		HAL_GPIO_WritePin(LED_RGB_PORT, LED_GREEN_PIN, 1);
			
		TimerTime_t currentTime = TimerGetCurrentTime();
		
		while(HAL_GPIO_ReadPin(GPIO_USERKEY_PORT,GPIO_USERKEY_PIN)==GPIO_PIN_RESET)
		{				
			if(TimerGetElapsedTime(currentTime) >= 1000 && TimerGetElapsedTime(currentTime) < 3000)//send
			{			
			  user_key_duration=1;
			}			
			else if(TimerGetElapsedTime(currentTime) >= 3000)//system reset,Activation Mode
			{ 
				*((uint8_t *)(0x2000F00A))=0x11;
        press_button_times=0;					
				for(int i=0;i<10;i++)
				{
					HAL_GPIO_TogglePin(LED_RGB_PORT, LED_GREEN_PIN);
					HAL_Delay(100);
				}
				user_key_duration=3;
				break;
			}			
    }
		
		HAL_GPIO_WritePin(LED_RGB_PORT, LED_GREEN_PIN, 0);
		
		if(press_button_times==5)
		{	
			press_button_times=0;
			user_key_duration=2;
		}
			
		switch(user_key_duration)
		{
			case 1:
			{
				user_key_duration=0;
				
				if(sleep_status==0 && (LORA_JoinStatus () == LORA_SET) && (( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					TimerStop(&TxTimer);
					
					if(work_mode!=3)
					{
						TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
						TimerStart(&TxTimer);
					}
				  Send();
				}
				break;
			}
			
			case 2://sleep
			{
				sleep_status=1;
				
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
				TimerStop(&RedLEDAlarmTimer2);
        TimerStop(&CheckBLETimesTimer);		
				
				HAL_Delay(500);
				TimerInit( &PressButtonTimesLedTimer, OnPressButtonTimesLedEvent );
				TimerSetValue( &PressButtonTimesLedTimer, 5000);
				HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,GPIO_PIN_SET); 
				TimerStart( &PressButtonTimesLedTimer );
				user_key_duration=0;
				HAL_Delay(50);					
				MW_LOG(TS_OFF, VLEVEL_M,"AT+PWRM2\r\n");
				HAL_Delay(100);	
				MW_LOG(TS_OFF, VLEVEL_M,"SLEEP\r\n");				
				ble_sleep_command=0;
				ble_sleep_flags=1;				
				break;
			}
			
			case 3://system reset,Activation Mode
			{
				user_key_duration=0;
				NVIC_SystemReset();
				break;
			}
			
			default:
				break;
		}
	}
}

void read_sensor_data(uint32_t timestamp,uint8_t dev,uint16_t bat,float temp20,float hum20,float extsen,uint16_t rev)
{	
	/*
	 timestamp           | Rev Dev Battery | Temp_sht20 Hum_sht20 | extsen Rev
	 xx   xx    xx  xx   | xx  xx  xxxx    | xxxx       xxxx      | xxxx   xxxx
	 total: 16bytes
	 */
	
	bsp_sensor_data[0]=timestamp>>24&0xFF;
	bsp_sensor_data[1]=timestamp>>16&0xFF;
	bsp_sensor_data[2]=timestamp>>8&0xFF;
	bsp_sensor_data[3]=timestamp&0xFF;
	
	bsp_sensor_data[4]=0;
	bsp_sensor_data[5]=dev;
	bsp_sensor_data[6]=bat>>8&0xFF;
	bsp_sensor_data[7]=bat&0xFF;
	
	bsp_sensor_data[8]=((short)temp20>>8)&0xff;
	bsp_sensor_data[9]=((short)temp20)&0xff;

	bsp_sensor_data[10]=((short)hum20>>8)&0xff;
	bsp_sensor_data[11]=((short)hum20)&0xff;

	if((dev==0x01)||(dev==0x02)||(dev==0x05)||(dev==0x06)||(dev==0x09)||(dev==0x0A)||(dev==0x0B))
	{
		bsp_sensor_data[12]=((short)extsen>>8)&0xff;
		bsp_sensor_data[13]=((short)extsen)&0xff;
		bsp_sensor_data[14]=(rev>>8)&0xff;
		bsp_sensor_data[15]=rev&0xff;
	}
	else if(dev==0x04)
	{	
//		level_status=HAL_GPIO_ReadPin(GPIO_Exit_PORT,GPIO_Exit_PIN);			
		bsp_sensor_data[12]= level_status&0xff;
		bsp_sensor_data[13]= exti_flag&0xff;
		bsp_sensor_data[14]=(rev>>8)&0xff;
		bsp_sensor_data[15]=rev&0xff;		
	}	
	else if((dev==0x08)||(dev==0x0e))
	{
		bsp_sensor_data[12]=(count>>24)&0xff;
		bsp_sensor_data[13]=(count>>16)&0xff;
		bsp_sensor_data[14]=(count>>8)&0xff;
		bsp_sensor_data[15]= count&0xff;		
	}	
	else	
	{
		bsp_sensor_data[12]=0x00;
		bsp_sensor_data[13]=0x00;
		bsp_sensor_data[14]=0x00;
		bsp_sensor_data[15]=0x00;		
	}
}

void OndownlinkLedEvent(void *context)
{
	TimerStop(&downlinkLedTimer);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,0);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_BLUE_PIN,0);
}

void OnNetworkJoinedLedEvent(void *context)
{
	TimerStop(&NetworkJoinedLedTimer);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,0);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,0);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_BLUE_PIN,0);
	
	join_led_timeout_status=1;
}

void OnPressButtonTimesLedEvent(void *context)
{
	TimerStop(&PressButtonTimesLedTimer);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,0);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,0);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_BLUE_PIN,0);
}

void OnPressButtonTimeoutEvent(void *context)
{
	TimerStop(&PressButtonTimeoutTimer);
	OnPressButtonTimeout_status=0;
	press_button_times=0;
}

static void find_timestamp1_on_flash(void)//find the address where timestamp1 is stored
{
	if(is_time_to_find_timestamp1_on_flash==1)
	{
		is_time_to_find_timestamp1_on_flash=0;
		is_RetrievalTimer_running=1;
		find_timestamp_addr=FLASH_SENSOR_DATA_START_ADDR;
		StartRetrieval(TX_ON_EVENT);
	}
}

static void Reply_to_retrieval_request(void)
{
	if(is_time_to_send_retrieval_data==1)
	{
		uint8_t buff[16];
		uint32_t timestamp_in_flash=0,nextdata=0;
		uint8_t find_status=1,j=0;
		
		if(LoRaMacState!=0x00000010 && LoRaMacState!=0x00000001)
		{
		  is_time_to_send_retrieval_data=0;
			
			TimerSetValue( &RetrievalTimer,  retrieval_uplink_time*1000); 
			TimerStart(&RetrievalTimer);
			
			if ( LORA_JoinStatus () != LORA_SET)
			{
				/*Not joined, try again later*/
				return;
			}
		
			for(int i=0;i<16;i++)
			{
				buff[i] = *(uint8_t *)find_timestamp_addr;
				find_timestamp_addr = find_timestamp_addr + 1;
			}
			
			timestamp_in_flash=buff[0]<<24|buff[1]<<16|buff[2]<<8|buff[3];
			
			while(timestamp_in_flash>timestamp2 || timestamp_in_flash<timestamp1 || timestamp_in_flash==0 || timestamp_in_flash==0xFFFFFFFF)
			{
				if(find_timestamp_addr>=FLASH_SENSOR_DATA_END_ADDR)
				{		
					find_status=0;
					break;
				}
				
        if(timestamp_in_flash>=timestamp1 && timestamp_in_flash<=timestamp2 )			
				{										
          find_status=1;
					break;
				}
				
				for(int i=0;i<16;i++)
				{
					buff[i] = *(uint8_t *)find_timestamp_addr;
					find_timestamp_addr = find_timestamp_addr + 1;
				}
                
        timestamp_in_flash=buff[0]<<24|buff[1]<<16|buff[2]<<8|buff[3];
			}
			
			if(find_status==1)
			{
					if((buff[5]==8)||(buff[5]==0x0e))
					{
						AppData.Buff[j++] =buff[14];         
						AppData.Buff[j++] =buff[15]; 						
					}
					else
					{
						AppData.Buff[j++] =buff[12];         //ext sensor data
						AppData.Buff[j++] =buff[13]; 
					}
					
					AppData.Buff[j++] =buff[8];          //sht_20 temp
					AppData.Buff[j++] =buff[9];

					if((buff[5]==11)&&(s31f_ext==1))
					{
					AppData.Buff[j++] =buff[14];         
					AppData.Buff[j++] =buff[15];								
					}
					else
					{							
					AppData.Buff[j++] =buff[10];         //sht_20 hum
					AppData.Buff[j++] =buff[11];
					}
					
					AppData.Buff[j++]=buff[5]|0x40;      //Ext & Poll message
				
					AppData.Buff[j++] =buff[0];		       //timestamp	
					AppData.Buff[j++] =buff[1];
					AppData.Buff[j++] =buff[2];
					AppData.Buff[j++] =buff[3];
					
					AppData.BuffSize = j;
					
				  AppData.BuffSize +=11;
					while(Uplink_data_adaptive_rate(&AppData)==1)
					{
						for(int i=0;i<16;i++)
						{
							buff[i] = *(uint8_t *)find_timestamp_addr;
							find_timestamp_addr = find_timestamp_addr + 1;
						}
						
						timestamp_in_flash=buff[0]<<24|buff[1]<<16|buff[2]<<8|buff[3];
						
						if(timestamp_in_flash>=timestamp1 && timestamp_in_flash<=timestamp2 )			
						{										
							if((buff[5]==8)||(buff[5]==0x0e))
							{
								AppData.Buff[j++] =buff[14];         
								AppData.Buff[j++] =buff[15]; 						
							}
							else
							{
								AppData.Buff[j++] =buff[12];         //ext sensor data
								AppData.Buff[j++] =buff[13]; 
							}
                        
    					AppData.Buff[j++] =buff[8];          //sht_20 temp
    					AppData.Buff[j++] =buff[9];

							if((buff[5]==11)&&(s31f_ext==1))
							{
							AppData.Buff[j++] =buff[14];         
							AppData.Buff[j++] =buff[15];								
							}
							else
							{							
    					AppData.Buff[j++] =buff[10];         //sht_20 hum
    					AppData.Buff[j++] =buff[11];
    					}
							
    				  AppData.Buff[j++]=buff[5]|0x40;      //Ext & Poll message
    				
    					AppData.Buff[j++] =buff[0];		     //timestamp	
    					AppData.Buff[j++] =buff[1];
    					AppData.Buff[j++] =buff[2];
    					AppData.Buff[j++] =buff[3];
							
							AppData.BuffSize = j;
							
							AppData.BuffSize +=11;
						}												
						
						if(find_timestamp_addr>=FLASH_SENSOR_DATA_END_ADDR)
						{									
							break;
						}
					}
					
					AppData.BuffSize -=11;
			}
			
			if(find_timestamp_addr>=FLASH_SENSOR_DATA_END_ADDR)
			{
				MW_LOG(TS_OFF, VLEVEL_M, "\n\rsend retrieve data completed\r");
				is_RetrievalTimer_running=0;
				TimerStop(&RetrievalTimer);
			}
			
			find_timestamp_addr_record=find_timestamp_addr;
			while(find_timestamp_addr_record<FLASH_SENSOR_DATA_END_ADDR)
			{
        for(int i=0;i<16;i++)
				{
					buff[i] = *(uint8_t *)find_timestamp_addr_record;
					find_timestamp_addr_record = find_timestamp_addr_record + 1;
				}
                        
				nextdata=buff[0]<<24|buff[1]<<16|buff[2]<<8|buff[3];               
				
				if(nextdata>=timestamp1 && nextdata<=timestamp2)
				{
					break;
				}
				
				if(find_timestamp_addr_record>=FLASH_SENSOR_DATA_END_ADDR)
				{	
					MW_LOG(TS_OFF, VLEVEL_M, "\n\rsend retrieve data completed\r");					
					is_RetrievalTimer_running=0;
					TimerStop(&RetrievalTimer);
					break;
				}
			}

			if(find_status==0)
			{
				TimerStop(&RetrievalTimer);
				MW_LOG(TS_OFF, VLEVEL_M, "\rNo data retrieved\r");
				is_RetrievalTimer_running=0;
				uint8_t i = 0;

				AppData.Port = lora_config_application_port_get();
				
				for(uint8_t j=0;j<11;j++)
				{
					AppData.Buff[i++] =0;
				}					
			
				LoRaMacState_error_times=0;
				AppData.BuffSize = i;
				LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			}
			else if(find_status==1)
			{
				AppData.Port = lora_config_application_port_get();
				LoRaMacState_error_times=0;
				if(LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG)==LORA_ERROR)
				{
					find_timestamp_addr=FLASH_SENSOR_DATA_START_ADDR;
				}
			}
		}
	}
}

static void Reply_to_not_recv_ack_data(void)
{
	if(is_time_to_send_no_recv_ack_data==1)
	{
		uint8_t buff[16];
		uint8_t find_status=1,j=0,addr_index=0,ack_status_in_sram=0;
		find_ack_status_addr = SRAM_SENSOR_DATA_STORE_ACK_START_ADDR;
	  uint32_t find_sensor_data_addr=0;
		
		if(LoRaMacState!=0x00000010 && LoRaMacState!=0x00000001)
		{			
			is_time_to_send_no_recv_ack_data=0;
			
			TimerSetValue( &ReplyNoRecvAckDataTimer,  10000); 
			TimerStart(&ReplyNoRecvAckDataTimer);
			
			ack_status_in_sram=*(uint8_t *)find_ack_status_addr;
	    find_ack_status_addr=find_ack_status_addr+1;
			
			while(ack_status_in_sram==0xEE || ack_status_in_sram==0x00)
			{
				if(find_ack_status_addr>=SRAM_SENSOR_DATA_STORE_ACK_END_ADDR)
				{		
					find_status=0;
					break;
				}
				
        if(ack_status_in_sram==0x01)			
				{					
          find_status=1;
					break;
				}
				
				ack_status_in_sram=*(uint8_t *)find_ack_status_addr;
				find_ack_status_addr=find_ack_status_addr+1;
			}
			
			if(find_status==1)
			{
				  no_recv_ack_addr_record[addr_index++]=find_ack_status_addr-1;
				  find_sensor_data_addr=(find_ack_status_addr-1-SRAM_SENSOR_DATA_STORE_ACK_START_ADDR)*16+FLASH_SENSOR_DATA_START_ADDR;
				
					for(int i=0;i<16;i++)
					{
						buff[i] = *(uint8_t *)find_sensor_data_addr;
						find_sensor_data_addr = find_sensor_data_addr + 1;
					}
				
					if((buff[5]==8)||(buff[5]==0x0e))
					{
						AppData.Buff[j++] =buff[14];         
						AppData.Buff[j++] =buff[15]; 						
					}
					else
					{
						AppData.Buff[j++] =buff[12];         //ext sensor data
						AppData.Buff[j++] =buff[13]; 
					}
									
					AppData.Buff[j++] =buff[8];          //sht_20 temp
					AppData.Buff[j++] =buff[9];

					if((buff[5]==11)&&(s31f_ext==1))
					{
					AppData.Buff[j++] =buff[14];         
					AppData.Buff[j++] =buff[15];								
					}
					else
					{					
					AppData.Buff[j++] =buff[10];         //sht_20 hum
					AppData.Buff[j++] =buff[11];
					}
					
					AppData.Buff[j++]=buff[5]|0x80;      //Ext & Poll message
				
					AppData.Buff[j++] =buff[0];		       //timestamp	
					AppData.Buff[j++] =buff[1];
					AppData.Buff[j++] =buff[2];
					AppData.Buff[j++] =buff[3];
					
					AppData.BuffSize = j;
					
				  AppData.BuffSize +=11;
					while(Uplink_data_adaptive_rate(&AppData)==1)
					{		
            ack_status_in_sram=*(uint8_t *)find_ack_status_addr;
				    find_ack_status_addr=find_ack_status_addr+1;						
						if(ack_status_in_sram==0x01)
						{	
							no_recv_ack_addr_record[addr_index++]=find_ack_status_addr-1;
							find_sensor_data_addr=(find_ack_status_addr-1-SRAM_SENSOR_DATA_STORE_ACK_START_ADDR)*16+FLASH_SENSOR_DATA_START_ADDR;
							
							for(int i=0;i<16;i++)
							{
								buff[i] = *(uint8_t *)find_sensor_data_addr;
								find_sensor_data_addr = find_sensor_data_addr + 1;
							}
						
							if((buff[5]==8)||(buff[5]==0x0e))
							{
								AppData.Buff[j++] =buff[14];         
								AppData.Buff[j++] =buff[15]; 						
							}
							else
							{
								AppData.Buff[j++] =buff[12];         //ext sensor data
								AppData.Buff[j++] =buff[13]; 
							}
											
							AppData.Buff[j++] =buff[8];          //sht_20 temp
							AppData.Buff[j++] =buff[9];
						
							if((buff[5]==11)&&(s31f_ext==1))
							{
							AppData.Buff[j++] =buff[14];         
							AppData.Buff[j++] =buff[15];								
							}
							else
							{
							AppData.Buff[j++] =buff[10];         //sht_20 hum
							AppData.Buff[j++] =buff[11];
							}
							
							AppData.Buff[j++]=buff[5]|0x80;      //Ext & Poll message
						
							AppData.Buff[j++] =buff[0];		       //timestamp	
							AppData.Buff[j++] =buff[1];
							AppData.Buff[j++] =buff[2];
							AppData.Buff[j++] =buff[3];
							
							AppData.BuffSize = j;
							
							AppData.BuffSize +=11;
						}												
						
						if(find_ack_status_addr>=SRAM_SENSOR_DATA_STORE_ACK_END_ADDR)
						{									
							break;
						}
					}
					
					AppData.BuffSize -=11;
			}
			
			if(find_ack_status_addr>=SRAM_SENSOR_DATA_STORE_ACK_END_ADDR)
			{				
				TimerStop(&ReplyNoRecvAckDataTimer);
			}

      if(find_status==1)
			{
				send_no_recv_ack_data_keep_status=1;
				ser_ack_record2=0;
				ser_ack_record=0;
				AppData.Port = lora_config_application_port_get();
				if(LORA_send( &AppData, LORAWAN_CONFIRMED_MSG)==LORA_ERROR)
				{
					send_no_recv_ack_data_keep_status=0;
					find_ack_status_addr=SRAM_SENSOR_DATA_STORE_ACK_START_ADDR;
				}
			}
		}		
	}
}

static void OnReJoinTimerEvent( void *context )
{
	TimerStop( &ReJoinTimer);
	
	is_time_to_rejoin=1;
}

static void LoraStartRejoin(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &ReJoinTimer, OnReJoinTimerEvent );
    TimerSetValue( &ReJoinTimer,  REJOIN_TX_DUTYCYCLE*60000); 
		TimerStart( &ReJoinTimer);
  }
}

static void OnRetrievalTimeoutEvent( void *context )
{
  TimerStop(&RetrievalTimer);
	
	is_time_to_send_retrieval_data=1;
}

static void StartRetrieval(TxEventType_t EventType)
{
	void *my_context;
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerSetValue( &RetrievalTimer,  retrieval_uplink_time*1000); 
		OnRetrievalTimeoutEvent(&my_context);
  }
}

static void OnReplyNoRecvAckDataTimeoutEvent( void *context )
{
  TimerStop(&ReplyNoRecvAckDataTimer);
	
	is_time_to_send_no_recv_ack_data=1;
}

static void StartReplyNoRecvAckData(TxEventType_t EventType)
{
	void *my_context;
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerSetValue( &ReplyNoRecvAckDataTimer,  10000); 
		OnReplyNoRecvAckDataTimeoutEvent(&my_context);
  }
}

static void OnRedLEDAlarm1TimeoutEvent( void *context )
{
  TimerSetValue( &RedLEDAlarmTimer1,  2000); 
	TimerStart(&RedLEDAlarmTimer1);
	
	is_time_to_LED_alarm=1;
}

static void StartRedLEDAlarm1(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerSetValue( &RedLEDAlarmTimer1,  2000); 
		TimerStart(&RedLEDAlarmTimer1);
  }
}

static void OnRedLEDAlarm2TimeoutEvent( void *context )
{
//  gpio_init(LED_RGB_PORT, LED_RED_PIN, GPIO_MODE_OUTPUT_PP_LOW);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,0);
	TimerStop(&RedLEDAlarmTimer2);
}

static void StartCalibrationUTCtime(TxEventType_t EventType)
{
	void *my_context;
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &CalibrationUTCTimer, OnCalibrationUTCTimeoutEvent );
    TimerSetValue( &CalibrationUTCTimer,  time_synchronization_interval*86400000); 
		OnCalibrationUTCTimeoutEvent(&my_context);
  }
}

static void OnCalibrationUTCTimeoutEvent( void *context )
{
  TimerSetValue( &CalibrationUTCTimer,  time_synchronization_interval*86400000);
	
  TimerStart( &CalibrationUTCTimer);
	
	UTC_Request=1;
	
	UTC_Accept=0;
	
	if(time_synchronization_method==1)
	{
		#if defined( REGION_US915 )
		MibRequestConfirm_t mib;
		mib.Type=MIB_CHANNELS_DATARATE;
		LoRaMacMibGetRequestConfirm(&mib);
		if(mib.Param.ChannelsDatarate==0)
		{
			payload_oversize=1;//mac command + payload>11
		}
		#elif defined ( REGION_AU915 ) || defined ( REGION_AS923 )
			MibRequestConfirm_t mib;
			mib.Type=MIB_CHANNELS_DATARATE;
			LoRaMacMibGetRequestConfirm(&mib);
			if(mib.Param.ChannelsDatarate==2&&dwelltime==1)
			{
				payload_oversize=1;//mac command + payload>11
			}
	  #endif
	}
}

void Startexttempcomptime(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &exttempcompTimer, onexttempcompTimeoutEvent );
		
		if(work_mode==3)
		{
				TimerSetValue( &exttempcompTimer,  external_sensor_sample_interval_for_work_mode3*1000); 
				TimerStart( &exttempcompTimer);
		}
		else
		{
			if(collection_second!=0)
			{
				TimerSetValue( &exttempcompTimer,  collection_second*1000); 
				TimerStart( &exttempcompTimer);
			}
	  }
  }	
}

static void onexttempcompTimeoutEvent( void *context )
{
		if(work_mode==3)
		{
			  is_time_to_exttemp_cmp=1;
				TimerSetValue( &exttempcompTimer,  external_sensor_sample_interval_for_work_mode3*1000); 
				TimerStart( &exttempcompTimer);
		}
		else
		{
			if(collection_second!=0)
			{
				is_time_to_exttemp_cmp=1;
				TimerSetValue( &exttempcompTimer,  collection_second*1000); 
				TimerStart( &exttempcompTimer);
			}
	  }
}

static void Startsht20tempcomptime(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &sht20tempcompTimer, onsht20tempcompTimeoutEvent );
    TimerSetValue( &sht20tempcompTimer,  collection_interval*60000); 
		TimerStart( &sht20tempcompTimer);
  }
}

static void onsht20tempcompTimeoutEvent( void *context )
{
  TimerSetValue( &sht20tempcompTimer,  collection_interval*60000);

  TimerStart( &sht20tempcompTimer);
	
	is_time_to_temp_comp=1;
}

void OnDS18B20IDTimeoutEvent(void *context)
{
   TimerSetValue( &DS18B20IDTimer,  86400000); 
//	TimerSetValue( &DS18B20IDTimer,  60000);
	 TimerStart( &DS18B20IDTimer );
	 
   if((pid_flag==1)&&(Ext==1 || Ext==9))
	 {
		 is_time_to_send_ds18b20_id=1;		 	
	 }		 
}

static void StartDS18B20ID(TxEventType_t EventType)
{
  void *my_context;	
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &DS18B20IDTimer, OnDS18B20IDTimeoutEvent );
    TimerSetValue( &DS18B20IDTimer,  86400000); 
		OnDS18B20IDTimeoutEvent(&my_context);
  }	
}

bool comp_ds18b20_id(void)
{
	return 0;
}

static void OnDownlinkDetectTimeoutEvent( void *context )
{
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
		rejoin_status=1;
	}
	
  /*Wait for next tx slot*/
  TimerStop( &DownlinkDetectTimeoutTimer);
}

static void StartDownlinkDetect(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &DownlinkDetectTimeoutTimer, OnDownlinkDetectTimeoutEvent );
    TimerSetValue( &DownlinkDetectTimeoutTimer,  downlink_detect_timeout*60000); 
    TimerStart( &DownlinkDetectTimeoutTimer);
  }
}

static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void *context )
{
	unconfirmed_uplink_change_to_confirmed_uplink_status=1;
	
  /*Wait for next tx slot*/
  TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
}

static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer, UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent );
    TimerSetValue( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,  unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
    TimerStart( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
  }
}

static void OnDelayresetTimeoutEvent( void *context )
{
	TimerStop(&DelayresetTimeoutTimer);
	atz_flags=1;
}

void LoraStartCheckBLE(void)
{
  TimerInit( &CheckBLETimesTimer, OnCheckBLETimesEvent );
  TimerSetValue( &CheckBLETimesTimer,  60000); 
  TimerStart( &CheckBLETimesTimer);	
}

void OnCheckBLETimesEvent(void *context)
{
	if(HAL_GPIO_ReadPin(GPIO_BT24_WORK_PORT,GPIO_BT24_WORK_PIN)==1)
	{
		TimerSetValue( &CheckBLETimesTimer,  60000); 
		TimerStart( &CheckBLETimesTimer);	
	}
	else
	{
		TimerStop( &CheckBLETimesTimer);	
		ble_sleep_command=1;
	}		
}


uint8_t HW_GetBatteryLevel( void ) 
{
    uint8_t batteryLevel = 0;
	  uint16_t bat_mv=0;
		
		bat_mv=SYS_GetBatteryLevel();

    if (bat_mv >= 3000)
    {
       batteryLevel = 254;
    }
		else if (bat_mv < 2405)
		{
			batteryLevel = 1;
		}
		else
		{
			batteryLevel = (( (uint32_t) (bat_mv - 2400)*254) /(3000-2400) ); 
		}
		return batteryLevel;
}

uint16_t HW_GetTemperatureLevel( void ) 
{  
  int16_t temperatureLevel = 0;

  temperatureLevel = (int16_t)(SYS_GetTemperatureLevel() >> 8);
  /* USER CODE BEGIN GetTemperatureLevel */

  /* USER CODE END GetTemperatureLevel */
  return temperatureLevel;
}

void HW_GetUniqueId( uint8_t *id )
{
  uint32_t val = 0;
  val = LL_FLASH_GetUDN();
  if (val == 0xFFFFFFFF)  /* Normally this should not happen */
  {
    uint32_t ID_1_3_val = HAL_GetUIDw0() + HAL_GetUIDw2();
    uint32_t ID_2_val = HAL_GetUIDw1();

    id[7] = (ID_1_3_val) >> 24;
    id[6] = (ID_1_3_val) >> 16;
    id[5] = (ID_1_3_val) >> 8;
    id[4] = (ID_1_3_val);
    id[3] = (ID_2_val) >> 24;
    id[2] = (ID_2_val) >> 16;
    id[1] = (ID_2_val) >> 8;
    id[0] = (ID_2_val);
  }
  else  /* Typical use case */
  {
    id[7] = val & 0xFF;
    id[6] = (val >> 8) & 0xFF;
    id[5] = (val >> 16) & 0xFF;
    id[4] = (val >> 24) & 0xFF;
    val = LL_FLASH_GetDeviceID();
    id[3] = val & 0xFF;
    val = LL_FLASH_GetSTCompanyID();
    id[2] = val & 0xFF;
    id[1] = (val >> 8) & 0xFF;
    id[0] = (val >> 16) & 0xFF;
  }
}

uint32_t HW_GetRandomSeed( void )
{
	uint32_t unique_id[2];
	
	unique_id[0] = HAL_GetUIDw0();
	unique_id[1] = HAL_GetUIDw1();
	
	unique_id[0]=unique_id[0]^unique_id[1];
	unique_id[1]=unique_id[0]&unique_id[1];
	
	return unique_id[0]^unique_id[1];
}

static void CmdProcessNotify(void)
{
	IsRxCmdReceived = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_3)
  {
			if((inmode!=0)&&(join_network==1))	
			{	
				if(inmode_delay==0)
				{				
					exti_flag=1;
				}
				else
				{
					status1=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
					TimerSetValue(&exdelay1Timer,inmode_delay); 
					TimerStart(&exdelay1Timer);				
				}
			}	
  }
	
  if (GPIO_Pin == GPIO_USERKEY_PIN)
  {
    user_key_exti_flag = 1;
  }

//  if (GPIO_Pin == GPIO_PIN_15)
//  {
//    APP_PRINTF("GPIO EXTI PB15\r\n");
//  }
}

static void Onexdelay1TimerEvent(void *context)
{
	TimerStop( &exdelay1Timer);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)==status1)
	{
		exti_flag=1;	  
	}
}

void LoraStartdelay1(void)
{
   TimerInit( &exdelay1Timer, Onexdelay1TimerEvent );
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void save_devNonce_buff(void)
{
	uint8_t bsp_sensor_data[16];
	
	SysTime_t sysTimeCurrent = { 0 };
	sysTimeCurrent=SysTimeGet();		

	bsp_sensor_data[0]=sysTimeCurrent.Seconds>>24&0xFF;
	bsp_sensor_data[1]=sysTimeCurrent.Seconds>>16&0xFF;
	bsp_sensor_data[2]=sysTimeCurrent.Seconds>>8&0xFF;
	bsp_sensor_data[3]=sysTimeCurrent.Seconds&0xFF;
	
	bsp_sensor_data[4]=LoRaMacDevNonce>>8&0xFF;
	bsp_sensor_data[5]=LoRaMacDevNonce&0xFF;
	
	write_address10=find_LoRaMacDevNonce_addr(write_address10);
	write_address10=store_sensor_data(write_address10,bsp_sensor_data);		
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
