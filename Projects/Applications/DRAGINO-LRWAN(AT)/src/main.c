#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "log.h"
#include "tremo_adc.h"
#include "tremo_uart.h"
#include "tremo_lpuart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "tremo_iwdg.h"
#include "tremo_delay.h"
#include "tremo_pwr.h"
#include "tremo_rtc.h"
#include "tremo_system.h"
#include "tremo_flash.h"
#include "flash_eraseprogram.h"
#include "sx126x.h"
#include "rtc-board.h"
#include "lora_app.h"
#include "bsp.h"
#include "command.h"
#include "gpio_exti.h"
#include "lora_config.h"
#include "LoRaMac.h"
#include "weight.h"
#include "pwm.h"

#define BCD_TO_HEX2(bcd) ((((bcd)>>4)*10)+((bcd)&0x0F))
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
#define LORAWAN_DEFAULT_DATA_RATE DR_0
#define LORAWAN_APP_PORT                            2
#define JOINREQ_NBTRIALS                            200
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
#define LORAWAN_APP_DATA_BUFF_SIZE                           242

log_level_t g_log_level = LL_ERR | LL_WARN | LL_DEBUG;

static uint8_t downlink_command_buffer[4];
static uint8_t downlink_command_buffersize=0;
static uint32_t ServerSetTDC;
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

__IO bool ble_sleep_flags=0;
__IO bool ble_sleep_command=0;
__IO bool wakeup_pa8_flag=0,wakeup_pa4_flag=0,wakeup_pb15_flag=0;
__IO bool uplink_data_status=0;
__IO bool manyuplink_data_status=0;
__IO bool is_time_to_IWDG_Refresh=0;
__IO uint16_t IWDG_Refresh_times=0;

uint32_t IWDG_Refresh_times_total_time=0;
uint32_t txdone_detection_timeout=0;
__IO uint8_t EnterLowPowerStopModeStatus=0,EnterLowPowerStopMode_error_times=0;

uint8_t switch_status=0,switch_status2=0,switch_status3=0;
static uint8_t normal_status=0,normal2_status=0,normal3_status=0;
bool is_check_exit=0;
bool join_network=0;
bool sleep_status=0;//AT+SLEEP
bool exti_flag=0,exti2_flag=0,exti3_flag=0;
bool exit_temp=0,exit2_temp=0,exit3_temp=0;
bool cfm_message_flag=0;
bool rxpr_flags=0;
bool joined_led=0;
bool joined_ledend=0;
bool joined_finish=0;
bool is_there_data=0;
bool rejoin_status=0;
bool rejoin_keep_status=0;
bool is_time_to_rejoin=0;
bool many_sends_flags=0;
bool downlink_data_status=0;
bool is_time_to_reply_downlink=0;
bool uplink_message_data_status=0;
bool JoinReq_NbTrails_over=0;
bool unconfirmed_downlink_data_ans_status=0,confirmed_downlink_data_ans_status=0;
bool MAC_COMMAND_ANS_status=0;
bool mac_response_flag=0;
uint8_t press_button_times=0;//Press the button times in a row fast
uint8_t OnPressButtonTimeout_status=0;
uint8_t user_key_exti_flag=0;
uint8_t user_key_duration=0;
uint8_t downlinklens;
uint8_t downlink_send[51];
uint8_t join_flag=0;
uint8_t TDC_flag=0;
uint8_t payloadlens=0;
uint8_t atz_flags=0;
uint8_t response_level=0;
uint16_t batteryLevel_mV;
uint16_t REJOIN_TX_DUTYCYCLE=20;//min
uint32_t APP_TX_DUTYCYCLE=300000;//ms
uint32_t count1=0,count2=0;

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

extern uint16_t inmode_delay,inmode2_delay,inmode3_delay;
extern uint8_t pwm_timer;
extern uint8_t product_id;
extern uint8_t fire_version;
extern uint8_t current_fre_band;
extern bool joined_flags;
extern uint16_t power_5v_time;
extern uint8_t dwelltime;
extern bool debug_flags;
extern uint8_t printf_dma_busy;
extern uint16_t printf_dma_idx_w;
extern uint16_t printf_dma_idx_r;

extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;
extern TimerEvent_t TxDelayedTimer;
extern bool rx2_flags;
extern uint32_t LoRaMacState;

extern bool bh1750flags;
extern uint8_t mode2_flag;
extern float GapValue;
extern uint8_t workmode;
extern uint8_t inmode,inmode2,inmode3;
extern bool message_flags;
extern bool mac_response_flag;
extern uint8_t fire_version;

static void LoraStartCheckBLE(void);
static void LoraStartTx(void);
static void LoraStartjoin(void);
static void StartIWDGRefresh(void);
static void LoraStartRejoin(void);
static void StartDownlinkDetect(void);
static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(void);

TimerEvent_t CheckBLETimesTimer;
TimerEvent_t PressButtonTimesLedTimer;
TimerEvent_t PressButtonTimeoutTimer;
TimerEvent_t downlinkLedTimer;
TimerEvent_t TxTimer;
TimerEvent_t TxTimer2;
TimerEvent_t NetworkJoinedLedTimer;
TimerEvent_t IWDGRefreshTimer;
TimerEvent_t ReJoinTimer;
TimerEvent_t DownlinkDetectTimeoutTimer;
TimerEvent_t UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer;

/* tx timer callback function*/
static void OnTxTimerEvent( void );
static void OnTxTimerEvent2( void );
void OndownlinkLedEvent(void);
void OnCheckBLETimesEvent(void);
void OnPressButtonTimesLedEvent(void);
void OnPressButtonTimeoutEvent(void);
void OndownlinkLedEvent(void);
void OnNetworkJoinedLedEvent(void);
static void OnIWDGRefreshTimeoutEvent(void);
static void OnReJoinTimerEvent( void );
static void OnDownlinkDetectTimeoutEvent( void );
static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void );

void board_init(void);
void sensors_data(void);
static void send_exti_pa4(void);
static void send_exti_pa8(void);
static void send_exti_pb15(void);
static void Send( void );
static void Send_device_status(void);
static void Send_reply_downlink( void );
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
static void LORA_RxData( lora_AppData_t *AppData);
static void LORA_HasJoined( void );
static void LORA_ConfirmClass ( DeviceClass_t Class );
uint8_t HW_GetBatteryLevel( void ); 
uint16_t HW_GetTemperatureLevel( void );	
void HW_GetUniqueId( uint8_t *id );
void check_pin_status(void);
uint32_t HW_GetRandomSeed( void );
void user_key_event(void);
extern bool print_isdone(void);
extern void printf_joinmessage(void);
extern void weightreset(void);
extern void LoraStartdelay1(void);
extern void LoraStartdelay2(void);
extern void LoraStartdelay3(void);

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

/* Private functions ---------------------------------------------------------*/
																	
void uart_log_init(uint32_t baudrate)
{
    lpuart_init_t lpuart_init_cofig;
    uart_config_t uart_config;

    // set iomux
    gpio_set_iomux(GPIOD, GPIO_PIN_12, 2); // LPUART_RX:GP60
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);  // UART0_TX:GP17

    // lpuart init
    lpuart_deinit(LPUART);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPUART, true);
    lpuart_init_cofig.baudrate         = baudrate;
    lpuart_init_cofig.data_width       = LPUART_DATA_8BIT;
    lpuart_init_cofig.parity           = LPUART_PARITY_NONE;
    lpuart_init_cofig.stop_bits        = LPUART_STOP_1BIT;
    lpuart_init_cofig.low_level_wakeup = false;
    lpuart_init_cofig.start_wakeup     = false;
    lpuart_init_cofig.rx_done_wakeup   = true;
    lpuart_init(LPUART, &lpuart_init_cofig);

    lpuart_config_interrupt(LPUART, LPUART_CR1_RX_DONE_INT, ENABLE);
    lpuart_config_tx(LPUART, false);
    lpuart_config_rx(LPUART, true);

    NVIC_SetPriority(LPUART_IRQn, 2);
    NVIC_EnableIRQ(LPUART_IRQn);

    // uart init
    uart_config_init(&uart_config);
    uart_config.fifo_mode = ENABLE;
    uart_config.mode     = UART_MODE_TX;
    uart_config.baudrate = baudrate;
    uart_init(CONFIG_DEBUG_UART, &uart_config);

    uart_cmd(CONFIG_DEBUG_UART, ENABLE);
}

void board_init(void)
{
		rcc_enable_oscillator(RCC_OSC_RCO32K, true);
		rcc_set_iwdg_clk_source(RCC_IWDG_CLK_SOURCE_RCO32K);
		rcc_enable_peripheral_clk(RCC_PERIPHERAL_IWDG, true);
    delay_ms(100);	  
    iwdg_init(true);

    iwdg_set_prescaler(IWDG_PRESCALER_256);
    iwdg_set_reload(0xFFF); 
    iwdg_start();
    rcc_enable_oscillator(RCC_OSC_XO32K, true);

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LPUART, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_UART0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_PWR, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_RTC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SAC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_LORA, true);
    #ifdef PRINT_BY_DMA
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_SYSCFG, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_DMA0, true);
    #endif

    delay_ms(100);

	  int rst_src = 0;
	
	  rst_src = RCC->RST_SR;

    if(!(rst_src & 0x24)){
        RtcInit();
			  *((unsigned int *)(0x2000F000)) =0x00;
				*((unsigned int *)(0x2000F001)) =0x00;
				*((unsigned int *)(0x2000F002)) =0x00;
				*((unsigned int *)(0x2000F003)) =0x00;
			
				*((unsigned int *)(0x2000F004)) =0x00;
				*((unsigned int *)(0x2000F005)) =0x00;
			
			  *((uint8_t *)(0x2000F00A))=0x11;
			
			  for(uint16_t i=0;i<3328;i++)
				{
					*((unsigned int *)(SRAM_SENSOR_DATA_STORE_ACK_START_ADDR+i)) =0xEE;
				}
    }else{
        rtc_calendar_cmd(ENABLE);
        NVIC_EnableIRQ(RTC_IRQn);
        
        RCC->RST_SR |= 0x24;
    }
}

void* _sbrk(int nbytes)
{
    // variables from linker script
    extern char _heap_bottom[];
    extern char _heap_top[];

    static char* heap_ptr = _heap_bottom;

    if ((unsigned int)(heap_ptr + nbytes) <= (unsigned int)_heap_top) {
        void* base = heap_ptr;
        heap_ptr += nbytes;
        return base;
    } else {
        return (void*)(-1);
    }
}

int main(void)
{
    // Target board initialization
    board_init();
	  SX126xInit();	
    uart_log_init(9600);
	  linkwan_at_init();
		Flash_read_key();
		Flash_Read_Config();	
	  new_firmware_update();	
		BSP_sensor_Init();
		StartIWDGRefresh();		
	  LoraStartCheckBLE();
	
	  display_message();
	  if(*((uint8_t *)(0x2000FE0B))==0xAA)
		{
			count1=(*((uint8_t *)(0x2000FE0C)))<<24|(*((uint8_t *)(0x2000FE0D)))<<16|(*((uint8_t *)(0x2000FE0E)))<<8|(*((uint8_t *)(0x2000FE0F)));
			count2=(*((uint8_t *)(0x2000FE10)))<<24|(*((uint8_t *)(0x2000FE11)))<<16|(*((uint8_t *)(0x2000FE12)))<<8|(*((uint8_t *)(0x2000FE13)));
		}
		
	  if(debug_flags==1)
		{
			LOG_PRINTF(LL_DEBUG,"dragino_6601_ota\r\n");
		}
		
		/* Configure the Lora Stack*/
		LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
	
	  TimerInit( &downlinkLedTimer, OndownlinkLedEvent );	
		LoraStartdelay1(); //PA8_exit_Delay
		LoraStartdelay2(); //PA4_exit_Delay
		LoraStartdelay3(); //PB15_exit_Delay
		
	  while( 1 )
    {  		
			  if (Radio.IrqProcess != NULL) {
            Radio.IrqProcess();
        }			
				
				/* Handle UART commands */
				linkwan_at_process();
					
			  if(ble_sleep_command==1)
			  {			
					delay_ms(50);					
					LOG_PRINTF(LL_DEBUG,"AT+PWRM2\r\n");
					delay_ms(100);										
					ble_sleep_command=0;
					ble_sleep_flags=1;
				}
				
				if(joined_led==1)
				{	
					joined_led=0;		
					joined_ledend=1;
					gpio_init(LED_RGB_PORT, LED_RED_PIN, GPIO_MODE_OUTPUT_PP_LOW);	
					gpio_init(LED_RGB_PORT, LED_BLUE_PIN, GPIO_MODE_OUTPUT_PP_LOW);	
					gpio_init(LED_RGB_PORT, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP_HIGH);	
					TimerInit( &NetworkJoinedLedTimer, OnNetworkJoinedLedEvent );
					TimerSetValue( &NetworkJoinedLedTimer, 5000);
					TimerStart( &NetworkJoinedLedTimer );				
				}
		
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
				
				if(rejoin_status==1)
				{
					if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
					{
						rejoin_keep_status=1;
						LoRaMacState_error_times=0;
						unconfirmed_uplink_change_to_confirmed_uplink_status=0;		
						TimerStop(&TxTimer);						
						TimerStop( &DownlinkDetectTimeoutTimer);
						TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);					
						LORA_Join();
					}
				}		

				if((inmode!=0)&&(sleep_status==0))
        {						
					send_exti_pa8();			
				}
				if((inmode2!=0)&&(sleep_status==0))
        {						
					send_exti_pa4();			
				}				
				if((inmode3!=0)&&(sleep_status==0))
        {						
					send_exti_pb15();			
				}
					
				if(joined_finish==1)
				{		
					if(((workmode!=6)&&(workmode!=9)&&(workmode!=3)&&(workmode!=8)&&(workmode!=12))&&(exti_flag==1))
					{
						if((( LoRaMacState & 0x00000001 ) != 0x00000001 )&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
						{
							uplink_data_status=1;
							exit_temp=1;
							exti_flag=0;
						}
					}		

					if((workmode==7)&&(exti2_flag==1))
					{
						if((( LoRaMacState & 0x00000001 ) != 0x00000001 )&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
						{
							uplink_data_status=1;
							exit2_temp=1;
							exti2_flag=0;
						}
					}	

					if(((workmode==3)||(workmode==7)||(workmode==8)||(workmode==9))&&(exti3_flag==1))
					{
						if((( LoRaMacState & 0x00000001 ) != 0x00000001 )&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
						{
							uplink_data_status=1;
							exit3_temp=1;
							exti3_flag=0;
						}
					}	
					
					if(LoRaMacState_error_times>=5)
					{
						LoRaMacState_error_times=0;
						delay_ms(100);
						system_reset();
					}
					
					if(atz_flags==1)
					{
						LoRaMacState_error_times=0;
						delay_ms(500);
						AppData.Buff[0]=0x11;
						AppData.BuffSize=1;
						AppData.Port = 4;
						LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
						atz_flags++;
					}
					else if((atz_flags==2)&&(( LoRaMacState & 0x00000001 ) != 0x00000001))
					{
						system_reset();
					}
					
					#ifdef REGION_US915
					if((MAC_COMMAND_ANS_status==1)&&(mac_response_flag==0))
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
					if((MAC_COMMAND_ANS_status==1)&&(dwelltime==1)&&(mac_response_flag==0))
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
					||(((MAC_COMMAND_ANS_status==1)||(confirmed_downlink_data_ans_status==1&&is_there_data==1))&&(response_level==4))
					||(((MAC_COMMAND_ANS_status==1)||(is_there_data==1))&&(response_level==5)))
					{
						if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
						{
							LoRaMacState_error_times=0;
							MAC_COMMAND_ANS_status=0;
							unconfirmed_downlink_data_ans_status=0;
							confirmed_downlink_data_ans_status=0;
							is_there_data=0;
							if(downlink_data_status==1)
							{
								downlink_data_status=0;
								Send_reply_downlink();
							}
							else
							{
								AppData.Buff[0]=0x00;
								AppData.BuffSize=1;
								AppData.Port = 4;							
								LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
							}
						}
					}
					
					if(downlink_received_status==1 && LORA_JoinStatus () == LORA_SET && downlink_detect_switch==1 && downlink_detect_timeout>0 && unconfirmed_uplink_change_to_confirmed_uplink_timeout>0)
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
							AppData.Port = 12;
							LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
						}
					}

					if((uplink_message_data_status==1)&&(( LoRaMacState & 0x00000001 ) != 0x00000001) &&(( LoRaMacState & 0x00000010 ) != 0x00000010))
					{				
						Send_device_status();			
						uplink_message_data_status=0;
					}		

          if((uplink_data_status==1)&&(( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
					{	
						Send();
						uplink_data_status=0;
					}
      
					if(is_check_exit==1)
					{
						if((( LoRaMacState & 0x00000001 ) != 0x00000001) &&(( LoRaMacState & 0x00000010 ) != 0x00000010))
						{
							normal_status=gpio_read(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN);
							normal2_status=gpio_read(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN);
							normal3_status=gpio_read(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN);
							is_check_exit=0;
							if(((switch_status!=normal_status)&&((workmode!=6)&&(workmode!=9)&&(workmode!=3)&&(workmode!=8)&&(workmode!=12))&&(inmode==1))||
								((switch_status2!=normal2_status)&&(workmode==7)&&(inmode2==1))||
								((switch_status3!=normal3_status)&&((workmode==3)||(workmode==7)||(workmode==8)||(workmode==9))&&(inmode3==1)))
							{
								switch_status=normal_status;
								switch_status2=normal2_status;
								switch_status3=normal3_status;
								uplink_data_status=1;
							}		
						}
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
					LoraStartRejoin();
				}
				
				if(is_time_to_IWDG_Refresh==1)
				{
					is_time_to_IWDG_Refresh=0;
					iwdg_reload();

					IWDG_Refresh_times_total_time = IWDG_Refresh_times*18;
					
					txdone_detection_timeout = (3*APP_TX_DUTYCYCLE)/1000;
					
					if((IWDG_Refresh_times_total_time>txdone_detection_timeout) && sleep_status==0 && joined_flags==0)
					{
						IWDG_Refresh_times=0;
						system_reset();
					}
					
					IWDG_Refresh_times++;
			
					if(EnterLowPowerStopModeStatus==0)
					{
						EnterLowPowerStopMode_error_times++;
						if(EnterLowPowerStopMode_error_times>=10)
						{
							EnterLowPowerStopMode_error_times=0;
							printf_dma_idx_w=  printf_dma_idx_r;
							printf_dma_busy=0;
						}					
					}			
				}
		
				user_key_event();		
				
				if( print_isdone( ) ) {
						EnterLowPowerStopModeStatus=1;
				  	TimerLowPowerHandler( );
				}
				else
					EnterLowPowerStopModeStatus=0;			
		}
}

static void LORA_HasJoined( void )
{	
	exti_flag=0;
	exti2_flag=0;
	exti3_flag=0;	
	joined_led=1;
	joined_finish=1;
	join_network=1;
	
  LOG_PRINTF(LL_DEBUG,"JOINED\n\r");
	
	rejoin_keep_status=0;
	
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
		printf_joinmessage();
	}		
	
	TimerStop(&ReJoinTimer);	
	
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
	
	LoraStartjoin();
}

static void Send( void )
{
	if(EnterLowPowerStopModeStatus==0)
	{
    printf_dma_idx_w=  printf_dma_idx_r;
		printf_dma_busy=0;
	}
	
	is_there_data=0;  

	check_pin_status();
	
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
	BSP_sensor_Read(&bsp_sensor_data_buff,message_flags,workmode);
	message_flags=0;
	
	uint8_t i = 0;

	AppData.Port = lora_config_application_port_get();	
  if(workmode==1)
	{		
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4);

		AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pa8<<7)|(bsp_sensor_data_buff.in1<<1)|(exit_temp&0x01);
	
		if(bh1750flags==1)
		{
			AppData.Buff[i++] =(bsp_sensor_data_buff.illuminance)>>8;      
			AppData.Buff[i++] =(bsp_sensor_data_buff.illuminance);
			AppData.Buff[i++] = 0x00;   
			AppData.Buff[i++] = 0x00;				
		}	
		else
		{
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10)>>8;      
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10);
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10)>>8;   
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10);
		}
	}
	else if(workmode==2)
	{
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int) bsp_sensor_data_buff.ADC_4;

		AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pa8<<7)|(bsp_sensor_data_buff.in1<<1)|0x04|(exit_temp&0x01);

		AppData.Buff[i++]=(bsp_sensor_data_buff.distance_mm)>>8;
		AppData.Buff[i++]=(bsp_sensor_data_buff.distance_mm);	
		if(mode2_flag==3)
		{
		  AppData.Buff[i++]=(bsp_sensor_data_buff.distance_signal_strengh)>>8;
		  AppData.Buff[i++]=(bsp_sensor_data_buff.distance_signal_strengh);				
		}
		else
		{
			AppData.Buff[i++] =0xff; 
			AppData.Buff[i++] =0xff;
		}			
	}
	else if(workmode==3)
	{	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int) bsp_sensor_data_buff.ADC_4;

		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_5)>>8;          
		AppData.Buff[i++] =(int) bsp_sensor_data_buff.ADC_5;
		
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_8)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_8);
		
		AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pb15<<7)|0x08|(exit3_temp&0x01);

		if(bh1750flags==1)
		{
			AppData.Buff[i++] =(bsp_sensor_data_buff.illuminance)>>8;      
			AppData.Buff[i++] =(bsp_sensor_data_buff.illuminance);
			AppData.Buff[i++] = 0x00;   
			AppData.Buff[i++] = 0x00;				
		}	
		else
		{
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10)>>8;      
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10);
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10)>>8;   
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10);
		}
   
    AppData.Buff[i++] =(int)(bsp_sensor_data_buff.bat_mv/100);			
	}
	else if(workmode==4)
	{	
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4);
		
		AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pa8<<7)|(bsp_sensor_data_buff.in1<<1)|0x0C|(exit_temp&0x01);

		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp2*10)>>8;      
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp2*10);
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp3*10)>>8;   
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp3*10);	
	}
	else if(workmode==5)
	{	
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4);
		
		AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pa8<<7)|(bsp_sensor_data_buff.in1<<1)|0x10|(exit_temp&0x01);

		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.Weight)>>8;      
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.Weight);
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.Weight)>>24;   
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.Weight)>>16;	
	}
	else if(workmode==6)
	{	
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int) bsp_sensor_data_buff.ADC_4;
		
		AppData.Buff[i++]= (bsp_sensor_data_buff.in1<<1)|0x14;

		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8)>>24;      
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8)>>16;
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8)>>8;   
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8);	
	}
	else if(workmode==7)
	{	
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_5)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_5);
		
		AppData.Buff[i++] =(bsp_sensor_data_buff.exit_pa8<<7)|0x18|(exit_temp&0x01);			
  	AppData.Buff[i++] =(exit2_temp<<4) | bsp_sensor_data_buff.exit_pa4;           				
	  AppData.Buff[i++] =(exit3_temp<<4) | bsp_sensor_data_buff.exit_pb15;   

		AppData.Buff[i++] = 0xFF;   
		AppData.Buff[i++] = 0xFF;	
	}
	else if(workmode==8)
	{	
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4);
		
		AppData.Buff[i++] =(bsp_sensor_data_buff.exit_pb15<<7)|0x1C|(exit3_temp&0x01);		

		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_5)>>8;     
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_5);
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_8)>>8; 
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_8);	
	}
	else if(workmode==9)
	{	
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp2*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp2*10);
		
		AppData.Buff[i++] =(bsp_sensor_data_buff.exit_pb15<<7)|0x20|(exit3_temp&0x01);	

		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp3*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp3*10);

		AppData.Buff[i++] = (uint8_t)((bsp_sensor_data_buff.count_pa8)>>24);
		AppData.Buff[i++] =	(uint8_t)((bsp_sensor_data_buff.count_pa8)>>16);	
		AppData.Buff[i++] = (uint8_t)((bsp_sensor_data_buff.count_pa8)>>8);
		AppData.Buff[i++] =	(uint8_t)(bsp_sensor_data_buff.count_pa8); 
		
		AppData.Buff[i++] = (uint8_t)((bsp_sensor_data_buff.count_pa4)>>24);
		AppData.Buff[i++] =	(uint8_t)((bsp_sensor_data_buff.count_pa4)>>16);	
		AppData.Buff[i++] = (uint8_t)((bsp_sensor_data_buff.count_pa4)>>8);
		AppData.Buff[i++] =	(uint8_t)(bsp_sensor_data_buff.count_pa4); 	
	}
  else if(workmode==10)
	{		
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4);

		AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pa8<<7)|0x24|((pwm_timer&0x01)<<1)|(exit_temp&0x01);
	
		AppData.Buff[i++]= (bsp_sensor_data_buff.pwm_freq)>>8;
		AppData.Buff[i++]=  bsp_sensor_data_buff.pwm_freq;
		
		AppData.Buff[i++]= (bsp_sensor_data_buff.pwm_duty)>>8;
		AppData.Buff[i++]=  bsp_sensor_data_buff.pwm_duty;
	}
  else if(workmode==11)
	{		
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;
	
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10)>>8;     
		AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp1*10);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4)>>8;          
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.ADC_4);

		AppData.Buff[i++]=(bsp_sensor_data_buff.exit_pa8<<7)|0x28|(bsp_sensor_data_buff.in1<<1)|(exit_temp&0x01);
	
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_tmp117*100)>>8;      
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_tmp117*100);
		AppData.Buff[i++] = 0x00;   
		AppData.Buff[i++] = 0x00; 
	}
	else if(workmode==12)
	{
		AppData.Buff[i++] =(bsp_sensor_data_buff.bat_mv>>8);       
		AppData.Buff[i++] = bsp_sensor_data_buff.bat_mv & 0xFF;		
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10)>>8;      
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*10);
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10)>>8;   
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10);	
		AppData.Buff[i++]= (bsp_sensor_data_buff.in1<<1) | 0x2C;		
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8)>>24;      
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8)>>16;
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8)>>8;   
		AppData.Buff[i++] =(int)(bsp_sensor_data_buff.count_pa8);	    		
	}
	
  AppData.BuffSize = i;
	payloadlens=i;
	
	if(exit_temp==1)
		exit_temp=0;
	if(exit2_temp==1)
		exit2_temp=0;
	if(exit3_temp==1)
		exit3_temp=0;

	if((inmode==1)||(inmode2==1)||(inmode3==1))
	{
		switch_status=gpio_read(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN);
		switch_status2=gpio_read(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN);
		switch_status3=gpio_read(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN);
		is_check_exit=1;
	}
	
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
	if(EnterLowPowerStopModeStatus==0)
	{
    printf_dma_idx_w=  printf_dma_idx_r;
		printf_dma_busy=0;
	}	
	
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
	#elif defined( REGION_CN470 )
		freq_band=0x0B;
	#elif defined( REGION_EU433 )
	  freq_band=0x0C;
	#elif defined( REGION_KR920 )	
	  freq_band=0x0D;
	#elif defined( REGION_MA869 )	
	  freq_band=0x0E;
	#else
    freq_band=0x00;
	#endif
	
	#if defined( REGION_US915 )	|| defined( REGION_AU915 ) || defined( REGION_CN470 )
	sub_band = customize_set8channel_get();
	#else
	sub_band = 0xff;
	#endif
	
	if(fire_version>100)
	{
		version=(fire_version/100)<<8|(fire_version/10%10)<<4|(fire_version%10);
	}
	else
	{
		version=(fire_version/10)<<8|(fire_version%10)<<4;		
	}

	uint16_t battery_mv;
  battery_mv=battery_voltage_measurement();
  if(battery_mv>3750)
	{
	  battery_mv=battery_voltage_measurement();
	}	
	
	uint32_t i = 0;

  AppData.Port = 5;
	
	AppData.Buff[i++] = 0x1C;
	
	AppData.Buff[i++] = (version>>8)&0xff;
	AppData.Buff[i++] =  version&0xff;
	
	AppData.Buff[i++] = freq_band;
	
	AppData.Buff[i++] = sub_band;

  AppData.Buff[i++]= (battery_mv >>8);
	AppData.Buff[i++]=  battery_mv;
	
	AppData.BuffSize = i;
	payloadlens=i;
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);		  
}

static void Send_reply_downlink( void )
{
	if(EnterLowPowerStopModeStatus==0)
	{
    printf_dma_idx_w=  printf_dma_idx_r;
		printf_dma_busy=0;
	}
	
	is_there_data=0;  	
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }

	uint32_t i = 0;

	AppData.Port = 100;	
	
	for(int j=0;j<downlinklens;j++)
	{
		AppData.Buff[i++] = downlink_send[j];
		downlink_send[j]=0x00;
	}	
		
	downlinklens=0;
  AppData.BuffSize = i;
	payloadlens=i;
	LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);	
}

static void LORA_RxData( lora_AppData_t *AppData )
{
	uint8_t downlink_config_store_in_flash=0;
  is_there_data=1;
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
	
  TimerSetValue(&downlinkLedTimer, 200);
  gpio_write(LED_RGB_PORT, LED_RED_PIN, 1);
	gpio_write(LED_RGB_PORT, LED_BLUE_PIN, 1);

  TimerStart( &downlinkLedTimer );
	
  switch(AppData->Buff[0] & 0xff)
  {
		case 0x01:
		{
			if( AppData->BuffSize == 4 )
			{
			  ServerSetTDC=( AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3] );//S
				if(ServerSetTDC<5)
				{
				  LOG_PRINTF(LL_DEBUG,"TDC setting needs to be high than 4s\n\r");
				}
				else
				{
				  TDC_flag=1;
			    APP_TX_DUTYCYCLE=ServerSetTDC*1000;
					rxpr_flags=1;								
				}						
			}
			break;
		}
				
    case 0x04:
		{
			if( AppData->BuffSize == 2 )
			{
				if(AppData->Buff[1]==0xFF)  //---->ATZ
				{
					atz_flags=1;		
					rxpr_flags=1;								
				}
				else if(AppData->Buff[1]==0xFE)  //---->AT+FDR
			  {	
					uint8_t status[128]={0};
					memset(status, 0x00, 128);
          __disable_irq();
					status[0]=0x12;
					status[1]=product_id;
					status[2]=current_fre_band;
					status[3]=fire_version;					
					flash_erase_page(FLASH_USER_START_ADDR_CONFIG);
					delay_ms(5);					
					if(flash_program_bytes(FLASH_USER_START_ADDR_CONFIG,status,128)==ERRNO_FLASH_SEC_ERROR)
					{
						LOG_PRINTF(LL_DEBUG,"write config error\r\n");
					}
					__enable_irq();
					
					atz_flags=1;			
					rxpr_flags=1;								
				}
			}
			break;
		}	
				
    case 0x05:
		{
			is_time_to_reply_downlink=1;
					
			if( AppData->BuffSize == 4)
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
					rxpr_flags=1;			
			  }
			}
			else if(AppData->BuffSize == 2)
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
				rxpr_flags=1;	
					
				downlink_command_buffersize=AppData->BuffSize;
				for(int i=0;i<AppData->BuffSize;i++)
				{
					downlink_command_buffer[i]=AppData->Buff[i];
				}				
			}
			else
		  {
				is_time_to_reply_downlink=0;
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
					rxpr_flags=1;		
				}			
			  else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x01)&&(AppData->Buff[3]<=0x04))   		  //---->AT+INTMOD2
				{
					if( AppData->BuffSize == 6)
					{
						inmode2_delay=AppData->Buff[4]<<8 | AppData->Buff[5];
					}
					inmode2=AppData->Buff[3];
					GPIO_EXTI4_IoInit(inmode);
					downlink_config_store_in_flash=1;
					rxpr_flags=1;		
				}		
			  if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x02)&&(AppData->Buff[3]<=0x04))   		  //---->AT+INTMOD3
				{
					if( AppData->BuffSize == 6)
					{
						inmode3_delay=AppData->Buff[4]<<8 | AppData->Buff[5];
					}
					inmode3=AppData->Buff[3];
					GPIO_EXTI15_IoInit(inmode);
					downlink_config_store_in_flash=1;
					rxpr_flags=1;		
				}						
		  }
			break;	
	  }	
		
		case 0x07:
		{	
			if( AppData->BuffSize == 3 )
			{	
			  power_5v_time=(AppData->Buff[1]<<8) | AppData->Buff[2];  //---->AT+5VT
				downlink_config_store_in_flash=1;
			  rxpr_flags=1;							
			}			
			break;									
		}

    case 0x08:			
		{
			if(workmode==5)
			{
				if((AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))   //---->AT+WEIGRE
				{	
				  weightreset();
				  rxpr_flags=1;							
				}
				else if((AppData->BuffSize == 4 )&&(AppData->Buff[1]==0x02))  //---->AT+WEIGAP
				{
				  GapValue=(float)((AppData->Buff[2]<<8 | AppData->Buff[3])/10.0);
					downlink_config_store_in_flash=1;					
					rxpr_flags=1;							
				}
			}
			break;			
	  }		
		
		case 0x09:
		{
			if( AppData->BuffSize == 6 )             //---->AT+SETCNT
			{	
				if(AppData->Buff[1]==0x01)
				{				 
					count1=AppData->Buff[2]<<24 | AppData->Buff[3]<<16 | AppData->Buff[4]<<8 | AppData->Buff[5];
					rxpr_flags=1;								
				}
				else if(AppData->Buff[1]==0x02)
				{					 
					count2=AppData->Buff[2]<<24 | AppData->Buff[3]<<16 | AppData->Buff[4]<<8 | AppData->Buff[5];
				  rxpr_flags=1;							
				}
			}				
			break;				
		}

		case 0x0A:
		{
			if( AppData->BuffSize == 2 )         
			{	
				if((AppData->Buff[1]>=0x01)&&(AppData->Buff[1]<=0x0C))    //---->AT+MOD
				{
					workmode=AppData->Buff[1];
					downlink_config_store_in_flash=1;
					atz_flags=1;						
					rxpr_flags=1;	
				}						 
			}				
			break;
		}

		case 0x0B:
		{
			if(workmode==10)      
			{	
				if(AppData->BuffSize == 7)
				{
					uint32_t frq = AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3];
					uint8_t dct  = AppData->Buff[4];
					uint16_t delay_time = AppData->Buff[5]<<8 | AppData->Buff[6];
					if(frq!=0)    
					{		
						gptimer_pwm_output(delay_time,frq,dct);
						rxpr_flags=1;	
					}	
				}				
			}				
			break;
		}
		
		case 0x0C:
		{
			if( AppData->BuffSize == 2 )      //AT+PWMSET
			{		
				pwm_timer= AppData->Buff[1];
				downlink_config_store_in_flash=1;
				rxpr_flags=1;
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
					rxpr_flags=1;							
				}						 
			 }
			 break;				
		}	
				
		case 0x21:
		{
			if( (AppData->BuffSize == 2) && (AppData->Buff[1]<=5) )
			{
				response_level=( AppData->Buff[1] );//0~5					//---->AT+RPL
				downlink_config_store_in_flash=1;	
				rxpr_flags=1;						
			}
			else if( (AppData->BuffSize == 3) && (AppData->Buff[1]==0x00) && (AppData->Buff[2]<=1))  //---->AT+DISMACANS
			{
				mac_response_flag=AppData->Buff[2];
				downlink_config_store_in_flash=1;	
				rxpr_flags=1;					
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
				rxpr_flags=1;							
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
					}
					else if(downlink_data_rate<2)
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
				 rxpr_flags=1;							 
			}
			break;				
		}			
				
		case 0x23:			
		{
			if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]!=0x00))
			{		
				lora_config_application_port_set(AppData->Buff[1]);    //---->AT+PORT
				downlink_config_store_in_flash=1;
				rxpr_flags=1;						
			}
			break;					
		}		
		
		case 0x24:
		{
			if( AppData->BuffSize == 2 )
			{
				#if defined( REGION_US915 )	|| defined( REGION_AU915 )
				if(AppData->Buff[1]<9)
				{
				  customize_set8channel_set(AppData->Buff[1]);
					downlink_config_store_in_flash=1;
					atz_flags=1;
					rxpr_flags=1;					
				}
				#elif defined( REGION_CN470 )
				if(AppData->Buff[1]<13)
				{
					customize_set8channel_set(AppData->Buff[1]);
					downlink_config_store_in_flash=1;
					atz_flags=1;
					rxpr_flags=1;					
				}
				#endif
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
				 rxpr_flags=1;						 
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
				rxpr_flags=1;					
			}			
			else if( AppData->BuffSize == 3 )
			{
				uint16_t value;			
						
				value=( AppData->Buff[1]<<8 | AppData->Buff[2] );//1~65535
						
				if(value>0)
				{
					REJOIN_TX_DUTYCYCLE=value;
					downlink_config_store_in_flash=1;
				  rxpr_flags=1;		
				}					
			}
			break;
		}
				
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
				rxpr_flags=1;						
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
				 rxpr_flags=1;						 
			 }
			 break;
		}
		
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
	
	LOG_PRINTF(LL_DEBUG,"\r\n");	
	LOG_PRINTF(LL_DEBUG,"Receive data\r\n");
	if((AppData->BuffSize<=8)&&(rxpr_flags==1))
	{			
		LOG_PRINTF(LL_DEBUG,"%d:",AppData->Port);
		for (int i = 0; i < AppData->BuffSize; i++)
		{
			LOG_PRINTF(LL_DEBUG,"%02x ", AppData->Buff[i]);
		}
		LOG_PRINTF(LL_DEBUG,"\r\n");
	}
	else
	{
		LOG_PRINTF(LL_DEBUG,"BuffSize:%d,Run AT+RECVB=? to see detail\r\n",AppData->BuffSize);
	}
	
	if((response_level!=0)&&(response_level!=3))
  {
		if(rxpr_flags==1)
		{
			downlink_send[0]=0x01;
		}
		else 
		{
			downlink_send[0]=0x00;
		}
		
		downlinklens=1;
		for (uint8_t g = 0; g < AppData->BuffSize; g++)
		{
			if(downlinklens<51)
			{
				downlink_send[downlinklens++]=AppData->Buff[g];
			}
		}
		
		is_time_to_reply_downlink=0;
		if((AppData->BuffSize==2)&&(AppData->Buff[0]==0x26)&&(AppData->Buff[1]==0x01))
		{
			is_there_data=0;
			MAC_COMMAND_ANS_status=0;
		}
		else
		{
			downlink_data_status=1;
		}		
	}	
}

static void OnTxTimerEvent( void )
{
	TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
	
	uplink_data_status=1;
	
	if((( LoRaMacState & 0x00000001 ) == 0x00000001)||(( LoRaMacState & 0x00000010 ) == 0x00000010))
	{
		LoRaMacState_error_times++;
	}
}

static void LoraStartTx(void)
{
  /* send everytime timer elapses */
  TimerInit( &TxTimer, OnTxTimerEvent );
  TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
  OnTxTimerEvent();
}

static void OnTxTimerEvent2( void )
{
	if(join_flag==0)
	{
		TimerSetValue( &TxTimer2,  1000);
		
		/*Wait for next tx slot*/
		TimerStart( &TxTimer2);
			
		join_flag++;
	}
	else if(join_flag==1)
	{
		TimerStop( &TxTimer2);	
		if(downlink_detect_switch==1)
		{
			if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
			{
				StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer();
				delay_ms(5);
			}
			StartDownlinkDetect();
			delay_ms(5);
		}
		LoraStartTx();
		join_flag=0;
	}
}

static void LoraStartjoin(void)
{
  /* send everytime timer elapses */
  TimerInit( &TxTimer2, OnTxTimerEvent2 );
  TimerSetValue( &TxTimer2, 1000); 
  OnTxTimerEvent2();
}

static void OnIWDGRefreshTimeoutEvent( void )
{
	TimerSetValue( &IWDGRefreshTimer,  18000);

  TimerStart( &IWDGRefreshTimer);
	
	is_time_to_IWDG_Refresh=1;
}

static void StartIWDGRefresh( void )
{
  /* send everytime timer elapses */
  TimerInit( &IWDGRefreshTimer, OnIWDGRefreshTimeoutEvent );
  TimerSetValue( &IWDGRefreshTimer,  18000); 
  TimerStart( &IWDGRefreshTimer);
}

void OndownlinkLedEvent(void)
{
	TimerStop(&downlinkLedTimer);
	gpio_write(LED_RGB_PORT,LED_RED_PIN,0);
	gpio_write(LED_RGB_PORT,LED_BLUE_PIN,0);
}

void OnNetworkJoinedLedEvent(void)
{
	gpio_init(LED_RGB_PORT, LED_GREEN_PIN, GPIO_MODE_OUTPUT_PP_LOW);	
	TimerStop(&NetworkJoinedLedTimer);
	joined_ledend=0;	
}

void OnPressButtonTimesLedEvent(void)
{
	TimerStop(&PressButtonTimesLedTimer);
	gpio_write(LED_RGB_PORT,LED_RED_PIN,0);
	gpio_write(LED_RGB_PORT,LED_GREEN_PIN,0);
	gpio_write(LED_RGB_PORT,LED_BLUE_PIN,0);
}

void OnPressButtonTimeoutEvent(void)
{
	TimerStop(&PressButtonTimeoutTimer);
	OnPressButtonTimeout_status=0;
	press_button_times=0;
}

static void OnReJoinTimerEvent( void )
{
	TimerStop( &ReJoinTimer);
	
	is_time_to_rejoin=1;
}

static void LoraStartRejoin(void)
{
  /* send everytime timer elapses */
  TimerInit( &ReJoinTimer, OnReJoinTimerEvent );
  TimerSetValue( &ReJoinTimer,  REJOIN_TX_DUTYCYCLE*60000); 
  TimerStart( &ReJoinTimer);
}

static void OnDownlinkDetectTimeoutEvent( void )
{
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
		rejoin_status=1;
	}
	
  /*Wait for next tx slot*/
  TimerStop( &DownlinkDetectTimeoutTimer);
}

static void StartDownlinkDetect(void)
{
  /* send everytime timer elapses */
  TimerInit( &DownlinkDetectTimeoutTimer, OnDownlinkDetectTimeoutEvent );
  TimerSetValue( &DownlinkDetectTimeoutTimer,  downlink_detect_timeout*60000); 
  TimerStart( &DownlinkDetectTimeoutTimer);
}

static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void )
{
	unconfirmed_uplink_change_to_confirmed_uplink_status=1;
	
  /*Wait for next tx slot*/
  TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
}

static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(void)
{
  /* send everytime timer elapses */
  TimerInit( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer, UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent );
  TimerSetValue( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,  unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
  TimerStart( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
}

void LoraStartCheckBLE(void)
{
  TimerInit( &CheckBLETimesTimer, OnCheckBLETimesEvent );
  TimerSetValue( &CheckBLETimesTimer,  60000); 
  TimerStart( &CheckBLETimesTimer);	
}

void OnCheckBLETimesEvent(void)
{
	if(gpio_read(DX_BT24_STATUS_PORT,DX_BT24_LINK_PIN)==1)
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

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  LOG_PRINTF(LL_DEBUG,"switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  LoRaMacState_error_times=0;
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void send_exti_pa4(void)
{
  if(gpio_read(GPIOA, GPIO_PIN_4)== wakeup_pa4_flag)  //When the level changes, the wake-up method also changes
	{
		bool wakeup_a4_mode;		
		wakeup_a4_mode=gpio_read(GPIOA, GPIO_PIN_4);
		if(wakeup_a4_mode==1)
		{
			wakeup_a4_mode=0;
			wakeup_pa4_flag=0;
		}
		else
		{
			wakeup_a4_mode=1;
			wakeup_pa4_flag=1;
		}
		
		if((workmode==7)||(workmode==9))
		{
			gpio_config_stop3_wakeup(GPIOA, GPIO_PIN_4 ,true,wakeup_a4_mode);
		}
		else
		{
			gpio_config_stop3_wakeup(GPIOA, GPIO_PIN_4 ,false,wakeup_a4_mode);
		}
	}		
}

static void send_exti_pa8(void)
{
  if(gpio_read(GPIOA, GPIO_PIN_8)== wakeup_pa8_flag)  //When the level changes, the wake-up method also changes
	{
		bool wakeup_a8_mode;		
		wakeup_a8_mode=gpio_read(GPIOA, GPIO_PIN_8);
		if(wakeup_a8_mode==1)
		{
			wakeup_a8_mode=0;
			wakeup_pa8_flag=0;
		}
		else
		{
			wakeup_a8_mode=1;
			wakeup_pa8_flag=1;
		}
		
		if((workmode!=3)&&(workmode!=8))	
		{			
			gpio_config_stop3_wakeup(GPIOA, GPIO_PIN_8 ,true,wakeup_a8_mode);
		}
		else
		{
			gpio_config_stop3_wakeup(GPIOA, GPIO_PIN_8 ,false,wakeup_a8_mode);
		}
	}	
}

static void send_exti_pb15(void)
{
  if(gpio_read(GPIOB, GPIO_PIN_15)== wakeup_pb15_flag)  //When the level changes, the wake-up method also changes
	{
		bool wakeup_b15_mode;		
		wakeup_b15_mode=gpio_read(GPIOB, GPIO_PIN_15);
		if(wakeup_b15_mode==1)
		{
			wakeup_b15_mode=0;
			wakeup_pb15_flag=0;
		}
		else
		{
			wakeup_b15_mode=1;
			wakeup_pb15_flag=1;
		}
		
		if((workmode==3)||(workmode==7)||(workmode==8)||(workmode==9))	
		{			
			gpio_config_stop3_wakeup(GPIOB, GPIO_PIN_15 ,true,wakeup_b15_mode);
		}
		else
		{
			gpio_config_stop3_wakeup(GPIOB, GPIO_PIN_15 ,false,wakeup_b15_mode);
		}
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
		gpio_write(LED_RGB_PORT, LED_GREEN_PIN, 1);
			
		TimerTime_t currentTime = TimerGetCurrentTime();
		
		while(gpio_read(GPIO_USERKEY_PORT,GPIO_USERKEY_PIN)==GPIO_LEVEL_LOW)
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
					gpio_toggle(LED_RGB_PORT, LED_GREEN_PIN);
					delay_ms(100);
				}
				user_key_duration=3;
				break;
			}			
    }
		
		gpio_write(LED_RGB_PORT, LED_GREEN_PIN, 0);
		
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
				
				rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
				gpio_set_iomux(GPIOD, GPIO_PIN_10, 0);
				gpio_init(GPIOD, GPIO_PIN_10, GPIO_MODE_OUTPUT_PP_LOW); 
			  delay_ms(200);	 
			  gpio_init(GPIOD, GPIO_PIN_10, GPIO_MODE_OUTPUT_PP_HIGH); 					
			  TimerSetValue( &CheckBLETimesTimer,  60000); 
				TimerStart( &CheckBLETimesTimer);			
				ble_sleep_flags=0;
				
				if(sleep_status==0 && (LORA_JoinStatus () == LORA_SET) && (( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					TimerStop(&TxTimer);		
					TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
					TimerStart(&TxTimer);					
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
        TimerStop(&CheckBLETimesTimer);				
				TimerStop(&TxTimer);
				TimerStop(&ReJoinTimer);
				TimerStop(&DownlinkDetectTimeoutTimer);
				TimerStop(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
				
				delay_ms(500);
				TimerInit( &PressButtonTimesLedTimer, OnPressButtonTimesLedEvent );
				TimerSetValue( &PressButtonTimesLedTimer, 5000);
				gpio_write(LED_RGB_PORT,LED_RED_PIN,GPIO_LEVEL_HIGH); 
				TimerStart( &PressButtonTimesLedTimer );
				
				ble_sleep_command=0;
				ble_sleep_flags=1;
				POWER_IoDeInit();	
        delay_ms(50);	 			
			  LOG_PRINTF(LL_DEBUG,"AT+PWRM2\r\n");
				delay_ms(100);	 
				LOG_PRINTF(LL_DEBUG,"SLEEP\r\n");				
				gpio_init(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN, GPIO_MODE_ANALOG);
				gpio_config_stop3_wakeup(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN ,false,GPIO_LEVEL_HIGH);		
				gpio_init(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN, GPIO_MODE_ANALOG);
				gpio_config_stop3_wakeup(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN ,false,GPIO_LEVEL_HIGH);	
				gpio_init(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN, GPIO_MODE_ANALOG);
				gpio_config_stop3_wakeup(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN ,false,GPIO_LEVEL_HIGH);	
				if(workmode==10)
				{
					gptimer_pwm_Iodeinit();
				}
				joined_finish=0;
				user_key_duration=0;
								
				break;
			}
			
			case 3://system reset,Activation Mode
			{
				user_key_duration=0;
				system_reset();
				break;
			}
			
			default:
				break;
		}
	}
}

uint8_t HW_GetBatteryLevel( void ) 
{		
	  uint8_t batteryLevel=0;
	  uint16_t bat_mv=0;
	
		bat_mv=battery_voltage_measurement();

    if (bat_mv >= 3600)
    {
       batteryLevel = 254;
    }
		else if (bat_mv < 2405)
		{
			batteryLevel = 1;
		}
		else
		{
			batteryLevel = (( (uint32_t) (bat_mv - 2400)*254) /(3600-2400) ); 
		}
		return batteryLevel;
}

uint16_t HW_GetTemperatureLevel( void ) 
{  
  return 0;
}

void HW_GetUniqueId( uint8_t *id )
{
	  uint32_t unique_id[2];
	  system_get_chip_id(unique_id);
	
    id[7] = unique_id[0]>>24&0xFF;
    id[6] = unique_id[0]>>16&0xFF;
    id[5] = unique_id[0]>>8&0xFF;
    id[4] = unique_id[0]&0xFF;
    id[3] = unique_id[1]>>24&0xFF;
    id[2] = unique_id[1]>>16&0xFF;
    id[1] = unique_id[1]>>8&0xFF;
    id[0] = unique_id[1]&0xFF;
}

uint32_t HW_GetRandomSeed( void )
{
		uint32_t unique_id[2];
		system_get_chip_id(unique_id);
		
		return unique_id[0]^unique_id[1];
}

void check_pin_status(void)
{
	if(inmode!=0)
	{
		wakeup_pa8_flag=gpio_read(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN);
	}
	else
	{
		gpio_config_stop3_wakeup(GPIO_EXTI8_PORT, GPIO_EXTI8_PIN,false,GPIO_LEVEL_HIGH);			
	}
	
	if(inmode2!=0)
	{	
		wakeup_pa4_flag=gpio_read(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN);
	}
	else
	{
		gpio_config_stop3_wakeup(GPIO_EXTI4_PORT, GPIO_EXTI4_PIN,false,GPIO_LEVEL_HIGH);					
	}
	
	if(inmode3!=0)
	{		
		wakeup_pb15_flag=gpio_read(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN);
	}
	else
	{
		gpio_config_stop3_wakeup(GPIO_EXTI15_PORT, GPIO_EXTI15_PIN,false,GPIO_LEVEL_HIGH);			
	}
	
	if(ble_sleep_flags==1)
	{
		if((gpio_read(DX_BT24_STATUS_PORT,DX_BT24_LINK_PIN)==1)||(gpio_read(DX_BT24_STATUS_PORT,DX_BT24_WORK_PIN)==1))
		{
			delay_ms(50);					
			LOG_PRINTF(LL_DEBUG,"AT+PWRM2\r\n");			
		}
	}	
}

#ifdef USE_FULL_ASSERT
void assert_failed(void* file, uint32_t line)
{
    (void)file;
    (void)line;

    while (1) { }
}
#endif
