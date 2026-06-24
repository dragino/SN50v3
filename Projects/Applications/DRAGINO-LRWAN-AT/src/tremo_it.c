#include "tremo_lpuart.h"
#include "tremo_it.h"
#include "tremo_gpio.h"
#include "lora_config.h"
#include "tremo_uart.h"
#include "tfsensor.h"
#include "tremo_timer.h"
#include "timer.h"

uint16_t IC1[4],IC2[4];
uint16_t IC1Value=0,IC2Value=0;
uint8_t RxBuffer[1];
extern uint8_t icnumber;
extern bool exti_flag,exti2_flag,exti3_flag;
extern uint8_t user_key_exti_flag;
extern bool join_network;
extern uint8_t inmode,inmode2,inmode3;
extern uint32_t count1,count2;
extern uint8_t workmode;
extern uint16_t inmode_delay,inmode2_delay,inmode3_delay;
extern bool uplink_pin_status_pb15,uplink_pin_status_pa4,uplink_pin_status_pa8;

void LoraStartdelay1(void);
void LoraStartdelay2(void);
void LoraStartdelay3(void);
static uint8_t status1;
static uint8_t status2;
static uint8_t status3;
static void Onexdelay1TimerEvent( void );
static void Onexdelay2TimerEvent( void );
static void Onexdelay3TimerEvent( void );
static TimerEvent_t exdelay1Timer;
static TimerEvent_t exdelay2Timer;
static TimerEvent_t exdelay3Timer;

extern void RadioOnDioIrq(void);
extern void RtcOnIrq(void);
extern void linkwan_serial_input(uint8_t cmd);
extern void dma0_IRQHandler(void);
extern void dma1_IRQHandler(void);
/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{

    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
}

/**
 * @brief  This function handles PWR Handler.
 * @param  None
 * @retval None
 */
void PWR_IRQHandler()
{
}

/******************************************************************************/
/*                 Tremo Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_cm4.S).                                               */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
void LORA_IRQHandler()
{
    RadioOnDioIrq();
}

void RTC_IRQHandler(void)
{
    RtcOnIrq();
}

void UART0_IRQHandler(void)
{
}

void UART2_IRQHandler(void)
{
	if(uart_get_interrupt_status(UART2,UART_INTERRUPT_RX_DONE) == SET)
	{
		RxBuffer[0]=UART2->DR & 0xFF;
		HAL_UART_RxCallback(RxBuffer);
		uart_clear_interrupt(UART2,UART_INTERRUPT_RX_DONE);
	}
}

void LPUART_IRQHandler(void)
{
    if (lpuart_get_rx_status(LPUART, LPUART_SR0_RX_DONE_STATE)) {
        uint8_t rx_data_temp = lpuart_receive_data(LPUART);
        lpuart_clear_rx_status(LPUART, LPUART_SR0_RX_DONE_STATE);
        linkwan_serial_input(rx_data_temp);
    }
}
/**
 * @brief  This function handles dma0 Handler.
 * @param  None
 * @retval None
 */
void DMA0_IRQHandler(void)
{
    dma0_IRQHandler();
}

/**
 * @brief  This function handles dma1 Handler.
 * @param  None
 * @retval None
 */
void DMA1_IRQHandler(void)
{
    dma1_IRQHandler();
}

void TIMER1_IRQHandler(void)
{
  bool state;
  timer_get_status(TIMER1, TIMER_SR_CC0IF, &state);
  if (state) 
	{
    timer_clear_status(TIMER1, TIMER_SR_CC0IF);	  		
		IC1Value = TIMER1->CCR0;
		IC2Value = TIMER1->CCR1;
		if(icnumber<4)
		{
			IC1[icnumber]=IC1Value;
			IC2[icnumber]=IC2Value;
			icnumber++;
		}
  }
}
/******************************************************************************/
/*                 Tremo Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_cm4.S).                                               */
/******************************************************************************/
void GPIO_IRQHandler(void)
{	
  if (gpio_get_interrupt_status(GPIOA, GPIO_PIN_8) == SET) {
			if((inmode!=0)&&(join_network==1))	
			{	
				if(inmode_delay==0)
				{				
				if((workmode==6)||(workmode==9)||(workmode==12))
				{
					if((inmode==2)||(inmode==3))
					{
						count1++;
					}
				}
				else
				{
					if(inmode==4)
					{
						if(uplink_pin_status_pa8!=gpio_read(GPIOA, GPIO_PIN_8))
						{
							exti_flag=1;
						}
					}
					else
					{
						exti_flag=1;
					}
				}
				}
				else
				{
						status1=gpio_read(GPIOA, GPIO_PIN_8);
						TimerSetValue(&exdelay1Timer,inmode_delay); 
						TimerStart(&exdelay1Timer);				
				}
			}				
		  gpio_clear_interrupt(GPIOA, GPIO_PIN_8);	
	}	
	
  if (gpio_get_interrupt_status(GPIOA, GPIO_PIN_4) == SET) {	
			if((inmode2!=0)&&(join_network==1))	
			{		
				if(inmode2_delay==0)
				{
				if(workmode==7)
				{			
					if(inmode2==4)
					{
						if(uplink_pin_status_pa4!=gpio_read(GPIOA, GPIO_PIN_4))
						{
							exti2_flag=1;
						}
					}
					else
					{					
						exti2_flag=1;
					}
				}
				else if((workmode==9)&&((inmode2==2)||(inmode2==3)))
				{
				 count2++;					
				}
				}
				else
				{
						status2=gpio_read(GPIOA, GPIO_PIN_4);
						TimerSetValue(&exdelay2Timer,inmode2_delay); 
						TimerStart(&exdelay2Timer);			
				}
			}				
		  gpio_clear_interrupt(GPIOA, GPIO_PIN_4);	
	}	

  if (gpio_get_interrupt_status(GPIOB, GPIO_PIN_15) == SET) {	
			if((inmode3!=0)&&(join_network==1))	
			{	
			 if(inmode3_delay==0)
			 {				
			 if((workmode==3)||(workmode==7)||(workmode==8)||(workmode==9))
			 {
				if(inmode3==4)
				{
					if(uplink_pin_status_pb15!=gpio_read(GPIOB, GPIO_PIN_15))
					{
						exti3_flag=1;
					}
				}
				else
				{
					exti3_flag=1;
				}
			 }
			 }
			 else
			 {
						status3=gpio_read(GPIOB, GPIO_PIN_15);
						TimerSetValue(&exdelay3Timer,inmode3_delay); 
						TimerStart(&exdelay3Timer);			 
			 }
			}				
		  gpio_clear_interrupt(GPIOB, GPIO_PIN_15);	
	}	
	
  if (gpio_get_interrupt_status(GPIOC, GPIO_PIN_8) == SET) {	
			user_key_exti_flag=1;	
		  gpio_clear_interrupt(GPIOC, GPIO_PIN_8);	
	}		
}


static void Onexdelay1TimerEvent( void )
{
	TimerStop( &exdelay1Timer);
	if(gpio_read(GPIOA, GPIO_PIN_8)==status1)
	{
		if((workmode==6)||(workmode==9)||(workmode==12))
		{
			if((inmode==2)||(inmode==3))
			{
				count1++;
			}
		}
		else
		{
			if(inmode==4)
			{
				if(uplink_pin_status_pa8!=status1)
				{
					exti_flag=1;
				}
			}
			else
			{	
				exti_flag=1;
			}
		}		  
	}
}

void LoraStartdelay1(void)
{
   TimerInit( &exdelay1Timer, Onexdelay1TimerEvent );
}

static void Onexdelay2TimerEvent( void )
{
	TimerStop( &exdelay2Timer);
	if(gpio_read(GPIOA, GPIO_PIN_4)==status2)
	{
		if(workmode==7)
		{	
			if(inmode2==4)
			{
				if(uplink_pin_status_pa4!=status2)
				{
					exti2_flag=1;
				}
			}
			else
			{			
				exti2_flag=1;
			}
		}
		else if((workmode==9)&&((inmode2==2)||(inmode2==3)))
		{
		  count2++;					
		}	
	}
}

void LoraStartdelay2(void)
{
   TimerInit( &exdelay2Timer, Onexdelay2TimerEvent );
}

static void Onexdelay3TimerEvent( void )
{
	TimerStop( &exdelay3Timer);
	if(gpio_read(GPIOB, GPIO_PIN_15)==status3)
	{
	  if((workmode==3)||(workmode==7)||(workmode==8)||(workmode==9))
	  {        				
			if(inmode3==4)
			{
				if(uplink_pin_status_pb15!=status3)
				{
					exti3_flag=1;
				}
			}
			else
			{
				exti3_flag=1;
			}
		}		
	}
}

void LoraStartdelay3(void)
{
   TimerInit( &exdelay3Timer, Onexdelay3TimerEvent );
}