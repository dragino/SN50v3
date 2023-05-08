#include "stdio.h"	
#include "tfsensor.h"
#include "lora_config.h"
#include "tremo_delay.h"
#include "tremo_uart.h"
#include "gpio_exti.h"
#include "tremo_iwdg.h"
#include "log.h"

uint8_t response[1];
uint8_t num=0;
uint8_t num_check=0;
bool flags_command_ser=0;
bool flags_command_check=0;
uint8_t response_data[120]={0x00};
uint8_t rxdatacheck[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void send_uart_data(uint8_t *txdata,uint8_t txdatalen)
{
	for(uint8_t i=0;i<txdatalen;i++)
	{
		uart_send_data(UART2,(unsigned char)txdata[i]);
				
		while (uart_get_flag_status(UART2, UART_FLAG_BUSY) == SET)
		;			
	}		
}

void HAL_UART_RxCallback(uint8_t *rxbuff)
{		
	response[0]=rxbuff[0];
	if(flags_command_ser==1)	
	{
	  response_data[num++]= response[0];
		if(num==120)
		{
			flags_command_ser=0;
		}
	}
	else if(flags_command_check==1)
	{
		rxdatacheck[num_check++]= response[0];
		if(num_check==7)
		{
			flags_command_check=0;
		}
	}	
}

void uart2_IoInit(void)
{
	UART2_RCC_ENABLE();
	USAR2_CLK_ENABLE();	
	
  gpio_set_iomux(USAR2_TX_GPIO_PORT, USAR2_TX_PIN, USAR2_TX_RX_MUX);
	gpio_set_iomux(USAR2_RX_GPIO_PORT, USAR2_RX_PIN, USAR2_TX_RX_MUX);
	
	/* uart config struct init */
  uart_config_t uart_config;

	uart_config.baudrate = UART_BAUDRATE_115200;
	uart_config.data_width = UART_DATA_WIDTH_8;		
	uart_config.parity = UART_PARITY_NO;
	uart_config.stop_bits = UART_STOP_BITS_1;	
	uart_config.mode = UART_MODE_TXRX;
  uart_config.flow_control = UART_FLOW_CONTROL_DISABLED;
	uart_config.fifo_mode = DISABLE;
	
  uart_init(UART2, &uart_config);	
	uart_cmd(UART2, ENABLE);
	
	uart_config_interrupt(UART2,UART_INTERRUPT_RX_DONE,ENABLE);		
	
	NVIC_SetPriority(UART2_IRQn, 2);
	NVIC_EnableIRQ(UART2_IRQn);	
}

void uartsersion1_IoInit(uint8_t mode)
{
	USAR2_CLK_ENABLE();
	if(mode==1)
	{
		gpio_set_iomux(USAR2_TX_GPIO_PORT, USAR2_TX_PIN, USAR2_TX_RX_MUX);
		gpio_set_iomux(USAR2_RX_GPIO_PORT, USAR2_RX_PIN, USAR2_TX_RX_MUX);	
	}
	else
	{
		gpio_set_iomux(USAR2_TX_GPIO_PORT, USAR2_TX_PIN, 0);		
		gpio_init(USAR2_TX_GPIO_PORT, USAR2_TX_PIN, GPIO_MODE_ANALOG);			
		gpio_set_iomux(USAR2_RX_GPIO_PORT, USAR2_RX_PIN, 0);		
		gpio_init(USAR2_RX_GPIO_PORT, USAR2_RX_PIN, GPIO_MODE_ANALOG);
	}		
}

void BSP_tfsensor_Init(void)
{
	uint8_t txdisoutput[5]={0x5A,0x05,0x07,0x00,0x66};   
	uint8_t txlowpower[6] ={0x5A,0x06,0x35,0x01,0x00,0x96};  //1HZ
	uint8_t txsave[4]     ={0x5A,0x04,0x11,0x6f};

  uart2_IoInit();
	uartsersion1_IoInit(1);
	POWER_IoInit();
  delay_ms(1000);
  send_uart_data(txdisoutput,5);	
	delay_ms(50);
  send_uart_data(txlowpower,6);
	delay_ms(50);	
  send_uart_data(txsave,4);		
	delay_ms(500); 
}

uint8_t check_deceive(void)
{
	uint8_t temp_flag=0;
	uint8_t txID[4]={0x5A,0x04,0x01,0x5f};
	
	BSP_tfsensor_Init();

	send_uart_data(txID, 4);	
	flags_command_check=1;		
  delay_ms(500);	
	flags_command_check=0; 
	
	for(uint8_t i=0;i<7;i++)
  {
		if(rxdatacheck[i]!=0x00)
		{
			temp_flag=1;
			break;
		}
	}

	uartsersion1_IoInit(0);	
	POWER_IoDeInit();
	
	if(temp_flag==1)
	{	
		return 1;
	}	
	else
	{
		return 0;
	}
}

void tfsensor_read_distance(tfsensor_reading_t *tfsensor_reading)
{
	uint8_t rxdata_dis[9] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	
	read_distance(rxdata_dis);
	
	if((rxdata_dis[0] == 0x00)&&(rxdata_dis[1] == 0x00)&&(rxdata_dis[8] == 0x00)&&
		 (rxdata_dis[2] == 0x00)&&(rxdata_dis[3] == 0x00)&(rxdata_dis[4] == 0x00)&&(rxdata_dis[5] == 0x00)&&(rxdata_dis[6] == 0x00)&&(rxdata_dis[7] == 0x00))
	{
		tfsensor_reading->distance_cm = 0;
		tfsensor_reading->distance_signal_strengh = 65534;	
		tfsensor_reading->temperature = 0;				
	}
	else 
	{	
		tfsensor_reading->distance_cm = ((rxdata_dis[3]<<8)+rxdata_dis[2])*10;
		tfsensor_reading->distance_signal_strengh = ((rxdata_dis[5]<<8)+rxdata_dis[4]);
		tfsensor_reading->temperature = (int)((rxdata_dis[7]*256+rxdata_dis[6])/8-256);
	}
}

void read_distance(uint8_t rxdata10[])
{
	POWER_IoInit();
	uartsersion1_IoInit(1);
  delay_ms(1000);				
	for(int i=0;i<3;i++)
  {
		at_tfmini_data_receive(rxdata10,2000);
		if((getCheckSum(rxdata10,8)==rxdata10[8])&&(((rxdata10[3]<<8)+rxdata10[2]))!=0)
		{
		  break;
		}	
		iwdg_reload();		
	}
	uartsersion1_IoInit(0);	
	POWER_IoDeInit();
}

uint8_t getCheckSum(uint8_t *pack, uint8_t pack_len)
{
	uint8_t  i,check_sum=0;
	
  for(i=0; i<pack_len; i++)
  {
    check_sum += *(pack++);
  }
	
	return check_sum;
}

void at_tfmini_data_receive(uint8_t rxdatatemp[],uint16_t delayvalue)
{
	uint8_t responsetemp[1]={0x00};	
	uint8_t begin=0,datanumber=2,data_no=1;
	uint8_t txenoutput[5] ={0x5A,0x05,0x07,0x01,0x67};		  

	num=0;	
	send_uart_data(txenoutput, 5);			
	flags_command_ser=1;
	delay_ms(delayvalue);		
	flags_command_ser=0;
	
	for(uint8_t number=0;number<sizeof(response_data);number++)
	{
		if(begin==1)
		{
			rxdatatemp[datanumber++]=response_data[number];
			if(datanumber==9)
			{
				rxdatatemp[0]=0x59;
				rxdatatemp[1]=0x59;
				begin=2;
			}
		}	
		
		if((responsetemp[0]==0x59)&&(begin==0))
		{
			if(response_data[number]==0x59)
			{
				if(data_no==2)
				{
					begin=1;
				}
				data_no++;
			}
			else
			{
				responsetemp[0]=0x00;
			}
		}	
		else if(response_data[number]==0x59&&(begin==0))
		{
			responsetemp[0]=0x59;
		}
		
		response_data[number]=0x00;
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
