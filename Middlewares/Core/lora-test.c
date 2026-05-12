 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    lora-test.c
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
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "lora_app.h"
#include "lora-test.h"
#include "timer.h"
#include "stm32wlxx_hal.h"
#include "mw_log_conf.h"
#include "iwdg.h"

bool disport=0;
bool test_adr_flag=0,ce_rtc_flag=0;
uint8_t port_temp,lora_rx_test=0;
uint8_t certif_status_temp;
extern bool join_mothod;
extern TimerEvent_t TxTimer;
extern bool joined_finish;
extern uint16_t LoRaMacDevNonce;
extern uint32_t UpLinkCounter;
extern void lora_config_reqack_set(LoraConfirm_t reqack);
extern uint8_t lora_config_application_port_get(void );
extern __IO uint8_t is_time_to_send;
extern uint32_t LoRaMacState;
extern void save_devNonce_buff(void);

/* Private typedef -----------------------------------------------------------*/
typedef struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    LoraConfirm_t IsTxConfirmed;
    uint8_t DataBufferSize;
    uint8_t DataBuffer[242];
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest_t;

/* Private define ------------------------------------------------------------*/
uint32_t TEST_TX_DUTYCYCLE=5000;
/* Private variables ---------------------------------------------------------*/

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t CertifTxNextPacketTimer;
static ComplianceTest_t certifParam;
static LoraConfirm_t IsTxConfirmed;
static bool AdrEnableInit;

/* Private functions ---------------------------------------------------------*/

static void OnCertifTxNextPacketTimerEvent( void *context );
static bool certif_tx( void );

/* Exported functions definition---------------------------------------------------------*/
bool certif_running(void)
{
    return certifParam.Running;
}

void certif_DownLinkIncrement( void )
{
    certifParam.DownLinkCounter++;
}

void certif_linkCheck(MlmeConfirm_t *mlmeConfirm)
{
  certifParam.LinkCheck = true;
  certifParam.DemodMargin = mlmeConfirm->DemodMargin;
  certifParam.NbGateways = mlmeConfirm->NbGateways;
}

static bool certif_tx( void )
{
  McpsReq_t mcpsReq;
  LoRaMacTxInfo_t txInfo;
  
  if( certifParam.LinkCheck == true )
  {
    certifParam.LinkCheck = false;
    certifParam.DataBufferSize = 3;
    certifParam.DataBuffer[0] = 5;
    certifParam.DataBuffer[1] = certifParam.DemodMargin;
    certifParam.DataBuffer[2] = certifParam.NbGateways;
  }
  else
  {
    switch( certif_status_temp )
    {
			case 0x08:
				port_temp=CERTIF_PORT;
				certifParam.DataBuffer[0] = 0x08;
		  	certif_status_temp=0x06;
				break;
			case 0x09:
				port_temp =CERTIF_PORT;
				certifParam.DataBuffer[0] = 0x09;
				certifParam.DataBuffer[1] = certifParam.DownLinkCounter ;
				certifParam.DataBuffer[2] = certifParam.DownLinkCounter >> 8;
			  certif_status_temp=0x06;
				break;
			case 0x7f:
				port_temp=CERTIF_PORT;
			  certifParam.DataBufferSize = 13;
				certifParam.DataBuffer[0] = 0x7f;
			  certif_status_temp=0x06;
				break;
			default: 		
				port_temp = lora_config_application_port_get();
				certifParam.DataBufferSize = 11;
				certifParam.DataBuffer[0] = 0x00;
				certifParam.DataBuffer[1] = 0x01;
				certifParam.DataBuffer[2] = 0x02;
				certifParam.DataBuffer[3] = 0x03;
				certifParam.DataBuffer[4] = 0x04;
				certifParam.DataBuffer[5] = 0x05;
				certifParam.DataBuffer[6] = 0x06;
				certifParam.DataBuffer[7] = 0x07;
				certifParam.DataBuffer[8] = 0x08;
				certifParam.DataBuffer[9] = 0x09;
				certifParam.DataBuffer[10] = 0x0a;			
				certif_status_temp=0x06;	
				break;
    }
  }
    
  if( LoRaMacQueryTxPossible( certifParam.DataBufferSize, &txInfo ) != LORAMAC_STATUS_OK )
  {
      // Send empty frame in order to flush MAC commands
      mcpsReq.Type = MCPS_UNCONFIRMED;
      mcpsReq.Req.Unconfirmed.fBuffer = NULL;
      mcpsReq.Req.Unconfirmed.fBufferSize = 0;
      mcpsReq.Req.Unconfirmed.Datarate = DR_0;
  }
  else
  {
      if( IsTxConfirmed == LORAWAN_UNCONFIRMED_MSG )
      {
          mcpsReq.Type = MCPS_UNCONFIRMED;
          mcpsReq.Req.Unconfirmed.fPort = port_temp;
          mcpsReq.Req.Unconfirmed.fBufferSize = certifParam.DataBufferSize;
          mcpsReq.Req.Unconfirmed.fBuffer = &(certifParam.DataBuffer);
          mcpsReq.Req.Unconfirmed.Datarate = DR_0;
      }
      else
      {
          mcpsReq.Type = MCPS_CONFIRMED;
          mcpsReq.Req.Confirmed.fPort = port_temp;
          mcpsReq.Req.Confirmed.fBufferSize = certifParam.DataBufferSize;
          mcpsReq.Req.Confirmed.fBuffer = &(certifParam.DataBuffer);
          mcpsReq.Req.Confirmed.Datarate = DR_0;
      }
  }
	
  if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
  {
      return false;
  }
    return true;
}

void certif_rx( McpsIndication_t *mcpsIndication, MlmeReqJoin_t* JoinParameters)
{
	if(disport==0)
	{
    certifParam.State = mcpsIndication->Buffer[0];
		certif_status_temp = certifParam.State;
	  MW_LOG(TS_OFF, VLEVEL_M, "S:%02x\r\n",certifParam.State);
	
    switch( certifParam.State )
    {	
		  case 0x01:
			{
				if( mcpsIndication->BufferSize == 1 )
				{
					iwdg_reload();
					HAL_Delay(6000);
					NVIC_SystemReset();
				}
         break;
			}
				
      case 0x02: // (ix)
      {		
				if(( mcpsIndication->BufferSize == 1 )&&(join_mothod==1))
				{		
					if(ce_rtc_flag==1)
					{
						TimerStop( &CertifTxNextPacketTimer );	
						ce_rtc_flag=0;
					}		
					
					iwdg_reload();	
					HAL_Delay(6000);
					
					joined_finish=0;
					
					MlmeReq_t mlmeReq;

					// Disable TestMode and revert back to normal operation

					certifParam.DownLinkCounter = 0;
					certifParam.Running = false;

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = AdrEnableInit;
					LoRaMacMibSetRequestConfirm( &mibReq );

					mlmeReq.Type = MLME_JOIN;
					mlmeReq.Req.Join = *JoinParameters;
					
					LoRaMacDevNonce++;
					save_devNonce_buff();
					HAL_Delay(100);
					LoRaMacMlmeRequest( &mlmeReq );
         }
         break;
       }		
				
			 case 0x04:
			 {
				 if( mcpsIndication->BufferSize == 2 )
					{
						if(mcpsIndication->Buffer[1]==0x00)
						{		
							MibRequestConfirm_t mibReq;
							mibReq.Type = MIB_ADR;
							mibReq.Param.AdrEnable = false;
							LoRaMacMibSetRequestConfirm( &mibReq );						
						}					
						else if(mcpsIndication->Buffer[1]==0x01)
						{		
							MibRequestConfirm_t mibReq;
							mibReq.Type = MIB_ADR;
							mibReq.Param.AdrEnable = true;
							LoRaMacMibSetRequestConfirm( &mibReq );						
						}	
					}
          break;						
				}
				
        case 0x05: // (viii)
        {
					if( mcpsIndication->BufferSize == 2 )
					{
						if(mcpsIndication->Buffer[1]==0x00)
						{							
							LoRaMacTestSetDutyCycleOn( false );
						}
						else if(mcpsIndication->Buffer[1]==0x01)
						{							
							LoRaMacTestSetDutyCycleOn( true );
						} 
					}
					break;	
				}	
				
				case 0x06:
				{
					if( mcpsIndication->BufferSize == 2 )
					{
						if(mcpsIndication->Buffer[1]!=0)
						{
							uint8_t timelevel;
							bool txrtc_flag=0;
							
							timelevel=mcpsIndication->Buffer[1];
						
							MibRequestConfirm_t mibReq;
							IsTxConfirmed = LORAWAN_UNCONFIRMED_MSG;
							certifParam.DataBufferSize = 2;
							certifParam.DownLinkCounter = 0;
							certifParam.LinkCheck = false;
							certifParam.DemodMargin = 0;
							certifParam.NbGateways = 0;
							certifParam.Running = true;

							mibReq.Type = MIB_ADR;

							LoRaMacMibGetRequestConfirm( &mibReq );
							AdrEnableInit=mibReq.Param.AdrEnable;
          
							mibReq.Type = MIB_ADR;
							mibReq.Param.AdrEnable = true;
							LoRaMacMibSetRequestConfirm( &mibReq );

							switch(timelevel)
							{
								case 1:
								TEST_TX_DUTYCYCLE=5000;
								txrtc_flag=1;
								break;
								case 2:
								TEST_TX_DUTYCYCLE=10000;
								txrtc_flag=1;
								break;
								case 3:
								TEST_TX_DUTYCYCLE=20000;
								txrtc_flag=1;
								break;
								case 4:
								TEST_TX_DUTYCYCLE=30000;
								txrtc_flag=1;								
								break;
								case 5:
								TEST_TX_DUTYCYCLE=40000;
								txrtc_flag=1;								
								break;
								case 6:
								TEST_TX_DUTYCYCLE=50000;
								txrtc_flag=1;								
								break;
								case 7:
								TEST_TX_DUTYCYCLE=60000;
								txrtc_flag=1;								
								break;
								case 8:
								TEST_TX_DUTYCYCLE=120000;
								txrtc_flag=1;								
								break;
								case 9:
								TEST_TX_DUTYCYCLE=240000;
								txrtc_flag=1;								
								break;
								case 10:
								TEST_TX_DUTYCYCLE=480000;
								txrtc_flag=1;								
								break;
								default:                  
								break;							
							}
              if(txrtc_flag==1)
							{
								TimerStop(&TxTimer); 
								TimerInit( &CertifTxNextPacketTimer, OnCertifTxNextPacketTimerEvent );
								TimerSetValue( &CertifTxNextPacketTimer,  TEST_TX_DUTYCYCLE); 
								TimerStart( &CertifTxNextPacketTimer);								
                ce_rtc_flag=1;								
								test_adr_flag=1;
							}								
						}
						else 
						{
								certifParam.DownLinkCounter = 0;
								certifParam.Running = false;
          
								MibRequestConfirm_t mibReq;
								mibReq.Type = MIB_ADR;
								mibReq.Param.AdrEnable = AdrEnableInit;
								LoRaMacMibSetRequestConfirm( &mibReq );
					
								#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
								LoRaMacTestSetDutyCycleOn( true );
								#endif 
							  if(ce_rtc_flag==1)
								{
									TimerStop( &CertifTxNextPacketTimer );	
									ce_rtc_flag=0;
								}
								TimerStart(&TxTimer);
								is_time_to_send=1;								
								test_adr_flag=0;
					   }		
					}
					break;
				}
				
        case 0x07: // Enable confirmed messages (v)
				{
					if( mcpsIndication->BufferSize == 2 )
					{
						if(mcpsIndication->Buffer[1]==0x01)
						{					
							IsTxConfirmed=LORAWAN_UNCONFIRMED_MSG;
						}
						else if(mcpsIndication->Buffer[1]==0x02)
						{					
							IsTxConfirmed=LORAWAN_CONFIRMED_MSG;
						}
					}
          break;		
				}
				
        case 0x08:
			  {
          certifParam.DataBufferSize = mcpsIndication->BufferSize;
					for( uint8_t i = 1; i < certifParam.DataBufferSize; i++ )
					{
						certifParam.DataBuffer[i] = mcpsIndication->Buffer[i] + 1;
					}
          break;					
				}
				
				case 0x09:
				{
					if( mcpsIndication->BufferSize == 1 )
					{					
						certifParam.DataBufferSize = 3;					
					}								
					break;
				}
				
				case 0x0a:
				{
					if( mcpsIndication->BufferSize == 1 )
					{					
						certifParam.DownLinkCounter = 0;  	
					}				
					break;
				}			
				
				case 0x20:
				{
					if( mcpsIndication->BufferSize == 1 )
					{	
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_LINK_CHECK;
						LoRaMacMlmeRequest( &mlmeReq );
					}
          break;					
				}
				
				case 0x21:
				{
					if( mcpsIndication->BufferSize == 1 )
					{	
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_DEVICE_TIME;
						LoRaMacMlmeRequest( &mlmeReq );		
					}						
					break;	
				}
				
        case 0x7e: // Check compliance test disable command (ii)
        {
					if( mcpsIndication->BufferSize == 1 )
					{				
						if(ce_rtc_flag==1)
						{
							TimerStop( &CertifTxNextPacketTimer );	
							ce_rtc_flag=0;
						}		
						
						if(join_mothod==1)
						{	
							HAL_Delay(500);
					
							joined_finish=0;
					
							MlmeReq_t mlmeReq;

							// Disable TestMode and revert back to normal operation

							certifParam.DownLinkCounter = 0;
							certifParam.Running = false;

							MibRequestConfirm_t mibReq;
							mibReq.Type = MIB_ADR;
							mibReq.Param.AdrEnable = AdrEnableInit;
							LoRaMacMibSetRequestConfirm( &mibReq );

							mlmeReq.Type = MLME_JOIN;
							mlmeReq.Req.Join = *JoinParameters;
					
							LoRaMacDevNonce++;
					    save_devNonce_buff();
							HAL_Delay(50);
							LoRaMacMlmeRequest( &mlmeReq );
						}
						disport=1;
				 }						
				 break;
        }				
        case 0x7f:
				{
					if( mcpsIndication->BufferSize == 1 )
					{
						uint8_t software[12]={0x04,0x05,0x01,0x00,0x01,0x00,0x04,0x00,0x00,0x00,0x06,0x00};
						for( uint8_t i = 1; i < 13; i++ )
						{
              certifParam.DataBuffer[i] = software[i-1];
						}							
					}
				 break;	
				}
        default:   	
          break;
     }
	 }
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnCertifTxNextPacketTimerEvent( void *context )
{
	TimerSetValue( &CertifTxNextPacketTimer,  TEST_TX_DUTYCYCLE); 
  TimerStart( &CertifTxNextPacketTimer);
	if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
  {	
		certif_tx();
	}		
}
