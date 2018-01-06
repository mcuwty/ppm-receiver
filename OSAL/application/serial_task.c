/****************************************************************************************
 * 文件名  ：serial_task.c
 * 描述    ：系统串口通信任务
 * 开发平台：
 * 库版本  ：
 ***************************************************************************************/
#include "application.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "usart.h"
#include "led.h"
#include "delay.h"
#include "sbus.h"
#include "nrf24l01.h"
#include "ppm.h"

uint8 Serial_TaskID;					//系统串口通信任务ID
uint8 NRF_Status = 0;					//nrf模块外部连接状态，0为未连接，1为连接上
uint8 Link_Status = 0;					//nrf模块通信连接状态，0为断线状态，1为在线

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */
void sbus_out_cb(uint8_t * odata, int len);
void LogicChan2sbus(uint8_t* logic_ch);
/*********************************************************************
 * FUNCTIONS
 *********************************************************************/
//串口通信任务初始化
void Serial_Task_Init(uint8 task_id)
{
	Serial_TaskID = task_id;

	//串口配置初始化
	Usart_Init( COM1,921600,STOP_1_B,WROD_LEN_8B,PARITY_NO,HARD_NO );
	//Usart_Init( COM2,100000,STOP_2_B,WROD_LEN_9B,PARITY_EVEN,HARD_NO );

	osal_start_timerEx(Serial_TaskID,RE_CHECK_NRF,1000);		//1秒后才开始初始化NRF，等待供电稳定

	NRF_SPI_Init();
	PPM_Init();
	LED_Init();

	Usart_Puts(COM1,"Fucking running! \r\n");
}

//串口通信任务事件处理
uint16 Serial_Task_EventProcess(uint8 task_id,uint16 task_event)
{
	if ( task_event & SYS_EVENT_MSG )   	//判断系统消息事件
  	{
  		osal_sys_msg_t *MSGpkt;    			//定义一个指向接受系统消息结构体的指针
	    //从消息队列获取消息  
	    MSGpkt = (osal_sys_msg_t *)osal_msg_receive( task_id ); 
    
	    while ( MSGpkt )
	    {
	      	switch ( MSGpkt->hdr.event )  	//判断消息事件
	      	{
	          	case OSAL_PRINTF:
	          		break;

	        	default:
	          		break;
	      	}

	      	// Release the memory
	      	osal_msg_deallocate( (uint8 *)MSGpkt );

	      	// Next  获取下一个消息
	      	MSGpkt = (osal_sys_msg_t *)osal_msg_receive( task_id );
	    }

    	// return unprocessed events
    	return (task_event ^ SYS_EVENT_MSG);
  	}
  	
	if(task_event & SBUS_OUT)
	{
		// static uint16_t chanl[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

		// send_sbus_out(key_board_value.data1,9,sbus_out_cb);	
		// ((uint16_t*)NRF_Rx_BUF)[2] = 1802;
		if(Link_Status)
		{
			LogicChan2sbus(NRF_Rx_BUF);
			send_sbus_out(sbus_channel,8,sbus_out_cb);
		}
		// NRF_Rx_loop(NRF_Rx_BUF);

	  	return task_event ^ SBUS_OUT;
	}

	if(task_event & READ_NRF)
	{
		//static int dir = 0;

		//if(dir)
		//{
		//	LED_RX = 0;
		//	dir = 0;
		//}
		//else
		//{
		//	LED_RX = 1;
		//	dir = 1;
		//}
		
		//if(NRF_Rx_loop(NRF_Rx_BUF))
		//	Usart_Puts(COM1,"Receive nrf data !\r\n");
		// Usart_Puts(COM1,"Fucking !\r\n");
		// NRF_Rx_loop(NRF_Rx_BUF);

		if(Link_Status)
		{
			LogicChan2ppm(NRF_Rx_BUF);
		}
		Usart_Printf(COM1,"%d %d %d %d %d %d %d %d\r\n",ppm_channel[0],ppm_channel[1],ppm_channel[2],ppm_channel[3],
														ppm_channel[4],ppm_channel[5],ppm_channel[6],ppm_channel[7]);
	  	return task_event ^ READ_NRF;
	}

	if(task_event & NRF_UNLINK)
	{
		// send_sbus_out(key_board_value.data1,5,sbus_out_cb);
		// Usart_Puts(COM1,"fucking\r\n");
		Link_Status = 0;
		osal_memcpy(ppm_channel,ppm_channel_default,16);	//掉线后设置PPM通道值为默认值

	  	return task_event ^ NRF_UNLINK;
	}

	if(task_event & RE_CHECK_NRF)
	{
		if( NRF_Check() == SUCCESS )
		{
			NRF_Status = 1;
			Usart_Puts(COM1,"Check nrf24l01 success !\r\n");

			NRF_RX_Mode();
			Usart_Puts(COM1,"NRF set rx mode !\r\n");
		}
		else
		{
			osal_start_timerEx(Serial_TaskID,RE_CHECK_NRF,1000);		//初始化NRF失败则1秒后重试
		}
		
	  	return task_event ^ RE_CHECK_NRF;
	}

	return 0;
}

void sbus_out_cb(uint8_t * odata, int len)
{
	uint8_t i = 0;
	while(len--)
	{
		Usart_Transmit(COM2,odata[i++]);
	}	
}

//逻辑通道值转换到SBUS通道值，并进行范围判断处理，防止错误的数据
void LogicChan2sbus(uint8_t* logic_ch)
{
	uint8_t i;
	uint16_t chan;

	for(i=0;i<16;i++)
	{
		chan = ((uint16_t*)logic_ch)[i];

		if(chan<1000)
			chan = 1000;
		if(chan>2000)
			chan = 2000;

		chan = chan - 1000;

		sbus_channel[i] = (uint16_t)(chan*1.6) + 202;
	}
}
