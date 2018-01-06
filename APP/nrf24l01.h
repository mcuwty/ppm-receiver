#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f10x.h"

#define SUCCESS 		1 		//MCU与nrf24l01连接成功标志
#define FAIL    		2 		//MCU与nrf24l01连接失败标志

#define NRF_CE_L 		GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define NRF_CE_H 		GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define NRF_CSN_L 		GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define NRF_CSN_H 		GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define READ_IRQ		GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)

//******************************NRF24L01*************************************************
#define TX_ADR_WIDTH    5   	// 发送数据包地址长度
#define RX_ADR_WIDTH    5   	// 接收数据包地址长度
#define TX_PLOAD_WIDTH  32 		// 发送数据包数据长度
#define RX_PLOAD_WIDTH  32 		// 接收数据包数据长度
#define RX_DR 			6		// 接收中断标志位

#define MAX_TX  		0x10  	//达到最大发送次数中断
//******************************NRF24L01寄存器指令***************************************
#define NRF_READ_REG    0x00  	// 读寄存器指令
#define NRF_WRITE_REG   0x20 	// 写寄存器指令
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留
//******************************nRF24L01寄存器地址***************************************
#define CONFIG          0x00  	// 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  	// 自动应答功能设置
#define EN_RXADDR       0x02  	// 可用信道设置
#define SETUP_AW        0x03  	// 收发地址宽度设置
#define SETUP_RETR      0x04  	// 自动重发功能设置
#define RF_CH           0x05  	// 工作频率设置
#define RF_SETUP        0x06  	// 发射速率、功耗功能设置
#define STATUS          0x07  	// 状态寄存器
#define OBSERVE_TX      0x08  	// 发送监测功能
#define CD              0x09  	// 地址检测           
#define RX_ADDR_P0      0x0A  	// 频道0接收数据地址
#define RX_ADDR_P1      0x0B  	// 频道1接收数据地址
#define RX_ADDR_P2      0x0C  	// 频道2接收数据地址
#define RX_ADDR_P3      0x0D  	// 频道3接收数据地址
#define RX_ADDR_P4      0x0E  	// 频道4接收数据地址
#define RX_ADDR_P5      0x0F  	// 频道5接收数据地址
#define TX_ADDR         0x10  	// 发送地址寄存器
#define RX_PW_P0        0x11  	// 接收频道0接收数据长度
#define RX_PW_P1        0x12  	// 接收频道0接收数据长度
#define RX_PW_P2        0x13  	// 接收频道0接收数据长度
#define RX_PW_P3        0x14  	// 接收频道0接收数据长度
#define RX_PW_P4        0x15  	// 接收频道0接收数据长度
#define RX_PW_P5        0x16  	// 接收频道0接收数据长度
#define FIFO_STATUS     0x17  	// FIFO栈入栈出状态寄存器设置

extern u8 TX_ADDRESS[];
extern u8 RX_ADDRESS_0[];
extern u8 RX_ADDRESS_1[];
extern u8 RX_ADDRESS_2[];
extern u8 RX_ADDRESS_3[];

extern u8 NRF_Rx_BUF[];
extern u8 NRF_Tx_BUF[];

void NRF_SPI_Init(void);
u8 NRF_SPI_RW_Reg(u8 reg,u8 _data);
u8 NRF_SPI_WriteBuf(u8 reg,u8 *pBuf,u8 bytes);
u8 NRF_SPI_ReadBuf(u8 reg,u8 *pBuf,u8 bytes);
u8 NRF_Check(void);
void NRF_RX_Mode(void);
void NRF_TX_Mode(void);
void NRF_TxPacket(u8 *tx_address , u8 *tx_buf, u8 len);
void NRF_Tx_Fast(u8 *tx_buf, u8 len);
u8 NRF_RxPacket(u8 *rx_buf);
u8 NRF_Rx_loop(u8 *rx_buf);
void NRF_Rx_Test(u8 *rx_buf);

#endif
