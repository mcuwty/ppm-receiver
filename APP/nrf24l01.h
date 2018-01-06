#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f10x.h"

#define SUCCESS 		1 		//MCU��nrf24l01���ӳɹ���־
#define FAIL    		2 		//MCU��nrf24l01����ʧ�ܱ�־

#define NRF_CE_L 		GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define NRF_CE_H 		GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define NRF_CSN_L 		GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define NRF_CSN_H 		GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define READ_IRQ		GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)

//******************************NRF24L01*************************************************
#define TX_ADR_WIDTH    5   	// �������ݰ���ַ����
#define RX_ADR_WIDTH    5   	// �������ݰ���ַ����
#define TX_PLOAD_WIDTH  32 		// �������ݰ����ݳ���
#define RX_PLOAD_WIDTH  32 		// �������ݰ����ݳ���
#define RX_DR 			6		// �����жϱ�־λ

#define MAX_TX  		0x10  	//�ﵽ����ʹ����ж�
//******************************NRF24L01�Ĵ���ָ��***************************************
#define NRF_READ_REG    0x00  	// ���Ĵ���ָ��
#define NRF_WRITE_REG   0x20 	// д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61  	// ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  	// д��������ָ��
#define FLUSH_TX        0xE1 	// ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����
//******************************nRF24L01�Ĵ�����ַ***************************************
#define CONFIG          0x00  	// �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  	// �Զ�Ӧ��������
#define EN_RXADDR       0x02  	// �����ŵ�����
#define SETUP_AW        0x03  	// �շ���ַ�������
#define SETUP_RETR      0x04  	// �Զ��ط���������
#define RF_CH           0x05  	// ����Ƶ������
#define RF_SETUP        0x06  	// �������ʡ����Ĺ�������
#define STATUS          0x07  	// ״̬�Ĵ���
#define OBSERVE_TX      0x08  	// ���ͼ�⹦��
#define CD              0x09  	// ��ַ���           
#define RX_ADDR_P0      0x0A  	// Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  	// Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  	// Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  	// Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  	// Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  	// Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  	// ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  	// ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  	// ����Ƶ��0�������ݳ���
#define RX_PW_P2        0x13  	// ����Ƶ��0�������ݳ���
#define RX_PW_P3        0x14  	// ����Ƶ��0�������ݳ���
#define RX_PW_P4        0x15  	// ����Ƶ��0�������ݳ���
#define RX_PW_P5        0x16  	// ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  	// FIFOջ��ջ��״̬�Ĵ�������

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
