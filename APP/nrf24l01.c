/****************************************************************************************
 * �ļ���  ��nrf24l01.c
 * ����    ��nrf24l01���ܺ�����         
 * ʵ��ƽ̨��STM32F103
 * Ӳ�����ӣ�CSN -- PA4
 *           CE -- PA1
 * 			 IRQ -- PA0
 *           SCK -- PA5
 *           MISO -- PA6
 *           MOSI -- PA7         			
 * ��汾  ��ST3.5.0
 * ����    ����̩��
 ***************************************************************************************/
#include "spi.h"
#include "nrf24l01.h"
#include "delay.h"
#include "usart.h"
#include "application.h"
#include "ppm.h"

extern uint8_t Link_Status;

u8 TX_ADDRESS[TX_ADR_WIDTH]   = {0x12,0x23,0x34,0x45,0x56};	//���͵�ַ�������ն˵ĵ�ַ
u8 RX_ADDRESS_0[RX_ADR_WIDTH] = {0x12,0x23,0x34,0x45,0x56};	//����ͨ��0���յ�ַ����д���ֽ�
u8 RX_ADDRESS_1[RX_ADR_WIDTH] = {0x11,0x22,0x33,0x44,0x20};	//����ͨ��1���յ�ַ
u8 RX_ADDRESS_2[            ] = {0x40                    };	//����ͨ��2���յ�ַ
u8 RX_ADDRESS_3[            ] = {0x60                    };	//����ͨ��3���յ�ַ
//************************************�շ�������*****************************************
u8 NRF_Rx_BUF[RX_PLOAD_WIDTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
u8 NRF_Tx_BUF[TX_PLOAD_WIDTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/****************************************************************************************
 * ������  ��NRF_SPI_Init
 * ����    ��nrf24l01��ʼ��
 * ����    ����
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
void NRF_SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	SPI1_Init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;			//CE����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//�������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;			//CSN����
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			//IRQ����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 		//�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 					//��ʼ���ж��߲���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 	//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; 			//�����ȼ� 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 	//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);						//��ʼ�� NVIC
}
/****************************************************************************************
 * ������  ��NRF_SPI_RW_Reg
 * ����    ��nrf24l01��д�Ĵ�������
 * ����    ��reg - ��д����+�Ĵ�����ַ
 *           _data - д�������
 * ���    ����
 * ����    ��nrf24l01״̬��־
 * ����    ���ڲ�����
 ***************************************************************************************/
u8 NRF_SPI_RW_Reg(u8 reg,u8 _data)
{
	u8 status;
	NRF_CE_L;			//�Ƚ������ģʽ
	NRF_CSN_L;			//ʹ��SPI����
	status=SPI1_ReadWriteByte(reg);
	SPI1_ReadWriteByte(_data);
	NRF_CSN_H;			//����SPI����
	return status;
}
/****************************************************************************************
 * ������  ��NRF_SPI_WriteBuf
 * ����    ��nrf24l01����д�����ݺ���
 * ����    ��reg - �Ĵ�����ַ
 *           *pBuf - ��������
 *           bytes - д���ֽ���
 * ���    ����
 * ����    ��nrf24l01״̬��־
 * ����    ���ڲ�����
 ***************************************************************************************/
u8 NRF_SPI_WriteBuf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status,byte_count;
	NRF_CE_L;			//�Ƚ������ģʽ
	NRF_CSN_L;			//ʹ��SPI����
	status=SPI1_ReadWriteByte(reg);
	for(byte_count=0;byte_count<bytes;byte_count++)
		SPI1_ReadWriteByte(*pBuf++);
	NRF_CSN_H;			//����SPI����
	return status;
}
/****************************************************************************************
 * ������  ��NRF_SPI_ReadBuf
 * ����    ��nrf24l01������ȡ���ݺ���
 * ����    ��reg - �Ĵ�����ַ
 *           *pBuf - ��������
 *           bytes - ��ȡ�ֽ���
 * ���    ����
 * ����    ��nrf24l01״̬��־
 * ����    ���ڲ�����
 ***************************************************************************************/
u8 NRF_SPI_ReadBuf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status,byte_count;
	NRF_CE_L;			//�Ƚ������ģʽ
	NRF_CSN_L;			//ʹ��SPI����
	status=SPI1_ReadWriteByte(reg);
	for(byte_count=0;byte_count<bytes;byte_count++)
		*pBuf++=SPI1_ReadWriteByte(0xff);
	NRF_CSN_H;			//����SPI����
	return status;
}
/****************************************************************************************
 * ������  ��NRF_Check
 * ����    ��nrf24l01spiͨѶ��⺯��
 * ����    ����
 * ���    ����
 * ����    ��ͨѶ��־----success���ɹ���־��error��ʧ�ܱ�־
 * ����    ���ⲿ����
 ***************************************************************************************/
u8 NRF_Check(void)
{
	u8 i;
	u8 buf[5];
	NRF_SPI_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5);	//д��5���ֽڵĵ�ַ
	NRF_SPI_ReadBuf(NRF_READ_REG+TX_ADDR,buf,5);			//����д��ĵ�ַ
	
	for(i=0;i<5;i++)
	{
		if(buf[i]!=TX_ADDRESS[i])
			break;
	}
	if(i==5)
		return SUCCESS;	//MCU��nrf2401���ӳɹ�
	else
		return FAIL;	//MCU��nrf2401����ʧ��
}
/****************************************************************************************
 * ������  ��NRF_RX_Mode
 * ����    ��nrf24l01����ģʽ����
 * ����    ����
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
void NRF_RX_Mode(void)
{
	NRF_CE_L;

	NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS_0,RX_ADR_WIDTH);	//д��RX��P0ͨ����ַ
	// NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P1,RX_ADDRESS_1,RX_ADR_WIDTH);	//д��RX��P1ͨ����ַ
	// NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P2,RX_ADDRESS_2,1);	//д��RX��P2ͨ����ַ
	// NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P3,RX_ADDRESS_3,1);	//д��RX��P3ͨ����ַ

	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_AA,0x01);				//ʹ��ͨ��0���Զ�Ӧ��
	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);			//ʹ��ͨ��0�Ľ��յ�ַ
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_RXADDR,0x0f);			//ʹ��ͨ��0��1��2��3�Ľ��յ�ַ
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_CH,0x55);				//����RFͨ��Ƶ��
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	//����ͨ��0����Ч���ݿ��
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P1,RX_PLOAD_WIDTH);	//����ͨ��1����Ч���ݿ��
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P2,RX_PLOAD_WIDTH);	//����ͨ��2����Ч���ݿ��
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P3,RX_PLOAD_WIDTH);	//����ͨ��3����Ч���ݿ��
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);	//����TX���������0db���棬2Mbps�����������濪��
	NRF_SPI_RW_Reg(NRF_WRITE_REG+CONFIG,0x0f);		//���û�������ģʽ������ʹ��16CRC���ϵ磬����ģʽ�����������ж�

	NRF_CE_H;		//�������ģʽ
	delay_us(150);
}
/****************************************************************************************
 * ������  ��NRF_TX_Mode
 * ����    ��nrf24l01����ģʽ����
 * ����    ����
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
void NRF_TX_Mode(void)
{
	NRF_CE_L;

	NRF_SPI_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);		//д����ն˵�ַ
	NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,TX_ADDRESS,RX_ADR_WIDTH);		//д�뱾��ͨ��0���յ�ַ������ʹ��ACK

	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_AA,0x01);				//ʹ��ͨ��0���Զ�Ӧ��
	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);			//ʹ��ͨ��0�Ľ��յ�ַ
	NRF_SPI_RW_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);			//�����Զ��ط����ʱ�䣺500us+86us������ط�������10
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_CH,0x55);				//����RFͨ��Ƶ��
	//NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//����ͨ��0����Ч���ݿ��
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);	//����TX���������0db���棬2Mbps�����������濪��
	NRF_SPI_RW_Reg(NRF_WRITE_REG+CONFIG,0x0e);		//���û�������ģʽ������ʹ��16CRC���ϵ磬����ģʽ�����������ж�

	NRF_CE_H;		//���뷢��ģʽ
	delay_us(150);	//CE��Ҫ����һ��ʱ����ܽ��뷢��ģʽ
}
/****************************************************************************************
 * ������  ��NRF_TxPacket
 * ����    ��nrf24l01�������ݺ���
 * ����    ��*tx_address - ���ն˵�ַ
 *           *tx_buf - ��Ҫ���͵����ݵ�ַָ��
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
void NRF_TxPacket(u8 *tx_address , u8 *tx_buf, u8 len)
{
	u8 state;
	NRF_CE_L;	//StandBy Iģʽ	 
	NRF_SPI_WriteBuf(NRF_WRITE_REG + TX_ADDR, tx_address, TX_ADR_WIDTH);	//װ�ؽ��ն˵�ַ
	NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,tx_address,RX_ADR_WIDTH);		//д�뱾��ͨ��0���յ�ַ������ʹ��ACK
	if( len >= TX_PLOAD_WIDTH )
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 		 	    // װ������
	else
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, len); 		 	    		// װ������	
	NRF_CE_H;	//�ø�CE���������ݷ���
	// delay_us(15);
	while( READ_IRQ != 0 );
	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);							//��ȡ״̬�Ĵ�
	NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state); 							//ͨ��д1������жϱ�־
	if( state & MAX_TX )
		NRF_SPI_RW_Reg(FLUSH_TX,NOP);
}

/****************************************************************************************
 * ������  NRF_Tx_Fast
 * ����    ��nrf24l01���ٷ������ݺ���
 * ����    ��*tx_buf - ��Ҫ���͵����ݵ�ַָ��
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
void NRF_Tx_Fast(u8 *tx_buf, u8 len)
{
	u8 state;
	NRF_CE_L;	//StandBy Iģʽ
	if( len >= TX_PLOAD_WIDTH )
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 		 	    // װ������
	else
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, len); 		 	    		// װ������	
	NRF_CE_H;	//�ø�CE���������ݷ���
	while( READ_IRQ != 0 );
	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);							//��ȡ״̬�Ĵ�
	NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state); 							//ͨ��д1������жϱ�־
	if( state & MAX_TX )
		NRF_SPI_RW_Reg(FLUSH_TX,NOP);
}
/****************************************************************************************
 * ������  ��NRF_RxPacket
 * ����    ��nrf24l01�������ݺ���
 * ����    ��*rx_buf - ���յ������ַָ��
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
u8 NRF_RxPacket(u8 *rx_buf)
{	 
	u8 state ;
	NRF_CE_H;											//�������ģʽ
	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);		//��ȡ״̬�Ĵ������ж����ݽ���״��
	if(state & 0x40)									//�ж��Ƿ���յ�����
	{
	    NRF_CE_L; 										//SPIʹ��
		NRF_SPI_ReadBuf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);;//��ȡ�������ݵ�rx_buf
		NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state); 	//���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�־
		NRF_CE_H;										//�������ģʽ
		return ((state & 0x0e)>>1); 					//���ؽ���ͨ��
	}
	else
		return state; 									//����״̬��־
}
/****************************************************************************************
 * ������  ��NRF_Rx_loop
 * ����    ��nrf24l01��ѯ��������
 * ����    ��*rx_buf - ���յ������ַָ��
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
u8 NRF_Rx_loop(u8 *rx_buf)
{	 
	u8 state;

	NRF_CE_H;											//�������ģʽ

	if( READ_IRQ == 0 )
	{
		NRF_CE_L;										//�������ģʽ
		state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);	//��ȡ״̬�Ĵ���
		NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state);		//д1����жϱ�־

		if(state&(1<<RX_DR))							//�ж��Ƿ��յ�����
			NRF_SPI_ReadBuf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);	//��ȡ����
		NRF_SPI_RW_Reg(FLUSH_RX,NOP);	 				//���RX FIFO�Ĵ���
		NRF_CE_H;										//�������ģʽ

		return 1;
	}
	
	NRF_CE_H;											//�������ģʽ

	return 0;
}
//GPIOA0�ⲿ�жϷ�������NRF�жϷ�ʽ����
void EXTI0_IRQHandler(void)
{
	u8 state;
	//Usart_Puts(COM1,"Fucking !\r\n");

	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)				//�ж�ĳ�����ϵ��ж��Ƿ���
	{
		EXTI_ClearITPendingBit(EXTI_Line0); 			//��� LINE �ϵ��жϱ�־λ

		NRF_CE_L;										//�������ģʽ
		state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);	//��ȡ״̬�Ĵ���
		NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state);		//д1����жϱ�־

		if(state&(1<<RX_DR))							//�ж��Ƿ��յ�����
		{
			NRF_SPI_ReadBuf(RD_RX_PLOAD, NRF_Rx_BUF, RX_PLOAD_WIDTH);	//��ȡ����
			// osal_memcpy(ppm_channel,NRF_Rx_BUF,16);

			Link_Status = 1;							//���±�ʶnrfͨ����������
			osal_start_timerEx(Serial_TaskID,NRF_UNLINK,100);		//100�����δ�յ��µ��������ʶ����
		}
		NRF_SPI_RW_Reg(FLUSH_RX,NOP);	 				//���RX FIFO�Ĵ���
		NRF_CE_H;										//�������ģʽ
	} 
}
/****************************************************************************************
 * ������  ��NRF_Rx_Test
 * ����    ��nrf24l01�������ݲ��Ժ���
 * ����    ��*rx_buf - ���յ������ַָ��
 * ���    ����
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/
void NRF_Rx_Test(u8 *rx_buf)
{	 
	u8 state;

	NRF_CE_H;											//�������ģʽ
	while(READ_IRQ!=0);	//�ȴ������ж�
	NRF_CE_L;											//�������ģʽ

	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);		//��ȡ״̬�Ĵ���
	NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state);			//д1����жϱ�־

	if(state&(1<<RX_DR))								//�ж��Ƿ��յ�����
	{
		NRF_SPI_ReadBuf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);	//��ȡ����
	}
	//NRF_SPI_RW_Reg(FLUSH_RX,NOP);	 //���RX FIFO�Ĵ���
	NRF_CE_H;											//�������ģʽ
} 
