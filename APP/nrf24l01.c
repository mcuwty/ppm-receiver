/****************************************************************************************
 * 文件名  ：nrf24l01.c
 * 描述    ：nrf24l01功能函数库         
 * 实验平台：STM32F103
 * 硬件连接：CSN -- PA4
 *           CE -- PA1
 * 			 IRQ -- PA0
 *           SCK -- PA5
 *           MISO -- PA6
 *           MOSI -- PA7         			
 * 库版本  ：ST3.5.0
 * 作者    ：吴泰月
 ***************************************************************************************/
#include "spi.h"
#include "nrf24l01.h"
#include "delay.h"
#include "usart.h"
#include "application.h"
#include "ppm.h"

extern uint8_t Link_Status;

u8 TX_ADDRESS[TX_ADR_WIDTH]   = {0x12,0x23,0x34,0x45,0x56};	//发送地址，即接收端的地址
u8 RX_ADDRESS_0[RX_ADR_WIDTH] = {0x12,0x23,0x34,0x45,0x56};	//本地通道0接收地址，先写低字节
u8 RX_ADDRESS_1[RX_ADR_WIDTH] = {0x11,0x22,0x33,0x44,0x20};	//本地通道1接收地址
u8 RX_ADDRESS_2[            ] = {0x40                    };	//本地通道2接收地址
u8 RX_ADDRESS_3[            ] = {0x60                    };	//本地通道3接收地址
//************************************收发缓冲区*****************************************
u8 NRF_Rx_BUF[RX_PLOAD_WIDTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
u8 NRF_Tx_BUF[TX_PLOAD_WIDTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/****************************************************************************************
 * 函数名  ：NRF_SPI_Init
 * 描述    ：nrf24l01初始化
 * 输入    ：无
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
void NRF_SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	SPI1_Init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;			//CE引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;			//CSN引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			//IRQ引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 		//下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 					//初始化中断线参数

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; 	//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; 			//子优先级 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 	//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);						//初始化 NVIC
}
/****************************************************************************************
 * 函数名  ：NRF_SPI_RW_Reg
 * 描述    ：nrf24l01读写寄存器函数
 * 输入    ：reg - 读写命令+寄存器地址
 *           _data - 写入的数据
 * 输出    ：无
 * 返回    ：nrf24l01状态标志
 * 调用    ：内部调用
 ***************************************************************************************/
u8 NRF_SPI_RW_Reg(u8 reg,u8 _data)
{
	u8 status;
	NRF_CE_L;			//先进入待机模式
	NRF_CSN_L;			//使能SPI传输
	status=SPI1_ReadWriteByte(reg);
	SPI1_ReadWriteByte(_data);
	NRF_CSN_H;			//结束SPI传输
	return status;
}
/****************************************************************************************
 * 函数名  ：NRF_SPI_WriteBuf
 * 描述    ：nrf24l01连续写入数据函数
 * 输入    ：reg - 寄存器地址
 *           *pBuf - 储存数组
 *           bytes - 写入字节数
 * 输出    ：无
 * 返回    ：nrf24l01状态标志
 * 调用    ：内部调用
 ***************************************************************************************/
u8 NRF_SPI_WriteBuf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status,byte_count;
	NRF_CE_L;			//先进入待机模式
	NRF_CSN_L;			//使能SPI传输
	status=SPI1_ReadWriteByte(reg);
	for(byte_count=0;byte_count<bytes;byte_count++)
		SPI1_ReadWriteByte(*pBuf++);
	NRF_CSN_H;			//结束SPI传输
	return status;
}
/****************************************************************************************
 * 函数名  ：NRF_SPI_ReadBuf
 * 描述    ：nrf24l01连续读取数据函数
 * 输入    ：reg - 寄存器地址
 *           *pBuf - 储存数组
 *           bytes - 读取字节数
 * 输出    ：无
 * 返回    ：nrf24l01状态标志
 * 调用    ：内部调用
 ***************************************************************************************/
u8 NRF_SPI_ReadBuf(u8 reg,u8 *pBuf,u8 bytes)
{
	u8 status,byte_count;
	NRF_CE_L;			//先进入待机模式
	NRF_CSN_L;			//使能SPI传输
	status=SPI1_ReadWriteByte(reg);
	for(byte_count=0;byte_count<bytes;byte_count++)
		*pBuf++=SPI1_ReadWriteByte(0xff);
	NRF_CSN_H;			//结束SPI传输
	return status;
}
/****************************************************************************************
 * 函数名  ：NRF_Check
 * 描述    ：nrf24l01spi通讯检测函数
 * 输入    ：无
 * 输出    ：无
 * 返回    ：通讯标志----success，成功标志；error，失败标志
 * 调用    ：外部调用
 ***************************************************************************************/
u8 NRF_Check(void)
{
	u8 i;
	u8 buf[5];
	NRF_SPI_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5);	//写入5个字节的地址
	NRF_SPI_ReadBuf(NRF_READ_REG+TX_ADDR,buf,5);			//读出写入的地址
	
	for(i=0;i<5;i++)
	{
		if(buf[i]!=TX_ADDRESS[i])
			break;
	}
	if(i==5)
		return SUCCESS;	//MCU与nrf2401连接成功
	else
		return FAIL;	//MCU与nrf2401连接失败
}
/****************************************************************************************
 * 函数名  ：NRF_RX_Mode
 * 描述    ：nrf24l01接收模式配置
 * 输入    ：无
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
void NRF_RX_Mode(void)
{
	NRF_CE_L;

	NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS_0,RX_ADR_WIDTH);	//写入RX端P0通道地址
	// NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P1,RX_ADDRESS_1,RX_ADR_WIDTH);	//写入RX端P1通道地址
	// NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P2,RX_ADDRESS_2,1);	//写入RX端P2通道地址
	// NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P3,RX_ADDRESS_3,1);	//写入RX端P3通道地址

	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_AA,0x01);				//使能通道0的自动应答
	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);			//使能通道0的接收地址
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_RXADDR,0x0f);			//使能通道0、1、2、3的接收地址
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_CH,0x55);				//设置RF通信频率
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);	//设置通道0的有效数据宽度
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P1,RX_PLOAD_WIDTH);	//设置通道1的有效数据宽度
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P2,RX_PLOAD_WIDTH);	//设置通道2的有效数据宽度
	// NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P3,RX_PLOAD_WIDTH);	//设置通道3的有效数据宽度
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);	//设置TX发射参数，0db增益，2Mbps，低噪声增益开启
	NRF_SPI_RW_Reg(NRF_WRITE_REG+CONFIG,0x0f);		//配置基本工作模式参数，使能16CRC，上电，接收模式，开启所有中断

	NRF_CE_H;		//进入接收模式
	delay_us(150);
}
/****************************************************************************************
 * 函数名  ：NRF_TX_Mode
 * 描述    ：nrf24l01发射模式配置
 * 输入    ：无
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
void NRF_TX_Mode(void)
{
	NRF_CE_L;

	NRF_SPI_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);		//写入接收端地址
	NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,TX_ADDRESS,RX_ADR_WIDTH);		//写入本地通道0接收地址，用于使能ACK

	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_AA,0x01);				//使能通道0的自动应答
	NRF_SPI_RW_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);			//使能通道0的接收地址
	NRF_SPI_RW_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);			//设置自动重发间隔时间：500us+86us；最大重发次数：10
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_CH,0x55);				//设置RF通信频率
	//NRF_SPI_RW_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//设置通道0的有效数据宽度
	NRF_SPI_RW_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);	//设置TX发射参数，0db增益，2Mbps，低噪声增益开启
	NRF_SPI_RW_Reg(NRF_WRITE_REG+CONFIG,0x0e);		//配置基本工作模式参数，使能16CRC，上电，接收模式，开启所有中断

	NRF_CE_H;		//进入发射模式
	delay_us(150);	//CE需要拉高一段时间才能进入发射模式
}
/****************************************************************************************
 * 函数名  ：NRF_TxPacket
 * 描述    ：nrf24l01发射数据函数
 * 输入    ：*tx_address - 接收端地址
 *           *tx_buf - 需要发送的数据地址指针
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
void NRF_TxPacket(u8 *tx_address , u8 *tx_buf, u8 len)
{
	u8 state;
	NRF_CE_L;	//StandBy I模式	 
	NRF_SPI_WriteBuf(NRF_WRITE_REG + TX_ADDR, tx_address, TX_ADR_WIDTH);	//装载接收端地址
	NRF_SPI_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,tx_address,RX_ADR_WIDTH);		//写入本地通道0接收地址，用于使能ACK
	if( len >= TX_PLOAD_WIDTH )
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 		 	    // 装载数据
	else
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, len); 		 	    		// 装载数据	
	NRF_CE_H;	//置高CE，激发数据发送
	// delay_us(15);
	while( READ_IRQ != 0 );
	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);							//读取状态寄存
	NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state); 							//通过写1来清除中断标志
	if( state & MAX_TX )
		NRF_SPI_RW_Reg(FLUSH_TX,NOP);
}

/****************************************************************************************
 * 函数名  NRF_Tx_Fast
 * 描述    ：nrf24l01快速发射数据函数
 * 输入    ：*tx_buf - 需要发送的数据地址指针
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
void NRF_Tx_Fast(u8 *tx_buf, u8 len)
{
	u8 state;
	NRF_CE_L;	//StandBy I模式
	if( len >= TX_PLOAD_WIDTH )
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 		 	    // 装载数据
	else
		NRF_SPI_WriteBuf(WR_TX_PLOAD, tx_buf, len); 		 	    		// 装载数据	
	NRF_CE_H;	//置高CE，激发数据发送
	while( READ_IRQ != 0 );
	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);							//读取状态寄存
	NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state); 							//通过写1来清除中断标志
	if( state & MAX_TX )
		NRF_SPI_RW_Reg(FLUSH_TX,NOP);
}
/****************************************************************************************
 * 函数名  ：NRF_RxPacket
 * 描述    ：nrf24l01接收数据函数
 * 输入    ：*rx_buf - 接收的数组地址指针
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
u8 NRF_RxPacket(u8 *rx_buf)
{	 
	u8 state ;
	NRF_CE_H;											//进入接收模式
	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);		//读取状态寄存其来判断数据接收状况
	if(state & 0x40)									//判断是否接收到数据
	{
	    NRF_CE_L; 										//SPI使能
		NRF_SPI_ReadBuf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);;//读取接收数据到rx_buf
		NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state); 	//接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清除中断标志
		NRF_CE_H;										//进入接收模式
		return ((state & 0x0e)>>1); 					//返回接收通道
	}
	else
		return state; 									//返回状态标志
}
/****************************************************************************************
 * 函数名  ：NRF_Rx_loop
 * 描述    ：nrf24l01轮询接收数据
 * 输入    ：*rx_buf - 接收的数组地址指针
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
u8 NRF_Rx_loop(u8 *rx_buf)
{	 
	u8 state;

	NRF_CE_H;											//进入接收模式

	if( READ_IRQ == 0 )
	{
		NRF_CE_L;										//进入待机模式
		state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);	//读取状态寄存器
		NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state);		//写1清除中断标志

		if(state&(1<<RX_DR))							//判断是否收到数据
			NRF_SPI_ReadBuf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);	//读取数据
		NRF_SPI_RW_Reg(FLUSH_RX,NOP);	 				//清除RX FIFO寄存器
		NRF_CE_H;										//进入接收模式

		return 1;
	}
	
	NRF_CE_H;											//进入接收模式

	return 0;
}
//GPIOA0外部中断服务函数，NRF中断方式接收
void EXTI0_IRQHandler(void)
{
	u8 state;
	//Usart_Puts(COM1,"Fucking !\r\n");

	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)				//判断某个线上的中断是否发生
	{
		EXTI_ClearITPendingBit(EXTI_Line0); 			//清除 LINE 上的中断标志位

		NRF_CE_L;										//进入待机模式
		state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);	//读取状态寄存器
		NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state);		//写1清除中断标志

		if(state&(1<<RX_DR))							//判断是否收到数据
		{
			NRF_SPI_ReadBuf(RD_RX_PLOAD, NRF_Rx_BUF, RX_PLOAD_WIDTH);	//读取数据
			// osal_memcpy(ppm_channel,NRF_Rx_BUF,16);

			Link_Status = 1;							//更新标识nrf通信连接在线
			osal_start_timerEx(Serial_TaskID,NRF_UNLINK,100);		//100毫秒后未收到新的数据则标识断线
		}
		NRF_SPI_RW_Reg(FLUSH_RX,NOP);	 				//清除RX FIFO寄存器
		NRF_CE_H;										//进入接收模式
	} 
}
/****************************************************************************************
 * 函数名  ：NRF_Rx_Test
 * 描述    ：nrf24l01接收数据测试函数
 * 输入    ：*rx_buf - 接收的数组地址指针
 * 输出    ：无
 * 返回    ：无
 * 调用    ：外部调用
 ***************************************************************************************/
void NRF_Rx_Test(u8 *rx_buf)
{	 
	u8 state;

	NRF_CE_H;											//进入接收模式
	while(READ_IRQ!=0);	//等待数据中断
	NRF_CE_L;											//进入待机模式

	state=NRF_SPI_RW_Reg(NRF_READ_REG+STATUS,0xff);		//读取状态寄存器
	NRF_SPI_RW_Reg(NRF_WRITE_REG+STATUS,state);			//写1清除中断标志

	if(state&(1<<RX_DR))								//判断是否收到数据
	{
		NRF_SPI_ReadBuf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH);	//读取数据
	}
	//NRF_SPI_RW_Reg(FLUSH_RX,NOP);	 //清除RX FIFO寄存器
	NRF_CE_H;											//进入接收模式
} 
