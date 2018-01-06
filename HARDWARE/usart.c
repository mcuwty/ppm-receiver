/****************************************************************************************
 * �ļ���  ��usart.c
 * ����    ������ͨ������ 
 * ����ƽ̨��STM32F103		
 * ��汾  ��ST3.5.0
 * ����    ����̩��
 ***************************************************************************************/
#include "usart.h"

#define PreemptionPriority 		3 			//������ռ���ȼ�
#define SubPriority 			3 			//���������ȼ�

USART_TypeDef* UsartInstance[3] = {USART1, USART2, USART3};

/****************************************************************************************
 * ������  ��Usart_Init
 * ����    ��Usart��ʼ��
 * ����    ��Com - ���ڱ�ţ������ͷ�ļ�����ͬ
 *			 Baud - ������
 *			 StopBit - ֹͣλ
 *  		 Datalen - ���ݳ���
 *			 Parity - ��żУ��
 *			 HardControl - Ӳ��������
 * ���    ���������
 * ����    ����
 * ����    ���ⲿ����
 ***************************************************************************************/ 
void Usart_Init(uint8_t Com,uint32_t Baud,uint8_t StopBit,
                uint8_t Datalen,uint8_t Parity,uint8_t HardControl)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_InitStructure.USART_BaudRate = Baud;
	switch(StopBit)
	{
		case STOP_1_B:
            USART_InitStructure.USART_StopBits = USART_StopBits_1;
            break;
        case STOP_0_5_B:
        	USART_InitStructure.USART_StopBits = USART_StopBits_0_5;
        	break;
        case STOP_1_5_B:
            USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
            break;
        case STOP_2_B:
            USART_InitStructure.USART_StopBits = USART_StopBits_2;
            break;
        default :                   //Ĭ��1��ֹͣλ
            USART_InitStructure.USART_StopBits = USART_StopBits_1;
            break;
	}
	switch(Datalen)
    {
        case WROD_LEN_8B:
            USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            break;
        case WROD_LEN_9B:
            USART_InitStructure.USART_WordLength = USART_WordLength_9b;
            break;
        default :                   //Ĭ���ֳ�8�ֽ�
            USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            break;
    }
    switch(Parity)
    {
        case PARITY_EVEN:
            USART_InitStructure.USART_Parity = USART_Parity_Even;
            break;
        case PARITY_ODD:
            USART_InitStructure.USART_Parity = USART_Parity_Odd;
            break;
        case PARITY_NO:
            USART_InitStructure.USART_Parity = USART_Parity_No;
            break;
        default:                    //Ĭ������żУ��λ
            USART_InitStructure.USART_Parity = USART_Parity_No;
            break;
    }
    switch(HardControl)
    {
        case HARD_RTS:
            USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;
            break;
        case HARD_CTS:
            USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
            break;
        case HARD_RTS_CTS:
            USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
            break;
        case HARD_NO:
            USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
            break;
        default :                   //Ĭ����Ӳ��������
            USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
            break;
    }
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//�շ�ʹ��

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
    switch(Com)
    {
    	case COM1:
    		RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1,ENABLE);
    		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);

		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		    GPIO_Init(GPIOA, &GPIO_InitStructure);
		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		    GPIO_Init(GPIOA, &GPIO_InitStructure);  

			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			USART_Init(UsartInstance[Com], &USART_InitStructure);
		    USART_ITConfig(UsartInstance[Com], USART_IT_RXNE, ENABLE);
		    USART_Cmd(UsartInstance[Com], ENABLE);
    		break;
    	case COM2:
    		RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2,ENABLE);
    		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);

		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		    GPIO_Init(GPIOA, &GPIO_InitStructure);
		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		    GPIO_Init(GPIOA, &GPIO_InitStructure);  

			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			USART_Init(UsartInstance[Com], &USART_InitStructure);
		    USART_ITConfig(UsartInstance[Com], USART_IT_RXNE, ENABLE);
		    USART_Cmd(UsartInstance[Com], ENABLE);
    		break;
    	case COM3:
    		break;
    }
} 

/****************************************************************************************
 * ������  ��USART1_IRQHandler
 * ����    ������1�жϷ������
 * ����    ����
 * ���    ����
 * ����    ����
 * ����    ���ڲ�����
 ***************************************************************************************/
void USART1_IRQHandler(void)                
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)		//�����ж�
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		//Receive = USART_ReceiveData(USART1);//(USART1->DR);		//��ȡ���յ�������
        // USART_ReceiveData(USART1);//(USART1->DR);		//��ȡ���յ������� 
        // Usart_Puts(COM1,"fucking\r\n");

        // if( parse_key_board_value(USART1->DR, &read_kb_step, &read_key_board) ) 
        // {
        //     //�յ��Ϸ���ָ��֡
        //     osal_memcpy(&key_board_value,&read_key_board,sizeof(STM32_CTL_BOARD_KEY));
        //     osal_set_event(Serial_TaskID,RECE_KEY_VAL);
        // }
	}
}
/****************************************************************************************
 * ������  ��USART2_IRQHandler
 * ����    ������2�жϷ������
 * ����    ����
 * ���    ����
 * ����    ����
 * ����    ���ڲ�����
 ***************************************************************************************/
void USART2_IRQHandler(void)                
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)		//�����ж�
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		//Receive = USART_ReceiveData(USART2);//(USART2->DR);		//��ȡ���յ�������
        USART_ReceiveData(USART2);//(USART2->DR);		//��ȡ���յ�������  	        
	}
}
/****************************************************************************************
 * ��������Usart_transmit
 * ����  ��Usart���͵��ַ�����
 * ����  ��data - �ַ�����
 * ���  ���������
 * ����  ����
 * ����  ���ڲ�����
 ***************************************************************************************/
void Usart_Transmit(uint8_t Com, unsigned int data)
{
	while(USART_GetFlagStatus(UsartInstance[Com],USART_FLAG_TC)==RESET); 
	USART_SendData(UsartInstance[Com],data);
}
/****************************************************************************************
 * ��������Usart_puts
 * ����  ��Usart�����ַ�������
 * ����  ��*string - �ַ���
 * ���  ���������
 * ����  ����
 * ����  ���ڲ�����
 ***************************************************************************************/
void Usart_Puts(uint8_t Com,char *string)
{
	while(*string)
		Usart_Transmit(Com,*string++);
}
/****************************************************************************************
 * ��������Usart_printf
 * ����  ��Usart��ӡ��ʽ�ַ�������
 * ����  ��*format,...- ��ʽ�ַ���
 * ���  ���������
 * ����  ����
 * ����  ���ڡ��ⲿ����
 ***************************************************************************************/
void Usart_Printf(uint8_t Com,char *format, ...)
{
	__va_list arg_ptr;
	char str[100];

	va_start(arg_ptr, format);
	vsprintf(str, format, arg_ptr);
	va_end(arg_ptr);

	Usart_Puts(Com, str);
}
/************************************END OF FILE****************************************/
