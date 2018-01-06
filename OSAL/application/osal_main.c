/****************************************************************************************
 * �ļ���  ��osal_main.c
 * ����    ��osal����ϵͳ��������������������ڴ��ļ������
 * ����ƽ̨��
 * ��汾  ��
 ***************************************************************************************/
#include "application.h"

/******************************************************************************
 * @fn      main
 *
 * @brief   
 *
 * @param   
 *
 * @return
 *****************************************************************************/
int main(void)
{    
	//ϵͳӲ��������ȳ�ʼ��
	
	//��ֹ�ж�
    HAL_DISABLE_INTERRUPTS();

    //osal����ϵͳ��ʼ��
    osal_init_system();

	//�������
	osal_add_Task(Serial_Task_Init,Serial_Task_EventProcess,1);	
	//��ӵ�����ͳһ���г�ʼ��
	osal_Task_init();

	osal_mem_kick();

	//�����ж�
	HAL_ENABLE_INTERRUPTS();	

	//���ó�ʼ�����¼����ϵ����Ҫ�Զ���ѯ�������¼����ڴ����
	//osal_start_reload_timer( Serial_TaskID, SBUS_OUT, 7 );
	osal_start_reload_timer( Serial_TaskID, READ_NRF, 20 );

	//����osalϵͳ�������ٷ���
	osal_start_system();
}
