/**
  *************************************************************************************************************************
  * @file    : key.c
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : keyģ��C�ļ�
  *************************************************************************************************************************
  * @attention
  *
  * 
  *
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "key.h" 
#include "delay.h"


void KEY_GPIO_Config(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;
		/*����LED��ص�GPIO����ʱ��*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_15;	
		/*��������ģʽΪ��������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
}


//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//ע��˺�������Ӧ���ȼ�,KEY1>KEY2>KEY3!!
uint8_t KEY_Scan(uint8_t mode)
{	 
	static uint8_t key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY1==0||KEY2==0||KEY3==0))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY1==0)     return  KEY1_PRES;
		else if(KEY2==0)return  KEY2_PRES;
		else if(KEY3==0)return  KEY3_PRES;
	}else if (KEY1==1&&KEY2==1&&KEY3==1)key_up=1; 	    
 	return 0;// �ް�������
}

/*****************************************************END OF FILE*********************************************************/	
