/**
  *************************************************************************************************************************
  * @file    : motor.c
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : �������C�ļ�
  *************************************************************************************************************************
  * @attention
  *
  * 
  *
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "motor.h"

//�����ʼ��
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3 |GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);	
}


/*------------------------------------------
 ��������:PWM�޷�����
 ����˵��:
	->��ڲ���:�����PWMֵ
------------------------------------------*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}


/*------------------------------------------
 ��������:����ֵ����
 ����˵��:��ֵȡ����ֵ			
------------------------------------------*/
int abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}


/*------------------------------------------
 ��������:PWMװ�غ���
 ����˵��:
	->��ڲ���:PID������ɺ������PWMֵ
------------------------------------------*/
void Load(int moto1,int moto2)//moto1=-200����ת200������
{
	if(moto1>0)	Ain1=1,Ain2=0;//��ת
	else 				Ain1=0,Ain2=1;//��ת
	TIM_SetCompare1(TIM1,abs(moto1));
	
	if(moto2>0)	Bin1=0,Bin2=1;
	else 				Bin1=1,Bin2=0;	
	TIM_SetCompare4(TIM1,abs(moto2));
}

/*****************************************************END OF FILE*********************************************************/	
