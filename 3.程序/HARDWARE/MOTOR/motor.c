/**
  *************************************************************************************************************************
  * @file    : motor.c
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : 电机配置C文件
  *************************************************************************************************************************
  * @attention
  *
  * 
  *
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "motor.h"

//电机初始化
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
 函数功能:PWM限幅函数
 函数说明:
	->入口参数:运算后PWM值
------------------------------------------*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}


/*------------------------------------------
 函数功能:绝对值函数
 函数说明:数值取绝对值			
------------------------------------------*/
int abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}


/*------------------------------------------
 函数功能:PWM装载函数
 函数说明:
	->入口参数:PID运算完成后的最终PWM值
------------------------------------------*/
void Load(int moto1,int moto2)//moto1=-200：反转200个脉冲
{
	if(moto1>0)	Ain1=1,Ain2=0;//正转
	else 				Ain1=0,Ain2=1;//反转
	TIM_SetCompare1(TIM1,abs(moto1));
	
	if(moto2>0)	Bin1=0,Bin2=1;
	else 				Bin1=1,Bin2=0;	
	TIM_SetCompare4(TIM1,abs(moto2));
}

/*****************************************************END OF FILE*********************************************************/	
