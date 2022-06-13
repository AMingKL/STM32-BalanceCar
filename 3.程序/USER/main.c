/**
  *************************************************************************************************************************
  * @file    main.c
  * @author  AMKL
  * @version V1.0
  * @date    2022-05-01
  * @brief   ƽ��С��
  *************************************************************************************************************************
  * @attention
	
	Qq:2583518067
	Bվ��amingkl
	CSDN��amkl
	Github:https://github.com/AMingKL
	
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sys.h" 

uint8_t delay_50ms,delay_flag;

int main(void)	
{
	delay_init();//�δ�ʱ����ʼ��
	NVIC_Config();//�ⲿ�ж�����
	uart3_init(115200);//������Ӧ����3��ʼ��-115200-8-N-1	
	OLED_Init();//oled��ʼ��
	OLED_Clear();//oled����
	IIC_Init();//mpu6050iic��ʼ��	 	
	MPU6050_initialize();//mpu6050��ʼ��	        
	DMP_Init();//dmp��ʼ��             
	MPU6050_EXTI_Init();//mpu6050INT�����жϳ�ʼ��	
	Encoder_TIM2_Init();//�������������
	Encoder_TIM4_Init();//�������������
	Motor_Init();//�����ʼ��
	Adc_Init();	//adc��ʼ��
	PWM_Init_TIM1(0,7199);//10khzƵ���������
	KEY_GPIO_Config();//������ʼ��
	PID_ParaInit();//pid������ʼ��
  while(1)	
	{		
		 Oled_Proc();

		 delay_flag=1;	
		 delay_50ms=0;
		 while(delay_flag);	//ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ	
	} 	
}
