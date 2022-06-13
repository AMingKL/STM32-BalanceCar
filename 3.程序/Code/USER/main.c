/**
  *************************************************************************************************************************
  * @file    main.c
  * @author  AMKL
  * @version V1.0
  * @date    2022-05-01
  * @brief   平衡小车
  *************************************************************************************************************************
  * @attention
	
	Qq:2583518067
	B站：amingkl
	CSDN：amkl
	Github:https://github.com/AMingKL
	
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sys.h" 

uint8_t delay_50ms,delay_flag;

int main(void)	
{
	delay_init();//滴答定时器初始化
	NVIC_Config();//外部中断配置
	uart3_init(115200);//蓝牙对应串口3初始化-115200-8-N-1	
	OLED_Init();//oled初始化
	OLED_Clear();//oled清屏
	IIC_Init();//mpu6050iic初始化	 	
	MPU6050_initialize();//mpu6050初始化	        
	DMP_Init();//dmp初始化             
	MPU6050_EXTI_Init();//mpu6050INT引脚中断初始化	
	Encoder_TIM2_Init();//电机编码器配置
	Encoder_TIM4_Init();//电机编码器配置
	Motor_Init();//电机初始化
	Adc_Init();	//adc初始化
	PWM_Init_TIM1(0,7199);//10khz频率驱动电机
	KEY_GPIO_Config();//按键初始化
	PID_ParaInit();//pid参数初始化
  while(1)	
	{		
		 Oled_Proc();

		 delay_flag=1;	
		 delay_50ms=0;
		 while(delay_flag);	//通过MPU6050的INT中断实现的50ms精准延时	
	} 	
}
