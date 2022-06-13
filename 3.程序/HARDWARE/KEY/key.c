/**
  *************************************************************************************************************************
  * @file    : key.c
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : key模块C文件
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
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;
		/*开启LED相关的GPIO外设时钟*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_15;	
		/*设置引脚模式为上拉输入*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
		/*调用库函数，初始化GPIO*/
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
}


//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//注意此函数有响应优先级,KEY1>KEY2>KEY3!!
uint8_t KEY_Scan(uint8_t mode)
{	 
	static uint8_t key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY1==0||KEY2==0||KEY3==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY1==0)     return  KEY1_PRES;
		else if(KEY2==0)return  KEY2_PRES;
		else if(KEY3==0)return  KEY3_PRES;
	}else if (KEY1==1&&KEY2==1&&KEY3==1)key_up=1; 	    
 	return 0;// 无按键按下
}

/*****************************************************END OF FILE*********************************************************/	
