/**
  *************************************************************************************************************************
  * @file    : exti.h
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : 外部中断H文件
  *************************************************************************************************************************
  * @attention
  * 
  * 
  * 
  *************************************************************************************************************************
  */


/* Define to prevent recursive inclusion --------------------------------------------------------------------------------*/
#ifndef  _EXTI_H
#define  _EXTI_H
/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "sys.h" 

/* define ---------------------------------------------------------------------------------------------------------------*/
#define INT PAin(12)   //PA12连接到MPU6050的中断引脚

/* function -------------------------------------------------------------------------------------------------------------*/
void MPU6050_EXTI_Init(void);
#endif /* _EXTI_H */

/*****************************************************END OF FILE*********************************************************/	


