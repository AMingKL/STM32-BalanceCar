/**
  *************************************************************************************************************************
  * @file    : control.h
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : ��������--���ƺ���H�ļ�
  *************************************************************************************************************************
  * @attention
  * 
  * 
  * 
  *************************************************************************************************************************
  */


/* Define to prevent recursive inclusion --------------------------------------------------------------------------------*/
#ifndef  _CONTROL_H
#define  _CONTROL_H

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "sys.h" 

/* define ---------------------------------------------------------------------------------------------------------------*/
typedef struct
{
	float Med_Angle;			//��е��ֵ--��ʹ��С������ƽ��ס�ĽǶȡ�
	float Vertical_Kp;	  //ֱ����KP
	float Vertical_Kd;	  //ֱ����KD
	float Velocity_Kp;	  //�ٶȻ�KP
	float Velocity_Ki;    //�ٶȻ�KI
	float Turn_Kd;        //ת��KD
	float Turn_Kp;        //ת��KP
	int 	Vertical_out;   //ֱ�����������
	int 	Velocity_out;   //�ٶȻ��������
	int 	Turn_out;       //ת���������
	float Target_Speed;	  //�����ٶȣ�������
	float Turn_Speed;		  //�����ٶȣ�ƫ����	
}BlanceCar_PID;
/* function -------------------------------------------------------------------------------------------------------------*/
int Vertical(float Med,float Angle,float gyro_Y);//��������
int Velocity(int Target,int encoder_left,int encoder_right);
int Turn(int gyro_Z,int RC); 

void PID_ParaInit(void);
void Get_Angle(u8 way);
void Oled_Proc(void);
void Key_Proc(void);
void EXTI9_5_IRQHandler(void);


#endif

/*****************************************************END OF FILE*********************************************************/	
