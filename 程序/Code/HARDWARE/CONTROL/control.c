/**
  *************************************************************************************************************************
  * @file    : control.c
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : ��������--���ƺ���C�ļ�
  *************************************************************************************************************************
  * @attention
  *
  * 
  *
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "control.h"
/* define ---------------------------------------------------------------------------------------------------------------*/
#define SPEED_X 23 //����(ǰ��)����趨�ٶ�
#define SPEED_Z 100//ƫ��(����)����趨�ٶ� 

BlanceCar_PID PID;//pid�ṹ��

uint8_t Way_Angle=1;//��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
float Angle_Balance,Gyro_Balance,Gyro_Turn;//ƽ����� ƽ�������� ת��������
float Acceleration_Z;//Z����ٶȼ� 

int Encoder_Left,Encoder_Right;//���������ݣ��ٶȣ�
int Voltage;//��ȡ��ѹ����
int PWM_MAX=7200,PWM_MIN=-7200;//PWM�޷�����
int MOTO1,MOTO2;//���װ��PWM����

uint8_t ADC_TimeCnt;//ADC��ȡˢ��ʱ�����
uint8_t BalanceState;//ƽ�⳵״̬
int Temperature;//��ʾ�¶�




/*------------------------------------------
 ��������:MPU6050�жϷ�����
 ����˵��:�ɼ����ݣ�����С��			
------------------------------------------*/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5)!=0)//һ���ж�
	{
		int PWM_out;
		if(PBin(5)==0)//�����ж�
		{
			EXTI_ClearITPendingBit(EXTI_Line5);//����жϱ�־λ
			if(delay_flag==1)
		  {
			 if(++delay_50ms==10)	 delay_50ms=0,delay_flag=0;//���������ṩ50ms�ľ�׼��ʱ
		  }			
			//1.�ɼ�����������&MPU6050�Ƕ�&��ѹ��Ϣ��
			Encoder_Left= Read_Speed(2);
			Encoder_Right=Read_Speed(4);		
			Get_Angle(Way_Angle); 			
			ADC_TimeCnt==100?ADC_TimeCnt=0,Voltage=Get_battery_volt():ADC_TimeCnt++;//0.1s��ȡһ�ε�ѹ		
			/*********************************************************************************************/
			/*ǰ��*/
			if((Fore==0)&&(Back==0))
			{
			  if(PID.Target_Speed>0)
				{
				 (PID.Target_Speed<=1)?(PID.Target_Speed=0):(PID.Target_Speed--);//δ���ܵ�ǰ������ָ��-->�ٶ����㣬����ԭ��
				}
				else
				{
				 (PID.Target_Speed>=-1)?(PID.Target_Speed=0):(PID.Target_Speed++);
				}
			}	
			
			if(Fore==1)PID.Target_Speed--;//ǰ��1��־λ����-->��Ҫǰ��
			if(Back==1)PID.Target_Speed++;//
			PID.Target_Speed=PID.Target_Speed>SPEED_X?SPEED_X:(PID.Target_Speed<-SPEED_X?(-SPEED_X):PID.Target_Speed);//�޷�
			
			/*����*/
			if((Left==0)&&(Right==0))
			{
			 if(PID.Turn_Speed>0)
			 {
				(PID.Turn_Speed<=1)?(PID.Turn_Speed=0):(PID.Turn_Speed--);	
			 }
			 else
			 {
			  (PID.Turn_Speed>=-1)?(PID.Turn_Speed=0):(PID.Turn_Speed++);	
			 }
			}				
			if(Left==1)PID.Turn_Speed++;	//��ת
			if(Right==1)PID.Turn_Speed--;	//��ת
			PID.Turn_Speed=PID.Turn_Speed>SPEED_Z?SPEED_Z:(PID.Turn_Speed<-SPEED_Z?(-SPEED_Z):PID.Turn_Speed);//�޷�( (20*100) * 100  )
			
			/*ת��Լ��*/
			if((Left==0)&&(Right==0))PID.Turn_Kd=-0.8;//��������ת��ָ�����ת��Լ��
			else if((Left==1)||(Right==1))PID.Turn_Kd=0;//������ת��ָ����յ�����ȥ��ת��Լ��			
		  Key_Proc();//��ȡ����״̬	
			/*********************************************************************************************/			
			//2.������ѹ��ջ������У�����������������
			PID.Velocity_out=Velocity(PID.Target_Speed,Encoder_Left,Encoder_Right);	             //�ٶȻ�
			PID.Vertical_out=Vertical(PID.Med_Angle,Angle_Balance,Gyro_Balance);//ֱ����
			PID.Turn_out=Turn(Gyro_Turn,PID.Turn_Speed);																//ת��
			
			PWM_out=PID.Vertical_out-PID.Vertical_Kp*PID.Velocity_out;//�������
			//3.�ѿ�����������ص�����ϣ�������յĵĿ��ơ�
			MOTO1=PWM_out-PID.Turn_out;//����
			MOTO2=PWM_out+PID.Turn_out;//�ҵ��
			Limit(&MOTO1,&MOTO2);//PWM�޷�	
			if((Angle_Balance>80)||(Angle_Balance<-80)||Voltage<1130||BalanceState==0)//С���Ѿ��㵹���ߵ�ѹ���ͻ���δʹ�ܣ��رյ��
			{
			  Load(0,0);//���ص�����ϡ�
			}
			else
				Load(MOTO1,MOTO2);//���ص������			 			
		}
	}
}




/*------------------------------------------
 ��������:ֱ����PD������
 ����˵��:
	->��ڲ����������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
	->���ڲ�����ֱ�������
------------------------------------------*/
int Vertical(float Med,float Angle,float gyro_X)
{
	int PWM_out;
	
	PWM_out=PID.Vertical_Kp*(Angle-Med)+PID.Vertical_Kd*(gyro_X-0);//��1��
	return PWM_out;
}


/*------------------------------------------
 ��������:�ٶȻ�PI������
 ����˵��:
	->��ڲ����������ٶȶȡ����������ֵ���ұ�������ֵ
	->���ڲ������ٶȻ����
------------------------------------------*/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;//��2��
	float a=0.7;//��3��
	
	//1.�����ٶ�ƫ��
	Encoder_Err=(encoder_left+encoder_right)-Target;//��ȥ���
	//2.���ٶ�ƫ����е�ͨ�˲�
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout_last=EnC_Err_Lowout;//��ֹ�ٶȹ����Ӱ��ֱ����������������
	//3.���ٶ�ƫ����֣����ֳ�λ��
	Encoder_S+=EnC_Err_Lowout;//��4��
	//4.�����޷�
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	if((Angle_Balance>80)||(Angle_Balance<-80)||Voltage<1130||BalanceState==0)//С���Ѿ��㵹���ߵ�ѹ���ͻ���δʹ�ܣ���������
	{
		Encoder_S=0;//���ص�����ϡ�
	}
	//5.�ٶȻ������������
	PWM_out=PID.Velocity_Kp*EnC_Err_Lowout+PID.Velocity_Ki*Encoder_S;//��5��
	return PWM_out;
}




/*------------------------------------------
 ��������:ת�򻷿�����
 ����˵��:
	->��ڲ�����Z����ٶȡ�ϵ��*ң������
	->���ڲ�����ת�����
------------------------------------------*/
int Turn(int gyro_Z,int RC)
{
	int PWM_out;
	//�ⲻ��һ���ϸ��PD��������Kd��Ե���ת���Լ������Kp��Ե���ң�ص�ת��	
	PWM_out=PID.Turn_Kd*gyro_Z+ PID.Turn_Kp*RC;
	return PWM_out;
}


/*------------------------------------------
 ��������:��ȡ�Ƕ��㷨 ->����ƽ��֮��MPU6050Դ��
 ����˵��:
	->��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
------------------------------------------*/
void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_Angle,Accel_Z,Gyro_X,Gyro_Z;
			Temperature=Read_Temperature();      //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	    if(way==1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
			{	
					Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
					Angle_Balance=-Roll;             //===����ƽ�����
					Gyro_Balance=-gyro[0];            //===����ƽ����ٶ�
					Gyro_Turn=gyro[2];               //===����ת����ٶ�
				  Acceleration_Z=accel[2];         //===����Z����ٶȼ�
			}			
      else
      {
			Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡY��������
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		  Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		  if(Gyro_X>32768)  Gyro_X-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��������ת��
	  	if(Accel_Y>32768) Accel_Y-=65536;                      //��������ת��
		  if(Accel_Z>32768) Accel_Z-=65536;                      //��������ת��
			Gyro_Balance=Gyro_X;                                  //����ƽ����ٶ�
	   	Accel_Angle=atan2(Accel_Y,Accel_Z)*180/3.14159265;      //�������	
			Gyro_X=Gyro_X/16.4;                                    //����������ת��	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Angle,Gyro_X);//�������˲�	
			else if(Way_Angle==3)   Yijielvbo(Accel_Angle,Gyro_X);    //�����˲�
	    Angle_Balance=angle;                                   //����ƽ�����
			Gyro_Turn=Gyro_Z;                                      //����ת����ٶ�
			Acceleration_Z=Accel_Z;                                //===����Z����ٶȼ�	
		}
}



/*------------------------------------------
 ��������:PID������ʼ��
 ����˵��:��
------------------------------------------*/
void PID_ParaInit(void)
{
	PID.Med_Angle=1;
	
	PID.Vertical_Kp=-380;
	PID.Vertical_Kd=-1.5;
	 
	PID.Velocity_Kp=0.52;
	PID.Velocity_Ki=0.0026;
	
	PID.Turn_Kp=-30;
	PID.Turn_Kd=-0.8;
		
	PID.Vertical_out=0;	
	PID.Velocity_out=0;
	PID.Turn_out=0;
}


/*------------------------------------------
 ��������:����������
 ����˵��:��
------------------------------------------*/
void Key_Proc(void)
{
  uint8_t KeyRead;
	KeyRead= KEY_Scan(0);
	if(KeyRead)
	{
	 switch (KeyRead)
	 {
		 case KEY1_PRES:MCU_Reset();break;//��λ
		 case KEY2_PRES:if(BalanceState==0){Way_Angle=Way_Angle%3+1;}break;//MPU6050�˲��㷨�л�(����δʹ��������������л��㷨)
		 case KEY3_PRES:BalanceState=!BalanceState;break;//ƽ�⳵ʹ�ܿ���
		 default:break;
	 }	
	}
}

/*------------------------------------------
 ��������:OLED��ʾ����
 ����˵��:��
------------------------------------------*/
void Oled_Proc(void)
{
	//�ڶ�����ʾ����//
	OLED_ShowString(0,0,"   BalanceCar   ",16 );
	
	//�ڶ�����ʾ�㷨//	
	if(Way_Angle==1) {OLED_ShowString(0,2,"Filter:DMP   ",12);	PID.Med_Angle=-0.2;}
	else if(Way_Angle==2)	{OLED_ShowString(0,2,"Filter:Kalman",12);	PID.Med_Angle=0.5;}
	else if(Way_Angle==3)	{OLED_ShowString(0,2,"Filter:Hubu  ",12);	PID.Med_Angle=-0.1;}
	
	//��������ʾ�¶�//	
	OLED_ShowString(0,3,"Temp:",12);
	OLED_ShowNumber(40,3,Temperature/10,2,12);
	OLED_ShowString(52,3,".",12);
	OLED_ShowNumber(59,3,Temperature%10,1,12);
	OLED_ShowString(71,3,"'C",12);		
	
	//��������ʾ��ѹ//	
	OLED_ShowString(0,4,"Voltage:",12);
	OLED_ShowNumber(54+10,4,Voltage/100,2,12);
	OLED_ShowString(66+10,4,".",12);
	OLED_ShowNumber(73+10,4,Voltage%100,2,12);
	OLED_ShowString(85+10,4,"V",12);
	
	//��������ʾ�����ǽǶ�//
	OLED_ShowString(0,5,"Angle:",12);
	if((int)Angle_Balance>=0)
	{
		OLED_ShowString(44+6,5,"+",12);
		if((int)Angle_Balance%1000>99)
		{
	   OLED_ShowNumber(48+10,5,(int)Angle_Balance%1000,3,12);
		 OLED_ShowString(68+10,5,"'",12);
		}
		else
		{
	   OLED_ShowNumber(48+10,5,(int)Angle_Balance%1000,2,12);
		 OLED_ShowString(60+10,5,"'  ",12);		
		}
	}
		else 		
	{
	  OLED_ShowString(44+6,5,"-",12);				
		if((7000-(int)(Angle_Balance)%1000)<7099)
		{
		 OLED_ShowNumber(48+10,5,(7000-(int)(Angle_Balance)%1000),2,12);
		 OLED_ShowString(60+10,5,"'  ",12);
		}
		else
		{
		 OLED_ShowNumber(48+10,5,(7000-(int)(Angle_Balance)%1000),3,12);
		 OLED_ShowString(68+10,5,"'",12);
		}
	}
	
	//��������ʾ���ֱ�����//
	OLED_ShowString(0,6,"EncoLeft:",12);
	if( Encoder_Left<0)
	{
	  OLED_ShowString(62+10,6,"-",12);
		if(abs(Encoder_Left)<=9)
		{
		 OLED_ShowNumber(70+10,6,abs(Encoder_Left),1,12);
		 OLED_ShowString(76+10,6,"    ",12);
		}	
		else if((abs(Encoder_Left)>9)&&(abs(Encoder_Left)<=99))
		{
		 OLED_ShowNumber(70+10,6,abs(Encoder_Left),2,12);
		 OLED_ShowString(82+10,6,"    ",12);
		}
		else if(abs(Encoder_Left)>99)
		{
		 OLED_ShowNumber(70+10,6,abs(Encoder_Left),3,12);
		}
	}
	else
	{
	  OLED_ShowString(62+10,6,"+",12);
		if(Encoder_Left<=9)
		{
		 OLED_ShowNumber(70+10,6,Encoder_Left,1,12);
			OLED_ShowString(76+10,6,"    ",12);
		}	
		else if((Encoder_Left>9)&&(Encoder_Left<=99))
		{
		 OLED_ShowNumber(70+10,6,Encoder_Left,2,12);
		 OLED_ShowString(82+10,6,"    ",12);
		}
		else if(Encoder_Left>99)
		{
		 OLED_ShowNumber(70+10,6,Encoder_Left,3,12);
		}
	}
	//��������ʾ���ֱ�����//
		OLED_ShowString(0,7,"EncoRight:",12);
	if( Encoder_Right<0)
	{
	  OLED_ShowString(62+15,7,"-",12);
		if(abs(Encoder_Right)<=9)
		{
		 OLED_ShowNumber(70+15,7,abs(Encoder_Right),1,12);
		 OLED_ShowString(76+15,7,"    ",12);
		}	
		else if((abs(Encoder_Right)>9)&&(abs(Encoder_Right)<=99))
		{
		 OLED_ShowNumber(70+15,7,abs(Encoder_Right),2,12);		
		 OLED_ShowString(82+15,7,"    ",12);
		}
		else if(abs(Encoder_Right)>99)
		{
		 OLED_ShowNumber(70+15,7,abs(Encoder_Right),3,12);
		}
	}
	else
	{
	  OLED_ShowString(62+15,7,"+",12);
		if(Encoder_Right<=9)
		{
		 OLED_ShowNumber(70+15,7,Encoder_Right,1,12);
		 OLED_ShowString(76+15,7,"    ",12);
		}	
		else if((Encoder_Right>9)&&(Encoder_Right<=99))
		{
		 OLED_ShowNumber(70+15,7,Encoder_Right,2,12);
		 OLED_ShowString(82+15,7,"    ",12);
		}
		else if(Encoder_Right>99)
		{
		 OLED_ShowNumber(70+15,7,Encoder_Right,3,12);
		}
	}	

}

/*****************************************************END OF FILE*********************************************************/	

