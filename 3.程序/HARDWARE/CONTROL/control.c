/**
  *************************************************************************************************************************
  * @file    : control.c
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : 驱动程序--控制函数C文件
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
#define SPEED_X 23 //俯仰(前后)最大设定速度
#define SPEED_Z 100//偏航(左右)最大设定速度 

BlanceCar_PID PID;//pid结构体

uint8_t Way_Angle=1;//获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
float Angle_Balance,Gyro_Balance,Gyro_Turn;//平衡倾角 平衡陀螺仪 转向陀螺仪
float Acceleration_Z;//Z轴加速度计 

int Encoder_Left,Encoder_Right;//编码器数据（速度）
int Voltage;//读取电压变量
int PWM_MAX=7200,PWM_MIN=-7200;//PWM限幅变量
int MOTO1,MOTO2;//电机装载PWM变量

uint8_t ADC_TimeCnt;//ADC读取刷新时间变量
uint8_t BalanceState;//平衡车状态
int Temperature;//显示温度




/*------------------------------------------
 函数功能:MPU6050中断服务函数
 函数说明:采集数据，控制小车			
------------------------------------------*/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5)!=0)//一级判定
	{
		int PWM_out;
		if(PBin(5)==0)//二级判定
		{
			EXTI_ClearITPendingBit(EXTI_Line5);//清除中断标志位
			if(delay_flag==1)
		  {
			 if(++delay_50ms==10)	 delay_50ms=0,delay_flag=0;//给主函数提供50ms的精准延时
		  }			
			//1.采集编码器数据&MPU6050角度&电压信息。
			Encoder_Left= Read_Speed(2);
			Encoder_Right=Read_Speed(4);		
			Get_Angle(Way_Angle); 			
			ADC_TimeCnt==100?ADC_TimeCnt=0,Voltage=Get_battery_volt():ADC_TimeCnt++;//0.1s读取一次电压		
			/*********************************************************************************************/
			/*前后*/
			if((Fore==0)&&(Back==0))
			{
			  if(PID.Target_Speed>0)
				{
				 (PID.Target_Speed<=1)?(PID.Target_Speed=0):(PID.Target_Speed--);//未接受到前进后退指令-->速度清零，稳在原地
				}
				else
				{
				 (PID.Target_Speed>=-1)?(PID.Target_Speed=0):(PID.Target_Speed++);
				}
			}	
			
			if(Fore==1)PID.Target_Speed--;//前进1标志位拉高-->需要前进
			if(Back==1)PID.Target_Speed++;//
			PID.Target_Speed=PID.Target_Speed>SPEED_X?SPEED_X:(PID.Target_Speed<-SPEED_X?(-SPEED_X):PID.Target_Speed);//限幅
			
			/*左右*/
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
			if(Left==1)PID.Turn_Speed++;	//左转
			if(Right==1)PID.Turn_Speed--;	//右转
			PID.Turn_Speed=PID.Turn_Speed>SPEED_Z?SPEED_Z:(PID.Turn_Speed<-SPEED_Z?(-SPEED_Z):PID.Turn_Speed);//限幅( (20*100) * 100  )
			
			/*转向约束*/
			if((Left==0)&&(Right==0))PID.Turn_Kd=-0.8;//若无左右转向指令，则开启转向约束
			else if((Left==1)||(Right==1))PID.Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束			
		  Key_Proc();//读取按键状态	
			/*********************************************************************************************/			
			//2.将数据压入闭环控制中，计算出控制输出量。
			PID.Velocity_out=Velocity(PID.Target_Speed,Encoder_Left,Encoder_Right);	             //速度环
			PID.Vertical_out=Vertical(PID.Med_Angle,Angle_Balance,Gyro_Balance);//直立环
			PID.Turn_out=Turn(Gyro_Turn,PID.Turn_Speed);																//转向环
			
			PWM_out=PID.Vertical_out-PID.Vertical_Kp*PID.Velocity_out;//最终输出
			//3.把控制输出量加载到电机上，完成最终的的控制。
			MOTO1=PWM_out-PID.Turn_out;//左电机
			MOTO2=PWM_out+PID.Turn_out;//右电机
			Limit(&MOTO1,&MOTO2);//PWM限幅	
			if((Angle_Balance>80)||(Angle_Balance<-80)||Voltage<1130||BalanceState==0)//小车已经倾倒或者电压过低或者未使能，关闭电机
			{
			  Load(0,0);//加载到电机上。
			}
			else
				Load(MOTO1,MOTO2);//加载到电机。			 			
		}
	}
}




/*------------------------------------------
 函数功能:直立环PD控制器
 函数说明:
	->入口参数：期望角度、真实角度、真实角速度
	->出口参数：直立环输出
------------------------------------------*/
int Vertical(float Med,float Angle,float gyro_X)
{
	int PWM_out;
	
	PWM_out=PID.Vertical_Kp*(Angle-Med)+PID.Vertical_Kd*(gyro_X-0);//【1】
	return PWM_out;
}


/*------------------------------------------
 函数功能:速度环PI控制器
 函数说明:
	->入口参数：期望速度度、左编码器数值、右编码器数值
	->出口参数：速度环输出
------------------------------------------*/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;//【2】
	float a=0.7;//【3】
	
	//1.计算速度偏差
	Encoder_Err=(encoder_left+encoder_right)-Target;//舍去误差
	//2.对速度偏差进行低通滤波
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
	//3.对速度偏差积分，积分出位移
	Encoder_S+=EnC_Err_Lowout;//【4】
	//4.积分限幅
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	if((Angle_Balance>80)||(Angle_Balance<-80)||Voltage<1130||BalanceState==0)//小车已经倾倒或者电压过低或者未使能，消除积分
	{
		Encoder_S=0;//加载到电机上。
	}
	//5.速度环控制输出计算
	PWM_out=PID.Velocity_Kp*EnC_Err_Lowout+PID.Velocity_Ki*Encoder_S;//【5】
	return PWM_out;
}




/*------------------------------------------
 函数功能:转向环控制器
 函数说明:
	->入口参数：Z轴角速度、系数*遥控数据
	->出口参数：转向环输出
------------------------------------------*/
int Turn(int gyro_Z,int RC)
{
	int PWM_out;
	//这不是一个严格的PD控制器，Kd针对的是转向的约束，但Kp针对的是遥控的转向。	
	PWM_out=PID.Turn_Kd*gyro_Z+ PID.Turn_Kp*RC;
	return PWM_out;
}


/*------------------------------------------
 函数功能:获取角度算法 ->引用平衡之家MPU6050源码
 函数说明:
	->入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
------------------------------------------*/
void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_Angle,Accel_Z,Gyro_X,Gyro_Z;
			Temperature=Read_Temperature();      //===读取MPU6050内置温度传感器数据，近似表示主板温度。
	    if(way==1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
			{	
					Read_DMP();                      //===读取加速度、角速度、倾角
					Angle_Balance=-Roll;             //===更新平衡倾角
					Gyro_Balance=-gyro[0];            //===更新平衡角速度
					Gyro_Turn=gyro[2];               //===更新转向角速度
				  Acceleration_Z=accel[2];         //===更新Z轴加速度计
			}			
      else
      {
			Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取Y轴陀螺仪
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		  Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		  if(Gyro_X>32768)  Gyro_X-=65536;                       //数据类型转换  也可通过short强制类型转换
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换
	  	if(Accel_Y>32768) Accel_Y-=65536;                      //数据类型转换
		  if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换
			Gyro_Balance=Gyro_X;                                  //更新平衡角速度
	   	Accel_Angle=atan2(Accel_Y,Accel_Z)*180/3.14159265;      //计算倾角	
			Gyro_X=Gyro_X/16.4;                                    //陀螺仪量程转换	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Angle,Gyro_X);//卡尔曼滤波	
			else if(Way_Angle==3)   Yijielvbo(Accel_Angle,Gyro_X);    //互补滤波
	    Angle_Balance=angle;                                   //更新平衡倾角
			Gyro_Turn=Gyro_Z;                                      //更新转向角速度
			Acceleration_Z=Accel_Z;                                //===更新Z轴加速度计	
		}
}



/*------------------------------------------
 函数功能:PID参数初始化
 函数说明:无
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
 函数功能:按键处理函数
 函数说明:无
------------------------------------------*/
void Key_Proc(void)
{
  uint8_t KeyRead;
	KeyRead= KEY_Scan(0);
	if(KeyRead)
	{
	 switch (KeyRead)
	 {
		 case KEY1_PRES:MCU_Reset();break;//复位
		 case KEY2_PRES:if(BalanceState==0){Way_Angle=Way_Angle%3+1;}break;//MPU6050滤波算法切换(仅在未使能启动情况下能切换算法)
		 case KEY3_PRES:BalanceState=!BalanceState;break;//平衡车使能开关
		 default:break;
	 }	
	}
}

/*------------------------------------------
 函数功能:OLED显示函数
 函数说明:无
------------------------------------------*/
void Oled_Proc(void)
{
	//第二行显示名称//
	OLED_ShowString(0,0,"   BalanceCar   ",16 );
	
	//第二行显示算法//	
	if(Way_Angle==1) {OLED_ShowString(0,2,"Filter:DMP   ",12);	PID.Med_Angle=-0.2;}
	else if(Way_Angle==2)	{OLED_ShowString(0,2,"Filter:Kalman",12);	PID.Med_Angle=0.5;}
	else if(Way_Angle==3)	{OLED_ShowString(0,2,"Filter:Hubu  ",12);	PID.Med_Angle=-0.1;}
	
	//第三行显示温度//	
	OLED_ShowString(0,3,"Temp:",12);
	OLED_ShowNumber(40,3,Temperature/10,2,12);
	OLED_ShowString(52,3,".",12);
	OLED_ShowNumber(59,3,Temperature%10,1,12);
	OLED_ShowString(71,3,"'C",12);		
	
	//第四行显示电压//	
	OLED_ShowString(0,4,"Voltage:",12);
	OLED_ShowNumber(54+10,4,Voltage/100,2,12);
	OLED_ShowString(66+10,4,".",12);
	OLED_ShowNumber(73+10,4,Voltage%100,2,12);
	OLED_ShowString(85+10,4,"V",12);
	
	//第五行显示俯仰角角度//
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
	
	//第六行显示左轮编码器//
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
	//第七行显示右轮编码器//
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

