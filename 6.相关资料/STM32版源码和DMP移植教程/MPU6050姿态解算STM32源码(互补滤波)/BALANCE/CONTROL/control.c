#include "control.h"	
#include "filter.h"	
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
int TIM1_UP_IRQHandler(void)  
{    
	if(TIM1->SR&0X0001)//5ms定时中断
	{   
		  TIM1->SR&=~(1<<0);                                       //===清除定时器1中断标志位		 

			Get_Angle(Way_Angle);                                    //===更新姿态	
		
	}       	
	 return 0;	  
} 

/**************************************************************************
函数功能：获取角度
入口参数：获取角度的算法 1：无  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_Y;
	    if(way==1)                                      //DMP没有涉及到严格的时序问题，在主函数读取
			{	
			}			
      else
      {
			Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
			Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L);    //读取Z轴陀螺仪
		  Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度记
	 		if(Gyro_Y>32768)  Gyro_Y-=65536;     //数据类型转换  也可通过short强制类型转换
			if(Accel_Z>32768)  Accel_Z-=65536;     //数据类型转换
	  	if(Accel_X>32768) Accel_X-=65536;    //数据类型转换
	   	Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //计算与地面的夹角	
		  Gyro_Y=Gyro_Y/16.4;                                    //陀螺仪量程转换	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Y,-Gyro_Y);//卡尔曼滤波	
			else if(Way_Angle==3)   Yijielvbo(Accel_Y,-Gyro_Y);    //互补滤波
	    Angle_Balance=angle;                                   //更新平衡倾角
	  	}
}
