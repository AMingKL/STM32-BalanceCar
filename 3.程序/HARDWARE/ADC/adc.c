/**
  *************************************************************************************************************************
  * @file    : adc.c
  * @author  : AMKL
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : ADCģ��C�ļ�
  *************************************************************************************************************************
  * @attention
  *
  * 
  *
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "adc.h"



/**************************************************************************
�������ܣ�ACD��ʼ��
��ڲ�������
����ֵ����
**************************************************************************/
void  Adc_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	//����ģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
}		

/**************************************************************************
�������ܣ�AD����
��ڲ�����ADC1��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������		 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}


/**************************************************************************
�������ܣ���ȡ��ص�ѹ 
��ڲ�������
����  ֵ����ص�ѹ ��λMV
**************************************************************************/
int Get_battery_volt(void)   
{  
	int Volt;//��ص�ѹ
	Volt=Get_Adc(Battery_Ch)*3.3*11*100/4096;	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
	return Volt;
}



/*****************************************************END OF FILE*********************************************************/	





