#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


void uart3_init(u32 bound);					//����1��ʼ������
void USART3_IRQHandler(void);     	//����1�жϷ������

void USART3_Send_String(char *String);
#endif


