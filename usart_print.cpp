#include "usart_print.h"

void U_Print(USART_TypeDef * USARTx, uint8_t a)
{
	char temp[20] = "";
	int leng = sprintf(temp, "%d", a);
	for (int i = 0; i < leng; i++)
	{
			while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
			USART_SendData(USARTx,temp[i]);
			USART_ClearFlag(USARTx, USART_FLAG_TXE);

	}
}

void U_Print_Char(USART_TypeDef * USARTx, char *s)
{
	while (*s)
	{
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
		USART_SendData(USARTx, *s);
		USART_ClearFlag(USARTx, USART_FLAG_TXE);
		*s++;
	}
}

void U_Print_float(USART_TypeDef * USARTx, float a)
{
	char temp[20] = "";
	int leng = sprintf(temp, "%0.2f", a);
	for (int i = 0; i < leng; i++)
	{
			while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
			USART_SendData(USARTx,temp[i]);
			USART_ClearFlag(USARTx, USART_FLAG_TXE);
	}
}
