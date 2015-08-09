/*
 * ESP6288.c
 *
 *  Created on: 2015-1-17
 *      Author: 天马星空
 */
#include<msp430f5529.h>


void UART_Init()
{

	UCA0CTL1 |= UCSWRST;    //复位状态
	UCA0CTL0 = 0;          //普通异步串口模式
	UCA0CTL1 |= UCSSEL_1;    //时钟选择为ASCLK 现在是32768HZ
    P3SEL |= BIT3 | BIT4;  //设置为第二功能口
	//波特率设置
	UCA0BR0 = 3;           //N = INT(32768/9600)
	UCA0BR1 = 0;
	UCA0MCTL |= UCBRS_3;    //   round(32768/9600 - N）
	UCA0CTL1 &=~ UCSWRST;  //启动状态

	UCA0IE |= UCRXIE;    //打开接收终中断
	_EINT();     //开总中断
}

void Esp_Delay()
{
	unsigned int i,k;
	for(i=1000;i>0;i--)
		for(k=10000;k>0;k--);
}

//void Esp_Send()
//{
//	unsigned char EspSend[]="AT+CIPSEND=2,8\n"
//}
void ESP_Init()
{
	unsigned char temp,i;
	unsigned char len1,len2;
	unsigned char EspATRST[10]="AT+RST\n";
	unsigned char EspATAMX[15]="AT+CIPMUX=1\n";
	unsigned char EspATST[50]="AT+CIPSTART=1,\"TCP\",\"192.168.1.101\",1234\n";
	unsigned char check[150];
	//unsigned char EspATCWJAP[50] = "AT";//"AT+CWJAP=\"55555\",\"15011501\"\n";
	len1 = strlen(EspATAMX);
	len2 = strlen(EspATST);
	//len3 = strlen(EspATRST);
//    _DINT();
//	for(temp = 0;len3>0;temp++,len3--)
//	{
//		while(!(UCA0IFG&UCTXIFG));
//		UCA0TXBUF = EspATRST[temp];
//	}  //复位
//
//	while(check !='K')
//	{
//		while(!(UCA0IFG&UCRXIFG));
//		check = UCA0RXBUF;
//	}
//	check = 0;
	for(temp = 0;len1>0;temp++,len1--)
	{
		while(!(UCA0IFG&UCTXIFG));
		UCA0TXBUF = EspATAMX[temp];
	}  //多连接模式



	for(temp = 0;len2>0;temp++,len2--)
	{
		while(!(UCA0IFG&UCTXIFG));
		UCA0TXBUF = EspATST[temp];
	}

//	while(1)
//	{
//		while(!(UCA0IFG&UCRXIFG));
//		check[i++] = UCA0RXBUF;
//
//	}
//	while(check !='L')
//	{
//		while(!(UCA0IFG&UCRXIFG));
//		check = UCA0RXBUF;
//	}
//	_EINT();
}
