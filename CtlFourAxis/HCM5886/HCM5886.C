/*
 * HCM5886.C
 *
 *  Created on: 2015-3-14
 *      Author: ÃÏ¬Ì–«ø’
 */

//#include<msp430f5529.h>
//#include<i2c.h>
//#include<math.h>
//#define uchar unsigned char
//#define Hcm_RDADD 0x3D
//#define Hcm_WTADD 0x3C
//#define CRA_ADD 0x00
//#define CRB_ADD 0x01
//#define MOD_ADD 0x02
//#define XH 0x03
//
//uchar dat[7];
//
//void Hcm_Init()
//{
//	ISendChar(Hcm_WTADD,CRA_ADD,0x70);
//	ISendChar(Hcm_WTADD,MOD_ADD,0x00);
//}
//
//void Hcm_ReadDat()
//{
//	IRcvStr(Hcm_WTADD,XH,dat,6);
//}
//
//float Get_Angle()
//{
//	unsigned int X,Y,F;
//	float angle;
//	Hcm_ReadDat();
//	X = dat[0];
//	X = (X<<8) + dat[1];
//	Y = dat[2];
//	Y = (Y<<8) + dat[3];
//	F = dat[4];
//	F = (F<<8) + dat[5];
//	angle= atan2((double)Y,(double)X)*(180/3.14159265)+180;
//	return angle;
//}
