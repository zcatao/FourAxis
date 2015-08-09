/*
 * Display1.c
 *
 *  Created on: 2015年6月3日
 *      Author: panfeng
 */
#include "MSP430F5529.h"
#include "Display1.h"
#define uchar unsigned char
#define uint unsigned int
/******************************宏定义 OLED***********************************/
/******************************宏定义 OLED***********************************/
/******************************宏定义 OLED***********************************/
#define LED_SCL1      P6OUT |= BIT0
#define LED_SCL0      P6OUT &= ~BIT0
#define LED_SDA1      P6OUT |= BIT1
#define LED_SDA0      P6OUT &= ~BIT1
#define LED_RST1      P6OUT |= BIT2
#define LED_RST0      P6OUT &= ~BIT2
#define LED_DC1       P6OUT |= BIT3
#define LED_DC0       P6OUT &= ~BIT3
#define LED_dir       P6DIR

/*******************************************OLED 曲线*****************************/
/*******************************************OLED 曲线*****************************/
/*******************************************OLED 曲线*****************************/
//全局变量  8行X6列

const uchar F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   //sp0
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !1
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "2
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #3
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $4
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %5
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &6
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '7
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (8
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )9
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *10
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +11
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,12
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -13
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .14
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /15
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 016
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 117
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 218
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 319
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 420
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 521
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 622
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 723
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 824
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 925
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :26
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;27
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <28
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =29
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >30
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?31
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @32
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A33
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B34
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C35
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D36
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E37
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F38
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G39
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H40
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I41
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J42
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K43
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L44
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M45
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N46
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O47
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P48
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q49
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R50
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S51
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T52
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U53
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V54
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W55
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X56
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y57
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z58
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [59
    { 0x00, 0x02, 0x04, 0x08, 0x10, 0x20 },   // \60
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]61
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^62
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _63
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '64
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a65
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b66
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c67
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d68
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e69
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f70
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g71
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h72
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i73
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j74
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k75
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l76
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m77
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n78
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o79
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p80
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q81
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r82
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s83
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t84
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u85
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v86
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w87
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x88
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y89
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z90
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines91
};




//向OLED写数据
void LED_WrDat(uchar ucData)
{
    uchar i = 8;
    LED_DC1;
    LED_SCL0;
    while (i--)
    {
        if(ucData & 0x80)
            LED_SDA1;
        else
            LED_SDA0;
        LED_SCL1;
        _NOP();
        LED_SCL0;
        ucData <<= 1;
    }
}
//向OLED写命令
void LED_WrCmd(uchar ucCmd)
{
    uchar i = 8;
    LED_DC0;
    LED_SCL0;
    while (i--)
    {
        if(ucCmd & 0x80)
           LED_SDA1;
        else
            LED_SDA0;
        LED_SCL1;
        _NOP();
        LED_SCL0;
        ucCmd <<= 1;
    }
}
//设置坐标
void LED_SetPos(uchar X, uchar Y)
{
    LED_WrCmd(0xb0 + Y);
    LED_WrCmd(((X & 0xf0) >> 4) | 0x10);
    LED_WrCmd((X & 0x0f) | 0x00);
}

void LED_Fill(uchar ucData)
{
    uchar ucPage,ucColumn;

    for(ucPage = 0; ucPage < 8; ucPage++)
    {
        LED_WrCmd(0xb0 + ucPage);  //0xb0+0~7表示页0~7
        LED_WrCmd(0x00);           //0x00+0~16表示将128列分成16组其地址在某组中的第几列
        LED_WrCmd(0x10);           //0x10+0~16表示将128列分成16组其地址所在第几组
        for(ucColumn = 0; ucColumn < 128; ucColumn++)
        {
            LED_WrDat(ucData);
        }
    }
}

void SetStartColumn(uchar ucData)
{
    LED_WrCmd(0x00+ucData % 16);   // Set Lower Column Start Address for Page Addressing Mode
                                   // Default => 0x00
    LED_WrCmd(0x10+ucData / 16);   // Set Higher Column Start Address for Page Addressing Mode
                                   // Default => 0x10
}

void SetAddressingMode(uchar ucData)
{
    LED_WrCmd(0x20);        // Set Memory Addressing Mode
    LED_WrCmd(ucData);      // Default => 0x02
                            // 0x00 => Horizontal Addressing Mode
                            // 0x01 => Vertical Addressing Mode
                            // 0x02 => Page Addressing Mode
}
void SetColumnAddress(uchar a, uchar b)
{
    LED_WrCmd(0x21);        // Set Column Address
    LED_WrCmd(a);           // Default => 0x00 (Column Start Address)
    LED_WrCmd(b);           // Default => 0x7F (Column End Address)
}

void SetPageAddress(uchar a, uchar b)
{
    LED_WrCmd(0x22);        // Set Page Address
    LED_WrCmd(a);           // Default => 0x00 (Page Start Address)
    LED_WrCmd(b);           // Default => 0x07 (Page End Address)
}

void SetStartLine(uchar ucData)
{
    LED_WrCmd(0x40|ucData); // Set Display Start Line
                            // Default => 0x40 (0x00)
}

void SetContrastControl(uchar ucData)
{
    LED_WrCmd(0x81);        // Set Contrast Control
    LED_WrCmd(ucData);      // Default => 0x7F
}

void SetChargePump(uchar ucData)
{
    LED_WrCmd(0x8D);        // Set Charge Pump
    LED_WrCmd(0x10|ucData); // Default => 0x10
                            // 0x10 (0x00) => Disable Charge Pump
                            // 0x14 (0x04) => Enable Charge Pump
}

void SetSegmentRemap(uchar ucData)
{
    LED_WrCmd(0xA0|ucData); // Set Segment Re-Map
                            // Default => 0xA0
                            // 0xA0 (0x00) => Column Address 0 Mapped to SEG0
                            // 0xA1 (0x01) => Column Address 0 Mapped to SEG127
}

void SetEntireDisplay(uchar ucData)
{
    LED_WrCmd(0xA4|ucData); // Set Entire Display On / Off
                            // Default => 0xA4
                            // 0xA4 (0x00) => Normal Display
                            // 0xA5 (0x01) => Entire Display On
}

void SetInverseDisplay(uchar ucData)
{
    LED_WrCmd(0xA6|ucData); // Set Inverse Display On/Off
                            // Default => 0xA6
                            // 0xA6 (0x00) => Normal Display
                            // 0xA7 (0x01) => Inverse Display On
}

void SetMultiplexRatio(uchar ucData)
{
    LED_WrCmd(0xA8);        // Set Multiplex Ratio
    LED_WrCmd(ucData);      // Default => 0x3F (1/64 Duty)
}

void SetDisplayOnOff(uchar ucData)
{
    LED_WrCmd(0xAE|ucData); // Set Display On/Off
                            // Default => 0xAE
                            // 0xAE (0x00) => Display Off
                            // 0xAF (0x01) => Display On
}

void SetStartPage(uchar ucData)
{
    LED_WrCmd(0xB0|ucData); // Set Page Start Address for Page Addressing Mode
                            // Default => 0xB0 (0x00)
}

void SetCommonRemap(uchar ucData)
{
    LED_WrCmd(0xC0|ucData); // Set COM Output Scan Direction
                            // Default => 0xC0
                            // 0xC0 (0x00) => Scan from COM0 to 63
                            // 0xC8 (0x08) => Scan from COM63 to 0
}

void SetDisplayOffset(uchar ucData)
{
    LED_WrCmd(0xD3);        // Set Display Offset
    LED_WrCmd(ucData);      // Default => 0x00
}

void SetDisplayClock(uchar ucData)
{
    LED_WrCmd(0xD5);        // Set Display Clock Divide Ratio / Oscillator Frequency
    LED_WrCmd(ucData);      // Default => 0x80
                            // D[3:0] => Display Clock Divider
                            // D[7:4] => Oscillator Frequency
}

void SetPrechargePeriod(uchar ucData)
{
    LED_WrCmd(0xD9);        // Set Pre-Charge Period
    LED_WrCmd(ucData);      // Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
                            // D[3:0] => Phase 1 Period in 1~15 Display Clocks
                            // D[7:4] => Phase 2 Period in 1~15 Display Clocks
}

void SetCommonConfig(uchar ucData)
{
    LED_WrCmd(0xDA);        // Set COM Pins Hardware Configuration
    LED_WrCmd(0x02|ucData); // Default => 0x12 (0x10)
                            // Alternative COM Pin Configuration
                            // Disable COM Left/Right Re-Map
}

void SetVCOMH(uchar ucData)
{
    LED_WrCmd(0xDB);        // Set VCOMH Deselect Level
    LED_WrCmd(ucData);      // Default => 0x20 (0.77*VCC)
}

void SetNop(void)
{
    LED_WrCmd(0xE3);        // Command for No Operation
}


//OLED初始化
void oled_Init(void)
{
    unsigned char i;
    LED_dir=0x0f;
    LED_SCL1;
    LED_RST0;
    for(i=0;i<100;i++)
        _NOP();   //从上电到下面开始初始化要有足够的时间，即等待RC复位完毕
    LED_RST1;
    SetDisplayOnOff(0x00);     // Display Off (0x00/0x01)
    SetChargePump(0x04);       // Enable Embedded DC/DC Converter (0x00/0x04)
    //SetAddressingMode(0x02);   // Set Page Addressing Mode (0x00/0x01/0x02)
    SetSegmentRemap(0x01);     // Set SEG/Column Mapping     0x00左右反置 0x01正常
    SetCommonRemap(0x08);      // Set COM/Row Scan Direction 0x00上下反置 0x08正常
    SetContrastControl(0xCF);  // Set SEG Output Current
    SetInverseDisplay(0x00);   // Disable Inverse Display On (0x00/0x01)
    SetDisplayOnOff(0x01);     // Display On (0x00/0x01)
    LED_Fill(0x00);            // 初始清屏
    LED_SetPos(0,0);
}

//显示一个6x8标准ASCII字符,X横坐标0~122,Y页范围0～7,ucData显示字符
void LED_P6x8Char(uchar X, uchar Y, uchar Data)
{
    uchar i, ucDataTmp;
    ucDataTmp =Data-32;
    if(X > 122)
    {
        X=0;
        Y++;
    }
    LED_SetPos(X, Y);
    for(i = 0; i < 6; i++)
        LED_WrDat(F6x8[ucDataTmp][i]);
}
//写入一组6x8标准ASCII字符串,x:0~122,y:0~7
void LED_P6x8Str(uchar ucIdxX, uchar ucIdxY, uchar ucDataStr[])
{
    uchar i, j, ucDataTmp;
    for (j=0;ucDataStr[j]!= '\0'; j++)
    {
        ucDataTmp = ucDataStr[j] - 32;
        if(ucIdxX > 122)
        {
            ucIdxX = 0;
            ucIdxY++;
        }
        LED_SetPos(ucIdxX,ucIdxY);
        for(i = 0; i < 6; i++)
            LED_WrDat(F6x8[ucDataTmp][i]);
        ucIdxX += 6;
    }
}



/**********************************浮点型转字符串************************************/
/**********************************浮点型转字符串************************************/
/**********************************浮点型转字符串************************************/
void sto_float(float n,unsigned char *st)
{
	long n_int,m_int;         //n_int为n的整数部分，m_int为浮点部分乘10的7方所得整数
	float n_float,m;     //n_float为n的浮点部分，m为浮点部分乘10的7方所得浮点数
	int i=0,j=0,k;            //i为存储数组的元素序号，j为倒序取数循环变量和正序取数元素序号，k为正序转换循环变量
        unsigned char symbol_n=0;    //数值符号标记
//	int pt;                      //pt为小数点在存储数组中的元素序号
	unsigned char tp[16]={"/0"};          //倒序取数临时数组
	if(n < 0)                   //数值为负，标记置1，数值取绝对值
	{
		symbol_n=1;
		n = -n;
	}
	n_int = (long)n;             //取整数部分绝对值
	n_float = n-n_int;           //取浮点部分绝对值


//	if((n !=0) && (n_int==0))  //如果该数只有浮点部分
//	{
//		tp[i++]='0';        //取数数组首元素为0
//	}
//	if(n_int != 0)              //如果该数整数部分不为0
//	{

		for(j=0;j<3;j++)    //倒序取整数，最大8位，取完退出循环
		{
			//if(n_int == 0)break;
			tp[i++] ='0'+n_int%10;
			n_int = n_int/10;
		}
//	}


	 m=n_float*pow(10,3);           //浮点部分乘10^7所得浮点数

	 m_int =(long)m;                //浮点部分乘10^7所得长整数

//	if(m_int != 0)                  //浮点部分不为0
//	{
//		pt = i;                 //pt指向‘.’的位置
		tp[i++] = '.';

		for(j=3;j>=1;j--)       //倒序取转化为整形的浮点数，最大7位，取完退出循环
		{
			//if(m_int== 0)break;
		    tp[i++] ='0'+m_int%10;
			m_int = m_int/10;
		}

//	}
//
//        else pt=i;                       //若浮点部分为0，小数点存储在倒序数组最后
        j=0;                            //正序存储起始位为1

	//顺序转换
        if(symbol_n==1) st[j++]='-'; //符号位为1，正序数组首位为符号
        else st[j++]='+';


		for(k=2;k>=0;k--)         //整数部分正序转换
		{
			st[j++] = tp[k];
		}
//                m_int =(int)m;               //m_int/10消耗之后，重新赋原值
//                if(m_int!=0)                    //浮点部分不为0，取整之后正序数组添加小数点
                st[j++]='.';
		for(k=6;k>3;k--)             //浮点部分正序转换
		{
			st[j++] = tp[k];
		}
}
