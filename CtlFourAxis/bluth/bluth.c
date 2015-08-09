/*
 * bluth.c
 *
 *  Created on: 2015��5��23��
 *      Author: �����ǿ�
 */
/*
 *   UART_Send  ��û�㶨
 */
#include"msp430f5529.h"
#define u8 unsigned char
#define vs16 unsigned int

u8 data_to_send[32];
extern Pitch_Moto_Offset;
extern Roll_Moto_Offset;
typedef struct
{
	float yall;
	float pitch;
	float roll;    //�����ǵ������Ƕ� ƫ�� ���� ����
}AttAngle;
extern AttAngle Att_Angle;

typedef struct
{
	vs16 pid_pitch_pwm;
	vs16 pid_roll_pwm;  //PID�����ֵ
	vs16 pid_yall_pwm;
}PidOut;
extern PidOut Pid_Out;


typedef struct
{
	vs16 CtlThro;	//ӳ������Űٷֱ�
	vs16 Thro;			//���ž���ռ�ձ�
	vs16 motor_front_pwm,motor_back_pwm,motor_left_pwm,motor_right_pwm; //�ĸ�������������ռ�ձ�
	u8 CtlYall;  //���Ʒ���
	u8 CtlRoll;  //����ROLL��б�Ƕ�
	u8 CtlPitch;	//����Pitch��б�Ƕ�
	int CtlXdirPwm;
	int CtlYdirPwm;
	float XdirPwm;
	float YdirPwm;
	int XdirAngle;
	int YdirAngle;
}PwmPara;
extern PwmPara Pwm_Para;

typedef struct  //�ɿ�PID�����Լ���ʼ��
{
	float In_KP;  //ROLL KP
	float In_KI;  //ROLL KI
	float In_KD;	//roll KD
	float Pos_Max;  //�����޷����
	float Neg_Max;	//�����޷� ��С
	float Last_output;//����ȥ����ϴ�����
}PidPara;
extern PidPara Pid_Para_Roll,Pid_Para_Pitch,Pid_Para_Yall;

typedef struct
{
	short Acc_X;
	short Acc_Y;
	short Acc_Z;
	short Gyro_X;
	short Gyro_Y;
	short Gyro_Z;
}SensorDat;
extern SensorDat Sensor_Dat;
extern u8 Pid_Set_OK;
u8 BYTE1(vs16 temp)
{
	return (temp>>8);
}

u8 BYTE0(temp)
{
	return (temp&0x00ff);
}

vs16 HexToDem(vs16 Hex_Dat)
{
	vs16 Dem_Dat;
	Dem_Dat =Hex_Dat%16 + (Hex_Dat>>4)%16*10 + (Hex_Dat>>8)%16*100 + (Hex_Dat>>12)%16*1000;
	return Dem_Dat;
}
void Bluth_UART_Init()
{

	UCA0CTL1 |= UCSWRST;    //��λ״̬
	UCA0CTL0 = 0;          //��ͨ�첽����ģʽ
	UCA0CTL1 |= UCSSEL_1;    //ʱ��ѡ��UCSSEL_1ΪASCLK 25MHZ              //UCSSEL_3 SMCLK 156250HZ
    P3SEL |= BIT3 | BIT4;  //����Ϊ�ڶ����ܿ�
	//����������
	UCA0BR0 = 13;           //UCOS16 = 0  N = INT(32768/9600) UCOS16 = 1 N = INT(25000000/115200 /16)
	UCA0BR1 = 0;
	UCA0MCTL |= UCBRF_9 | UCOS16;    // UCOS16 = 0 round(32768/9600 - N��UCOS16 =1 ROUND(25000000/115200 - INT(25000000/115200))
	UCA0CTL1 &=~ UCSWRST;  //����״̬

	UCA0IE |= UCRXIE;    //�򿪽������ж�
//	UCA0IE |= UCTXIE;    //�򿪷������ж�
	_EINT();     //�����ж�
}

//Send Dat to DatBuf
//Send_dat is the dat
//num is the lenth of dat
void Uart0_Put_Buf(u8 *Send_dat,u8 num)
{
	u8 i;
	for(i=0;i<num;i++)
	{
		UCA0TXBUF = Send_dat[i];
		while(!(UCA0IFG&UCTXIFG));
		UCA0IFG &=~ UCTXIFG;
	}

}
void Data_Send_Check(vs16 check)   //����У��
{
	u8 i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;

	data_to_send[5]=(check&0xff00)>>8;
	data_to_send[6]=(check&0x00ff);

	u8 sum = 0;
	for(i=0;i<7;i++)
		sum += data_to_send[i];

	data_to_send[7]=sum;

	Uart0_Put_Buf(data_to_send,8);
}

void Data_Receive_Ctl(u8 *data_buf,u8 num)
{
	if(data_buf[0] == 0xcc && data_buf[1] == 0xaf && data_buf[2] == 0x03)  //ȷ��֡�ֽڵĽ�����ȷ�� ��֤֡ͷ�Լ��������Ƿ���ȷ
	{
		Pwm_Para.CtlThro = HexToDem((vs16)(*(data_buf+4)<<8)|*(data_buf+5));
		Pitch_Moto_Offset = HexToDem((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
		Roll_Moto_Offset = HexToDem((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
//		Pwm_Para.CtlPitch = HexToDem((vs16)(*(data_buf+6)<<8)|*(data_buf+7));
//		Pwm_Para.CtlRoll = HexToDem((vs16)(*(data_buf+8)<<8)|*(data_buf+9));
		Pwm_Para.CtlYall = HexToDem((vs16)(*(data_buf+10)<<8)|*(data_buf+11));
		//����һ���߶�ֵû�и�ֵ
	}
}

void Data_Receive_Anl(u8 *data_buf,u8 num)
{
//	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ

	if(*(data_buf+2)==0X10)								//PID1
	{
		Pid_Para_Roll.In_KP = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
		Pid_Para_Roll.In_KI = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
		Pid_Para_Roll.In_KD = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;  //��ʱֻҪһ·  //���п���ROLL��PID

		Pid_Para_Pitch.In_KP = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
		Pid_Para_Pitch.In_KI = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
		Pid_Para_Pitch.In_KD = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;

		Pid_Para_Yall.In_KP = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
		Pid_Para_Yall.In_KI = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
		Pid_Para_Yall.In_KD = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		Data_Send_Check(sum);
		Pid_Set_OK = 1;  //PID������� ��������ACK
	}
}


//1�����ͻ�����Ϣ����̬������״̬��
void Data_Send_Status(void)
{
	u8 _cnt=0;
		u8 sum = 0;
		u8 i;
		vs16 _temp;
		data_to_send[_cnt++]=0xAA;
		data_to_send[_cnt++]=0xAA;
		data_to_send[_cnt++]=0x01;
		data_to_send[_cnt++]=0;

		_temp = (int)(Att_Angle.roll*100);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		_temp = (int)(Att_Angle.pitch*100);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		_temp = (int)(Att_Angle.yall*100);  //����ط���ʱ�õ���6050��YALLֵ
		//_temp = (int)(Mag_Heading*100);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		//_temp = Alt_CSB;
		//data_to_send[_cnt++]=BYTE1(_temp);
		//data_to_send[_cnt++]=BYTE0(_temp);  //������
		//vs32 _temp2 = Alt;
		//data_to_send[_cnt++]=BYTE3(_temp2);  //��ѹ��
		//data_to_send[_cnt++]=BYTE2(_temp2);
		//data_to_send[_cnt++]=BYTE1(_temp2);
		//data_to_send[_cnt++]=BYTE0(_temp2);

		//if(Rc_C.ARMED==0)		data_to_send[_cnt++]=0xA0;
		//else if(Rc_C.ARMED==1)		data_to_send[_cnt++]=0xA1;

		data_to_send[3] = _cnt-4;


		for(i=0;i<_cnt;i++)
			sum += data_to_send[i];
		data_to_send[_cnt++]=sum;

		Uart0_Put_Buf(data_to_send,_cnt);
}

//��2�����ʹ���������
void Data_Send_Senser(void)
{
	u8 _cnt=0;
	u8 i;
	u8 sum = 0;  //У�������
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Sensor_Dat.Acc_X);  //��8λ
	data_to_send[_cnt++]=BYTE0(Sensor_Dat.Acc_X);  //��8λ
	data_to_send[_cnt++]=BYTE1(Sensor_Dat.Acc_Y);
	data_to_send[_cnt++]=BYTE0(Sensor_Dat.Acc_Y);
	data_to_send[_cnt++]=BYTE1(Sensor_Dat.Acc_Z);
	data_to_send[_cnt++]=BYTE0(Sensor_Dat.Acc_Z);
	data_to_send[_cnt++]=BYTE1(Sensor_Dat.Gyro_X);
	data_to_send[_cnt++]=BYTE0(Sensor_Dat.Gyro_X);
	data_to_send[_cnt++]=BYTE1(Sensor_Dat.Gyro_Y);
	data_to_send[_cnt++]=BYTE0(Sensor_Dat.Gyro_Y);
	data_to_send[_cnt++]=BYTE1(Sensor_Dat.Gyro_Z);
	data_to_send[_cnt++]=BYTE0(Sensor_Dat.Gyro_Z);
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	Uart0_Put_Buf(data_to_send,_cnt);
}



//��4������PWMֵ
void Data_Send_MotoPWM(void)
{
	u8 _cnt=0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Pwm_Para.motor_front_pwm);
	data_to_send[_cnt++]=BYTE0(Pwm_Para.motor_front_pwm);
	data_to_send[_cnt++]=BYTE1(Pwm_Para.motor_back_pwm);
	data_to_send[_cnt++]=BYTE0(Pwm_Para.motor_back_pwm);
	data_to_send[_cnt++]=BYTE1(Pwm_Para.motor_left_pwm);
	data_to_send[_cnt++]=BYTE0(Pwm_Para.motor_left_pwm);
	data_to_send[_cnt++]=BYTE1(Pwm_Para.motor_right_pwm);
	data_to_send[_cnt++]=BYTE0(Pwm_Para.motor_right_pwm);  //�ĸ��������

	data_to_send[3] = _cnt-4;

	u8 sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	Uart0_Put_Buf(data_to_send,_cnt);
}

//��6������PID1
void Data_Send_PID1(void)
{
	u8 _cnt=0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;

	vs16 _temp;
	_temp = Pid_Para_Roll.In_KP*100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Roll.In_KI * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Roll.In_KD * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Pitch.In_KP * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Pitch.In_KI * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Pitch.In_KD * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Yall.In_KP * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Yall.In_KI * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pid_Para_Yall.In_KD * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	u8 sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	Uart0_Put_Buf(data_to_send,_cnt);
}

