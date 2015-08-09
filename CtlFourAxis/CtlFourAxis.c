/*
 * CtlFourAxis.c
 *
 *  Created on: 2015-7-22
 *      Author: �����ǿ�
 *
 */
#include<msp430f5529.h>
#include<stdio.h>
#include"int_clk.h"
#include"sto_int.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Display1.h"
#include <math.h>
#include <bluth.h>
#include"ESP6288.h"
#include"i2c.h"
#define DEFAULT_MPU_HZ  (100)
#define uchar unsigned char
#define uint unsigned int
#define STILL 0
#define DELTA_TIME 0.003
//#define Roll_Moto_Offset 55
//#define Roll_Moto_Offset_84 35
//#define Pitch_Moto_Offset 70
uchar Start_Flag = 0;
uchar CpuTimePoll;
uchar Uart_Work_Flag = 0;
uchar Sta_Ctl_Flag = 0;
uchar time = 0;
uchar time1 = 0;   //��λ�����������Ƿ�һֱ����ͨ�š�
uint Pitch_Moto_Offset = 0;
uint Roll_Moto_Offset = 0;
/******************************************����������***************************/
/******************************************����������***************************/
/******************************************����������***************************/


typedef struct
{
	float yall;
	float pitch;
	float roll;    //�����ǵ������Ƕ� ƫ�� ���� ����
}AttAngle;
AttAngle Att_Angle;
typedef struct
{
	short Acc_X;
	short Acc_Y;
	short Acc_Z;
	short Gyro_X;
	short Gyro_Y;
	short Gyro_Z;
}SensorDat;
SensorDat Sensor_Dat;


float w,x,y,z;           //��Ԫ��
float temp1,temp2;
float temp3,temp4,temp5;      //��Ԫ��תŷ���ǹ��̱���
float gyro_roll;


typedef struct
{
	uint CtlThro;	//ӳ������Űٷֱ�
	uint Thro;			//���ž���ռ�ձ�
	int motor_front_pwm,motor_back_pwm,motor_left_pwm,motor_right_pwm; //�ĸ�������������ռ�ձ�
	uchar CtlYall;  //���Ʒ���
	uchar CtlRoll;  //����ROLL��б�Ƕ�
	uchar CtlPitch;	//����Pitch��б�Ƕ�
	int CtlXdirPwm;
	int CtlYdirPwm;
	float XdirPwm;
	float YdirPwm;
	int XdirAngle;
	int YdirAngle;
	int ZdirAngle;
}PwmPara;
PwmPara Pwm_Para = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


typedef struct
{
	int pid_pitch_pwm;
	int pid_roll_pwm;  //PID�����ֵ
	int pid_yall_pwm;
}PidOut;
PidOut Pid_Out ={0,0,0};


typedef struct  //�ɿ�PID�����Լ���ʼ��
{
	float In_KP;
	float In_KI;
	float In_KD;
	float Pos_Max;
	float Neg_Max;
	float Last_output;
	float Pid_LastError;  //Pid��һ����Ŀ��Ĳ�ֵ
	float integrator;  //������
	float derivative;  //΢����
}PidPara;
PidPara Pid_Para_Roll ={0,0,0,300,-300,0,0,0,0};
PidPara Pid_Para_Pitch ={0,0,0,300,-300,0,0,0,0};
PidPara Pid_Para_Yall ={0,0,0,300,-300,0,0,0,0};
//
static signed char gyro_orientation[9] = {-1, 0, 0,0,-1, 0,0, 0, 1};  //#ԭ����ע�Ͳμ�(motion_driver_test.c��79��)
volatile unsigned char new_gyro;  //#ԭΪhal.new_gyro(�μ�motion_driver_test.c��55��)

static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
static inline void run_self_test(void);
//
unsigned long sensor_timestamp;
unsigned char more;
short gyro[3],accel[3],sensors;
long quad[4];

unsigned short dmp_features;
//ESP6288


uchar temp_buf;
uchar DatRx[32] = {0};
uchar Pid_Rx_Flag = 0 ;
uchar Pid_Set_OK = 0;
uchar Ctl_Rx_Flag = 0;
uchar Pid_Rx_Count = 0;
uchar Ctl_Rx_Count = 0;
uchar Dat_num = 0;


/**************************************PWM******************************************/
/**************************************PWM******************************************/
/**************************************PWM******************************************/

void PWM_INIT()
{
	TA0CTL |= TASSEL_1;   //��ACLK
	TA0CCTL1 |= OUTMOD_6; //p1.2PWM��
	TA0CCTL2 |= OUTMOD_6; //p1.3PWM��
	TA0CCTL3 |= OUTMOD_6; //p1.4WM��
	TA0CCTL4 |= OUTMOD_6; //p1.5PWM��     toggle/reset
	TA0CCR0 = 12500;        //����������  ����Ϊ0.5ms SMCLK=25000000hz   2KHZ
	TA0CTL |= MC_1;      //������ģʽ
	P1DIR |= BIT2 + BIT3 + BIT4 + BIT5; //p1.2345���
	P1SEL |= BIT2 + BIT3 + BIT4 + BIT5;  //p1.2345 ��Ϊ�ڶ����ܿ�TA0.1234
}

void PID_ROLL()
{
	float Pid_Output=0;  //PID�����
	float Pid_Error;   //Pid��Ŀ��Ĳ�ֵ
	//�����⻷�Ƕ�Pid���
	Pid_Error = Att_Angle.roll - STILL - Pwm_Para.YdirAngle;  //��ȥƽ��������� �Լ���б���нǶ�

	//����P��
	Pid_Output += Pid_Para_Roll.In_KP*Pid_Error;

	//����D��
	Pid_Para_Roll.derivative = Sensor_Dat.Gyro_X/10;                 //(Pid_Error - Pid_LastError) / DELTA_TIME;  //��T�����ж�ʱ��
	Pid_Output += Pid_Para_Roll.In_KD*Pid_Para_Roll.derivative;
	Pid_Para_Roll.Pid_LastError = Pid_Error;

	//����I��
	Pid_Para_Roll.integrator += Pid_Para_Roll.In_KI*Pid_Error*DELTA_TIME;

	if(Pid_Para_Roll.integrator > Pid_Para_Roll.Pos_Max)
		Pid_Para_Roll.integrator = Pid_Para_Roll.Pos_Max;
	else if(Pid_Para_Roll.integrator < Pid_Para_Roll.Neg_Max)
		Pid_Para_Roll.integrator = Pid_Para_Roll.Neg_Max;       //�������޷�

	Pid_Output +=  Pid_Para_Roll.integrator;

	Pid_Out.pid_roll_pwm = Pid_Output;


}


void PID_PITCH()
{
	float Pid_Output = 0;
	float Pid_Error;
	Pid_Error = Att_Angle.pitch - STILL - Pwm_Para.XdirAngle;  //��ȥƽ��������� �Լ���б���нǶ�

	//����P��
	Pid_Output += Pid_Para_Pitch.In_KP*Pid_Error;

	//����D��
	Pid_Para_Pitch.derivative = -Sensor_Dat.Gyro_Y/10;			//(Pid_Error - Pid_LastError) / DELTA_TIME;  //��T�����ж�ʱ��
	Pid_Output += Pid_Para_Pitch.In_KD*Pid_Para_Pitch.derivative;
	Pid_Para_Pitch.Pid_LastError = Pid_Error;

	//����I��
	Pid_Para_Pitch.integrator += Pid_Para_Pitch.In_KI*Pid_Error*DELTA_TIME;

	if(Pid_Para_Pitch.integrator > Pid_Para_Pitch.Pos_Max)
		Pid_Para_Pitch.integrator = Pid_Para_Pitch.Pos_Max;
	else if(Pid_Para_Pitch.integrator < Pid_Para_Pitch.Neg_Max)
		Pid_Para_Pitch.integrator = Pid_Para_Pitch.Neg_Max;       //�������޷�

	Pid_Output +=  Pid_Para_Pitch.integrator;

	Pid_Out.pid_pitch_pwm = Pid_Output;

}


void PID_YALL()
{
	float Pid_Output = 0;
	float Pid_Error;
	Pid_Error = Att_Angle.yall - 45 - Pwm_Para.ZdirAngle;  //��ȥƽ��������� �Լ���б���нǶ�
//	if(abs(Pid_Error) > 180)  //ȷ���Ƿ�Ϊ������ת����
//	{
//		if(Pid_Error < 0)
//		{
//			Pid_Error = abs(Pid_Error)%360;
//		}
//		else
//		{
//			Pid_Error = -abs(Pid_Error)%360;
//		}
//	}

	//����P��
	Pid_Output += Pid_Para_Yall.In_KP*Pid_Error;

	//����D��
	Pid_Para_Yall.derivative = -Sensor_Dat.Gyro_Z/10;			//(Pid_Error - Pid_LastError) / DELTA_TIME;  //��T�����ж�ʱ��
	Pid_Output += Pid_Para_Yall.In_KD*Pid_Para_Yall.derivative;
	Pid_Para_Yall.Pid_LastError = Pid_Error;

	//����I��
	Pid_Para_Yall.integrator += Pid_Para_Yall.In_KI*Pid_Error*DELTA_TIME;

	if(Pid_Para_Yall.integrator > Pid_Para_Yall.Pos_Max)
		Pid_Para_Yall.integrator = Pid_Para_Yall.Pos_Max;
	else if(Pid_Para_Yall.integrator < Pid_Para_Yall.Neg_Max)
		Pid_Para_Yall.integrator = Pid_Para_Yall.Neg_Max;       //�������޷�

	Pid_Output +=  Pid_Para_Yall.integrator;

	Pid_Out.pid_yall_pwm = Pid_Output;

}

void PWM_CALCULATE()
{

	Pwm_Para.Thro = Pwm_Para.CtlThro;    //����PWM��ӳ��         PWM��ӳ�䵽0-1000  15625�����PWM�ļ�������

	Pwm_Para.motor_front_pwm = Pwm_Para.Thro - Pid_Out.pid_pitch_pwm - Pid_Out.pid_yall_pwm;
	Pwm_Para.motor_back_pwm = Pwm_Para.Thro + Pid_Out.pid_pitch_pwm - Pid_Out.pid_yall_pwm + Pitch_Moto_Offset;
	Pwm_Para.motor_left_pwm =Pwm_Para.Thro - Pid_Out.pid_roll_pwm + Pid_Out.pid_yall_pwm + Roll_Moto_Offset;
	Pwm_Para.motor_right_pwm=Pwm_Para.Thro + Pid_Out.pid_roll_pwm + Pid_Out.pid_yall_pwm;            //���ƫ�� ÿ�ܷɻ����Լ���

	if(Pwm_Para.motor_front_pwm > 1000)
		Pwm_Para.motor_front_pwm = 1000;
	else if(Pwm_Para.motor_front_pwm < 0)
		Pwm_Para.motor_front_pwm = 0;

	if(Pwm_Para.motor_back_pwm > 1000)
		Pwm_Para.motor_back_pwm = 1000;
	else if(Pwm_Para.motor_back_pwm < 0)
		Pwm_Para.motor_back_pwm = 0;

	if(Pwm_Para.motor_left_pwm > 1000)
		Pwm_Para.motor_left_pwm = 1000;
	else if(Pwm_Para.motor_left_pwm < 0)
		Pwm_Para.motor_left_pwm = 0;

	if(Pwm_Para.motor_right_pwm > 1000)
		Pwm_Para.motor_right_pwm = 1000;
	else if(Pwm_Para.motor_right_pwm < 0)
		Pwm_Para.motor_right_pwm = 0;            //���ܳ������ռ�ձ�



	if(!Sta_Ctl_Flag)  //����ղ���������Ϣ��ͣ��
	{
		Pwm_Para.motor_front_pwm =0;
		Pwm_Para.motor_back_pwm =0;
		Pwm_Para.motor_left_pwm =0;
		Pwm_Para.motor_right_pwm =0;
	}
}

void PWM_SET()
{
		TA0CCR1 = Pwm_Para.motor_front_pwm*12.5;//motor_front_pwm;  3125ΪTCCR0��ʮ��֮һ
		TA0CCR2 = Pwm_Para.motor_back_pwm*12.5;//motor_left_pwm;
		TA0CCR3 = Pwm_Para.motor_left_pwm*12.5;//motor_right_pwm;
		TA0CCR4 = Pwm_Para.motor_right_pwm*12.5;//motor_back_pwm;
}

/**********************************MPU6050*************************************/
void MPU6050_INIT()
{
	mpu_init();
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);//#(INV_XYZ_GYRO | INV_XYZ_ACCEL)��inv_mpu.h����
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);//#���ļ���ͷ����
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//#���Ķ���
	dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
	DMP_FEATURE_SEND_CAL_GYRO |DMP_FEATURE_GYRO_CAL;//#ȫ����(inv_mpu_dmp_motion_driver.h)�ж���
	dmp_enable_feature(dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	run_self_test();
	mpu_set_dmp_state(1);
}

/******************************************���Ƕȼ���*******************************/
/******************************************���Ƕȼ���*******************************/
/******************************************���Ƕȼ���*******************************/
void ANGLE_CALCULATE()
{
    w = (float)quad[0]/1073741824.0f ;
    x = (float)quad[1]/1073741824.0f ;
    y = (float)quad[2]/1073741824.0f ;
    z = (float)quad[3]/1073741824.0f ;
    temp1 = 2*y*x + 2*w*z;
    temp2 = w*w + x*x - y*y -z*z;
    Att_Angle.yall = 57.3*atan2(temp1,temp2); //ƫ��

    temp3 = 2*x*z - 2*w*y;
    Att_Angle.pitch = 57.3*asin(temp3);   //����

    temp4 = 2*y*z + 2*w*x;
    temp5 = -2*x*x - 2*y*y + 1;
    Att_Angle.roll = 57.3*atan2(temp4,temp5);  //����

    Sensor_Dat.Acc_X = accel[0];  //����100��
    Sensor_Dat.Acc_Y = accel[1];
    Sensor_Dat.Acc_Z = accel[2];

    Sensor_Dat.Gyro_X = gyro[0];
    Sensor_Dat.Gyro_Y = gyro[1];
    Sensor_Dat.Gyro_Z = gyro[2];

}

/********************************************************************************/

/*********************************************TIME1�жϼ�ʱ*************************/
void TIMEA1_INIT()
{

	TA1CTL |= TASSEL_2;  //��SCLK 1562500hz
	TA1CCTL1 |= CCIE;    //TA1CCR1�жϴ�
	TA1CCR0 = 1562500/300;     //����������    3ms��ʱ  1562500/333
	TA1CCR1 = 2000;      //��������С��15625/2����
	TA1CTL |= MC_1;      //������ģʽ ���ж�
	_EINT();
}

void TIMEA2_INIT()
{

	TA2CTL |= TASSEL_1;  //��ACLK 25000000
	TA2CCR0 = 0;     //
	TA2CTL |= MC_0;      //�رռ���
}

void HC_RS04_Init()
{
	P2DIR |= BIT0;       //trig ���Ͷ˿�
	P2OUT &=~ BIT0;      //���Ͷ˿����͵ȴ��ߵ�ƽ

	P2DIR &=~ BIT2;     //ECHO  ���ն˿�
	P2IFG = 0x00;      //���p2�����жϱ�־λ
	P2IES |= BIT2;     //P2.2�����ش���
	P2IE |= BIT2;       //P2.2�жϴ�
	P2IN &=~ BIT2;     //�������ͣ��Ȼ��ⲿ�ߵ�ƽ�������߲���������
	P2OUT |= BIT2;
	P2REN |= BIT2;     //�����������������������������
	_EINT();
}
void HC_RS04_Send()
{
	P2OUT |= BIT0;  //TRIG ���͸ߵ�ƽ
	delay_us(10);
	P2OUT &=~ BIT0;
	TA2CTL |= MC_2;      //��������ģʽ CCR0������0XFFFF Ȼ��λ
}
/*
CPU���� NMSʱ���� ���п��ƣ�����ʱ��Ϊ���ڵ��жϺͷ��͡�
*/

void main(void)
{
	int_clk();         //ʱ�ӳ�ʼ��
	P2DIR|=BIT4+BIT5;  //������I2C�˿ڳ�ʼ��
	MPU6050_INIT();
	Bluth_UART_Init();	//��������
	while(!Pid_Set_OK);//�ȴ�����PID

	PWM_INIT();    //��·PWM��ʼ��
	TIMEA1_INIT(); //PID����5MS�жϳ�ʼ��  �жϳ�ʼ�������� ESPģ����� ��Ϊ������Ƕ���˶Դ����жϵ�ʹ��
//	HC_RS04_Init();
//	TIMEA2_INIT();
	while(1)
	{
//		HC_RS04_Send();
//		delay_ms(100);
			dmp_read_fifo(gyro, accel, quad, &sensor_timestamp, &sensors, &more);  //��ȡ���ٶȽ��ٶ���Ԫ��  �������Է��ָú���ִ�е�ʱ�侹Ȼ�ǲ�ȷ���ģ����ʺϷ����жϣ�
	}
}
#pragma vector = PORT2_VECTOR     //�̶���ʽ�������ж�������ַ
__interrupt void HC_SR06_ISR(void)
{
	P2IFG = 0x00;
	TA2CTL |= MC_0;      //��������ģʽ CCR0������0XFFFF Ȼ��λ

}
#pragma vector = TIMER1_A1_VECTOR  //��ʱ���ж�3ms��ʱ  ��Ϊʱ����poll���й���
__interrupt void PID_CONTROL()
{


	TA1CCTL1 &=~ CCIFG;
	switch(CpuTimePoll)
	{
	//��һ��3ms������  ÿ6ms����һ��
	case 0 :
		ANGLE_CALCULATE(); //��Ԫ��תŷ���� ���ִ����1.2MS����
		PID_ROLL();           //��Ӽ���PIDռ�ձ�    PIDʱ�� ����
		PID_PITCH();
		PID_YALL();
		PWM_CALCULATE();  //PID���ֵת��Ϊ��·�����PWMռ�ձ�
		PWM_SET();         //PWMռ�ձ�����
		//����ִ��2.2ms
		break;

	//�ڶ���3ms�����տ�����Ϣ  ��������Ϊ15ms���а���һ��3ms�Ľ���
	case 1 :
		if(time1 % 4 == 0)
		{
			Data_Send_PID1();
		}
		else if(time1 % 4 == 1)
		{
			Data_Send_MotoPWM(); //���õ��
		}
		else if(time1 % 4 == 2)
		{
			Data_Send_Status();
		}
		else if(time1 % 4 == 3)
		{
			Data_Send_Senser();
		}
		if(time1++ == 3)
			time1 = 0;
		break;
	}

	if(CpuTimePoll++ == 1)
		CpuTimePoll = 0;
}



#pragma vector = USCI_A0_VECTOR  //�������ͽ����жϣ�UART0��
__interrupt void Dat_SendReceive()
{

	temp_buf = UCA0RXBUF;  //��BUF�������IFG Ҫ�������0
	if(temp_buf == 0xaa)  //�ж�֡ͷ  PID����֡
	{
		Pid_Rx_Flag = 1;
	}
	if(Pid_Rx_Flag)
	{
		if(Pid_Rx_Count == 3)  //���ĸ����������ݵĳ���
		{
			Dat_num = temp_buf;
		}
		DatRx[Pid_Rx_Count++] = temp_buf;
		if(Pid_Rx_Count == Dat_num + 4 + 1) //��������λ֡ͷ Lenλ���� 1λSum
		{
			Data_Receive_Anl(DatRx,Pid_Rx_Count);  //���������Լ������ܳ���
			Pid_Rx_Count = 0;
			Pid_Rx_Flag = 0;
		}
	}


	if(temp_buf == 0xcc)  //����֡
	{
		Ctl_Rx_Flag = 1;
		TA1CCTL1 &=~ CCIE;
//		Uart_Work_Flag = 1;      //���ߵ�Ƭ�����Ծ��������������ܽ��յ�����֡���յ���λ���Ŀ�����Ϣ
	}
	if(Ctl_Rx_Flag)
	{
		DatRx[Ctl_Rx_Count++] = temp_buf;
		if(Ctl_Rx_Count == 14)
		{
			TA1CCTL1 |= CCIE;
			Data_Receive_Ctl(DatRx,14);
			Sta_Ctl_Flag = DatRx[13];
			Ctl_Rx_Count = 0;
			Ctl_Rx_Flag = 0;
		}
	}
	UCA0IFG &=~ UCRXIFG;  //���BUF

}


/*******************************

*******************************/
/* These next two functions converts the orienta
 * tion matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

/*******************************

*******************************/
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;




    return scalar;
}

/*******************************

*******************************/
static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }


    /* Report results. */  //#������ ��ΪN5110
    //test_packet[0] = 't';
    //test_packet[1] = result;
//    N5110_Set_XY(0,0);//#
//    N5110_Write_Char('T');
//    N5110_Write_Char(':');
//    N5110_Write_Char(result+48);
    //send_packet(PACKET_TYPE_MISC, test_packet);
}

/*******************************

*******************************/




