#ifndef __I2C_H
#define __I2C_H
//I2C���ģ��ʱ������
extern unsigned char ack;	              //Ӧ���־λ 
unsigned char ISendStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no);
// �����ӵ�ַ�������Ͷ��ֽ����ݺ���
unsigned char IRcvStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no);
//�����ӵ�ַ������ȡ���ֽ����ݺ���
unsigned char ISendChar(unsigned char sla,unsigned char suba,unsigned char s);

#endif
