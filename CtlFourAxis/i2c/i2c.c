/*-----------------------------------------------
  ���ƣ�IICЭ�� EEPROM24c02
  ���ݣ��˳������ڼ��EEPROM���ܣ����Է������£�д��24c02һЩ���ݣ�Ȼ�����ڴ��������Щ���ݣ�
        ��������ڴ潫ʧȥ��Щ��Ϣ��Ȼ���24c02�е�����Щ���ݡ����Ƿ���д�����ͬ��		
------------------------------------------------*/  
  
              
#include<msp430f5529.h>
#include"int_clk.h"                       //��ȷ��ʱ������ʱ�ӳ�ʼ��
#include"msp430_clock.h"


// ��,����������
#define DATDIR P2DIR               //I2C�˿ڷ���Ĵ���
#define DATIN P2IN             //����Ĵ���
#define DATOUT P2OUT           //����Ĵ���
#define SDA BIT4            //ģ��I2C���ݴ���λ
#define SCL BIT5            //ģ��I2Cʱ�ӿ���λ
                          
unsigned char ack;	              //Ӧ���־λ   
/*------------------------------------------------
                    ��������
------------------------------------------------*/
void Start_I2c()
{
  DATDIR|=SDA;      //�����
  DATDIR|=SCL;
  DATOUT|=SDA;//SDA=1;   //������ʼ�����������ź�
  delay_us(1);
  DATOUT|=SCL;//SCL=1;
  delay_us(5);    //��ʼ��������ʱ�����4.7us,��ʱ    
  DATOUT&=~SDA;//SDA=0;     //������ʼ�ź�
  delay_us(5);    //��ʼ��������ʱ�����4��       
  DATOUT&=~SCL;//SCL=0;    //ǯסI2C���ߣ�׼�����ͻ��������
  delay_us(2);
}
/*------------------------------------------------
                    ��������
------------------------------------------------*/
void Stop_I2c()
{
  DATOUT&=~SDA;//SDA=0;    //���ͽ��������������ź�
  delay_us(1);   //���ͽ���������ʱ���ź�
  DATOUT|=SCL;//SCL=1;    //������������ʱ�����4��
  delay_us(5);
  DATOUT|=SDA;//SDA=1;    //����I2C���߽����ź�
  delay_us(4);
}




/*----------------------------------------------------------------
                 �ֽ����ݴ��ͺ���               
����ԭ��: void  SendByte(unsigned char c);
����:  ������c���ͳ�ȥ,�����ǵ�ַ,Ҳ����������,�����ȴ�Ӧ��,����
     ��״̬λ���в���.(��Ӧ����Ӧ��ʹack=0 ��)     
     ��������������ack=1; ack=0��ʾ��������Ӧ����𻵡�
------------------------------------------------------------------*/
void  SendByte(unsigned char c)
{
 unsigned char BitCnt;
 
 for(BitCnt=0;BitCnt<8;BitCnt++)  //Ҫ���͵����ݳ���Ϊ8λ
    {
     if((c<<BitCnt)&0x80)
       DATOUT|=SDA;//SDA=1;   //�жϷ���λ
     else  DATOUT&=~SDA;//SDA=0;                
     delay_us(1);
     DATOUT|=SCL;//SCL=1;               //��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ      
     delay_us(5);             //��֤ʱ�Ӹߵ�ƽ���ڴ���4��         
     DATOUT&=~SCL;//SCL=0; 
    }
    delay_us(2);
    DATOUT|=SDA;//SDA=1;               //8λ��������ͷ������ߣ�׼������Ӧ��λ      
    DATDIR&=~SDA;
    delay_us(2);
    DATOUT|=SCL;//SCL=1;
    delay_us(3);
    if(DATIN&SDA)ack=0;     
    else ack=1;        //�ж��Ƿ���յ�Ӧ���ź�
    DATOUT&=~SCL;//SCL=0;
    delay_us(2);
    DATDIR|=SDA;        //����Ĵ������������״̬
}







/*----------------------------------------------------------------
                 �ֽ����ݴ��ͺ���               
����ԭ��: unsigned char  RcvByte();
����:  �������մ���������������,���ж����ߴ���(����Ӧ���ź�)��
     ���������Ӧ������  
------------------------------------------------------------------*/	
unsigned char  RcvByte()
{
  unsigned char retc;
  unsigned char BitCnt;
  
  retc=0; 
  DATDIR&=~SDA;//SDA=1;             //��������Ϊ���뷽ʽ
  for(BitCnt=0;BitCnt<8;BitCnt++)
      {
        delay_us(1);           
        DATOUT&=~SCL;//SCL=0;       //��ʱ����Ϊ�ͣ�׼����������λ


        delay_us(5);      //ʱ�ӵ͵�ƽ���ڴ���4.7us
        DATOUT|=SCL;//SCL=1;       //��ʱ����Ϊ��ʹ��������������Ч
        delay_us(2);
        retc=retc<<1;
        if(DATIN&SDA)retc=retc+1; //������λ,���յ�����λ����retc��
        delay_us(2);
      }
  DATOUT&=~SCL;//SCL=0;    
  delay_us(2);
  DATDIR|=SDA;
  return(retc);
}



/*----------------------------------------------------------------
                     Ӧ���Ӻ���
ԭ��:  void Ack_I2c(void);
 
----------------------------------------------------------------*/
void Ack_I2c(void)
{
  
  DATOUT&=~SDA;//SDA=0;     
  delay_us(3);     
  DATOUT|=SCL;//SCL=1;
  delay_us(5);              //ʱ�ӵ͵�ƽ���ڴ���4��
  DATOUT&=~SCL;//SCL=0;               //��ʱ���ߣ�ǯסI2C�����Ա��������
  delay_us(2);  
}
/*----------------------------------------------------------------
                     ��Ӧ���Ӻ���
ԭ��:  void NoAck_I2c(void);
 
----------------------------------------------------------------*/
void NoAck_I2c(void)
{
  
  DATOUT|=SDA;//SDA=1;
  delay_us(3);      
  DATOUT|=SCL;//SCL=1;
  delay_us(5);              //ʱ�ӵ͵�ƽ���ڴ���4��
  DATOUT&=~SCL;//SCL=0;                //��ʱ���ߣ�ǯסI2C�����Ա��������
  delay_us(2);  
}
unsigned char ISendChar(unsigned char sla,unsigned char suba,unsigned char s)
{
   unsigned char i;
   Start_I2c();               //��������
   SendByte(sla);             //����������ַ
     if(ack==0)return(0);
   SendByte(suba);            //���������ӵ�ַ
     if(ack==0)return(0);
   SendByte(s);            //��������
     if(ack==0)return(0);
   Stop_I2c();                  //��������
   delay_us(10);
   return(1);
}

/*----------------------------------------------------------------
                    �����ӵ�ַ�������Ͷ��ֽ����ݺ���               
����ԭ��: bit  ISendStr(unsigned char sla,unsigned char suba,ucahr *s,unsigned char no);  
����:     ���������ߵ����͵�ַ���ӵ�ַ,���ݣ��������ߵ�ȫ����,������
          ��ַsla���ӵ�ַsuba������������sָ������ݣ�����no���ֽڡ�
           �������1��ʾ�����ɹ��������������
ע�⣺    ʹ��ǰ�����ѽ������ߡ�
----------------------------------------------------------------*/
unsigned char ISendStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no)
{
   unsigned char i;

   Start_I2c();               //��������
   SendByte(sla);             //����������ַ
     if(ack==0)return(0);
   SendByte(suba);            //���������ӵ�ַ
     if(ack==0)return(0);
   for(i=0;i<no;i++)
    {   
     SendByte(*s);            //��������
     if(ack==0)return(0);
     s++;
    } 
    Stop_I2c();                  //��������
    delay_us(10);
    return(1);
}

/*----------------------------------------------------------------
                    �����ӵ�ַ������ȡ���ֽ����ݺ���               
����ԭ��: bit  ISendStr(unsigned char sla,unsigned char suba,ucahr *s,unsigned char no);  
����:     ���������ߵ����͵�ַ���ӵ�ַ,�����ݣ��������ߵ�ȫ����,������
          ��ַsla���ӵ�ַsuba�����������ݷ���sָ��Ĵ洢������no���ֽڡ�
           �������1��ʾ�����ɹ��������������
ע�⣺    ʹ��ǰ�����ѽ������ߡ�
----------------------------------------------------------------*/
unsigned char IRcvStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no)
{
   unsigned char i;

   Start_I2c();               //��������
   SendByte(sla);             //����������ַ
     if(ack==0)return(0);
   SendByte(suba);            //���������ӵ�ַ
     if(ack==0)return(0);

   Start_I2c();
   SendByte(sla+1);
      if(ack==0)return(0);

   for(i=0;i<no-1;i++)
    {   
     *s=RcvByte();              //��������
      Ack_I2c();                //���;ʹ�λ 
     s++;
    } 
   *s=RcvByte();
    NoAck_I2c();                 //���ͷ�Ӧλ
   Stop_I2c();                    //��������
  return(1);
}
