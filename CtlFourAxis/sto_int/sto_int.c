/*�������ܣ���ֵ��λ����洢������
����ֵn��λ���ֱַ�ȡ������˳�����������st�У�n���Ϊ����4λ��pstΪ�洢��ʼλ*/
#include<math.h>
void sto_int(int n,unsigned char *st,unsigned char pst)
{
        unsigned int n_int;                   //n_intΪ����ȡ������ʱ��ֵ
        unsigned char i=0,j=0;                   //iΪ�洢�����Ԫ����ţ�jΪ����ȡ��ѭ������������ȡ��Ԫ�����
        int k;                                 //kΪ����ת��ѭ������
        unsigned char symbol_n=0;    //��ֵ���ű��
        unsigned char tp[10]={' '};      //����ȡ����ʱ����
        if(n < 0)                   //��ֵΪ���������1����ֵȡ����ֵ
        	{
        		symbol_n=1;
        		n = -n;
        	}
        n_int=n;
        for(j=0;j<5;j++)                        //����ȡ���������8λ��ȡ���˳�ѭ��
        {
                //if(n_int == 0)break;
                tp[i++] ='0'+n_int%10;
                n_int = n_int/10;
        }
		if(symbol_n==1) tp[i]='-'; //����λΪ1������������λΪ����
		else tp[i]='+';
		//˳��ת��
        for(k=j;k>=0;k--)         //������������ת��
        {
          st[pst++] = tp[k];
        }   
}
