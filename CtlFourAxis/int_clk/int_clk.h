#ifndef __INT_CLK
#define __INT_CLK
#define CPU_F ((double)25000000)
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0))//��ʱx΢��
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))   //��ʱx����
void int_clk();														//ʱ�ӳ�ʼ��
#endif
