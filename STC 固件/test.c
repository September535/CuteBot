#include "STC8xxxx.H"
#include "intrins.h"

#define PWM5_2      0x01	//P1.7
#define PWM6_2      0x04	//P5.4
#define ENO5P       0x01
#define ENO6P       0x04
#define ENO7P       0x10
#define ENO8P       0x40
#define PWM_PERIOD  1000

unsigned int PWM_Period = 50;  // PWM���ڣ���λΪ��ʱ������ֵ

// PWM���Ŷ���
sbit PWM_A1 = P1^7; // ���A��ת
sbit PWM_A2 = P5^4; // ���A��ת
sbit PWM_B2 = P3^4; // ���B��ת
sbit PWM_B1 = P3^3; // ���B��ת

sbit L_RGB_r = P3^2;
sbit L_RGB_g = P3^5;
sbit L_RGB_b = P3^6;

sbit R_RGB_r = P1^6;
sbit R_RGB_g = P1^2;
sbit R_RGB_b = P1^0;

// ״̬��־λ
bit isda; // �豸��ַ��־
bit isma; // �洢��ַ��־
unsigned char addr; // �洢���յ����ڴ��ַ
unsigned char buffer[8]; // �洢���յ�������
unsigned char pwm_counter = 0; // PWM������
unsigned char count = 0; // PWM������

// PWMռ�ձȵĿ��Ʊ��� (0-255)
unsigned char L_R_pwm = 0;  // ����ɫ����
unsigned char L_G_pwm = 0;  // �����ɫ����
unsigned char L_B_pwm = 0;  // �����ɫ����

unsigned char R_R_pwm = 0;  // �Ҳ��ɫ����
unsigned char R_G_pwm = 0;  // �Ҳ���ɫ����
unsigned char R_B_pwm = 0;  // �Ҳ���ɫ����

unsigned char index;

typedef struct {
    int speed;      // ��ǰ�ٶȣ�0-100��
    int direction;  // ����1����ת����2����ת����0��ֹͣ��
    int brake;      // ɲ����־��1=ɲ����0=����
} Motor;

Motor leftMotor = {0, 0};  // ���ֵ��
Motor rightMotor = {0, 0}; // ���ֵ��
void Delay10ms(void)	//@11.0592MHz
{
	unsigned char data i, j;

	_nop_();
	_nop_();
	i = 144;
	j = 157;
	do
	{
		while (--j);
	} while (--i);
}


void motor_init()
{
	
	
	
	
	PWMB_CCER1_Disable();
	PWMB_CCER2_Disable();
	
	PWMB_PSCR = 0x0000;	// Ԥ��Ƶ�Ĵ���
	PWMB_DTR  = 0;		// ����ʱ������
	PWMB_ARR = 100;	// �Զ���װ�ؼĴ���,  ����PWM����
	PWMB_CCER1  = 0;
	PWMB_CCER2  = 0;
	PWMB_SR1    = 0;
	PWMB_SR2    = 0;
	PWMB_ENO    = 0;		// IO�������
	PWMB_PS     = 0;
	PWMB_IER    = 0;

	PWMB_CCMR1 = 0x68; //ͨ��ģʽ����
	PWMB_CCMR2 = 0x68;
	PWMB_CCMR3 = 0x68;
	PWMB_CCMR4 = 0x68;
	
	
	PWMB_CC5E_Enable();
	PWMB_CC6E_Enable();
	PWMB_CC7E_Enable();
	PWMB_CC8E_Enable();

 
	PWMB_AutoReload(PWM_PERIOD);
PWMB_ENO =0x00;
	PWM5P_OUT_EN();
	PWM6P_OUT_EN();
	PWM7P_OUT_EN();
	PWM8P_OUT_EN();


	PWM5_USE_P17();
	PWM6_USE_P54();
	PWM7_USE_P33();
	PWM8_USE_P34();
	PWMB_BrakeOutputEnable();   //ʹ�������
	PWMB_CEN_Enable();
	
	PWMB_Duty5(0);
	PWMB_Duty6(0);
	PWMB_Duty7(0);
	PWMB_Duty8(0);
}

void initMotor(Motor *motor) {
    motor->speed = 0;
    motor->direction = 0;
}

// ��ʱ��0��ʼ������
void Timer0_Init() {
    TMOD |= 0x01;  // ���ö�ʱ��0Ϊģʽ1��16λ��ʱ����
    TH0 = (65536 - PWM_Period) / 256;  // ��ʼ����ʱ��0��8λ
    TL0 = (65536 - PWM_Period) % 256;  // ��ʼ����ʱ��0��8λ
    ET0 = 1;  // ʹ�ܶ�ʱ��0�ж�
    EA = 1;   // ȫ���ж�ʹ��
    TR0 = 1;  // ������ʱ��0
}


// ��ʱ��0�жϷ�����
void Timer0_ISR() interrupt 1 {
    static unsigned int count = 0;
    TH0 = (65536 - PWM_Period) / 256;  // ���¼��ض�ʱ��0��8λ
    TL0 = (65536 - PWM_Period) % 256;  // ���¼��ض�ʱ��0��8λ
    count++;
	
		
		

		L_RGB_r = (count < L_R_pwm) ? 0 : 1;
	if(L_G_pwm == 0) L_RGB_g = 1;
		
   else L_RGB_g = (count < L_G_pwm) ? 0 : 1;
    L_RGB_b = (count < L_B_pwm) ? 0 : 1;
    
    R_RGB_r = (count < R_R_pwm) ? 0 : 1;
	if(R_G_pwm == 0) R_RGB_g = 1;
   else R_RGB_g = (count < R_G_pwm) ? 0 : 1;
    R_RGB_b = (count < R_B_pwm) ? 0 : 1;
	
		

		if (count >= PWM_Period) {
				count = 0;  // ����������
		}
}


// ����M2������ٶ�


void setRMotor(int direction, int speed) {
	
    // ������0-100ӳ�䵽70-100
    int mapped_speed = 70 + (speed * 30 / 100);
    
    // ȷ��ӳ�����ٶ���70-100��Χ��
    if (mapped_speed < 70) mapped_speed = 70;
    if (mapped_speed > 100) mapped_speed = 100;


    if (direction == 1) { // ��ת
			
			PWMB_CCR7 = 0;
			PWMB_CCR8 = mapped_speed*8;
			
    } else if (direction == 2) { // ��ת
			
			PWMB_CCR8 =0;
			PWMB_CCR7 = mapped_speed*8;

    } else { // ֹͣ
        PWMB_CCR7 = 0;
			PWMB_CCR8 = 0;
    }
}

// ����M1������ٶ�

int pwm_value = 0;

void setLMotor(int direction, int speed) {
    // ������0-100ӳ�䵽70-100
    int mapped_speed = 70 + (speed * 30 / 100);
    
    // ȷ��ӳ�����ٶ���70-100��Χ��
    if (mapped_speed < 70) mapped_speed = 70;
    if (mapped_speed > 100) mapped_speed = 100;
    
    // ת��ΪPWMֵ������PWM��Χ0-800��
     pwm_value = mapped_speed * 8;

    if (direction == 1) { // ��ת
        PWMB_CCR6 = pwm_value;
        PWMB_CCR5 = 0;
    } 
    else if (direction == 2) { // ��ת
        PWMB_CCR6 = 0;
        PWMB_CCR5 = pwm_value;
    } 
    else { // ֹͣ
        PWMB_CCR6 = 0;
        PWMB_CCR5 = 0;
    }
}

// �������RGB������ֵ
void Set_Left_RGB_Brightness(unsigned char red, unsigned char green, unsigned char blue) {
    L_R_pwm = red/5;
    L_G_pwm = green/5;
    L_B_pwm = blue/5;
	
	  if(red <= 10)L_RGB_r = 1;
	  if(green <= 10)L_RGB_g = 1;
	  if(blue <= 10)L_RGB_b = 1;
	
}

// �����Ҳ�RGB������ֵ
void Set_Right_RGB_Brightness(unsigned char red, unsigned char green, unsigned char blue) {
    R_R_pwm = red/5;
    R_G_pwm = green/5;
    R_B_pwm = blue/5;
	
	  if(red <= 10)R_RGB_r = 1;
	
	  if(green <= 10)R_RGB_g = 1;
	
	  if(blue <= 10)R_RGB_b = 1;

	
}

void I2C_Isr() interrupt 24 {
    _push_(P_SW2);
    P_SW2 |= 0x80;

    if (I2CSLST & 0x40) {
        I2CSLST &= ~0x40; // ����START�¼�
    } else if (I2CSLST & 0x20) {
        I2CSLST &= ~0x20; // ����RECV�¼�
        if (isda) {
            isda = 0; // ����RECV�¼���RECV DEVICE ADDR��
        } else if (isma) {
            isma = 0; // ����RECV�¼���RECV MEMORY ADDR��
            addr = I2CRXD;
            I2CTXD = buffer[addr];
        } else {
            buffer[addr++] = I2CRXD; // ����RECV�¼���RECV DATA��
            if (addr >= 8) addr = 0;
        }
    } else if (I2CSLST & 0x10) {
        I2CSLST &= ~0x10; // ����SEND�¼�
        if (I2CSLST & 0x02) {
            I2CTXD = 0xff; // ���յ�NAK��ֹͣ��ȡ����
        } else {
            I2CTXD = buffer[++addr]; // ���յ�ACK�������ȡ����
        }
    } else if (I2CSLST & 0x08) {
        I2CSLST &= ~0x08; // ����STOP�¼�
        isda = 1;
        isma = 1;
    }

    _pop_(P_SW2);
}

int pwm_out1=50;
int pwm_out2=100;	
int pwm_out3=150;
int pwm_out4=200;
int pwm_out5=0;
int pwm_out6=50;
int pwm_out7=0;
int pwm_out8=10;
 
void PWM_Init()
{
	PWMB_PSCR = 0x0000;	// Ԥ��Ƶ�Ĵ���
	PWMB_DTR  = 0;		// ����ʱ������
	PWMB_ARR = 1000;	// �Զ���װ�ؼĴ���,  ����PWM����
	PWMB_CCER1  = 0;
	PWMB_CCER2  = 0;
	PWMB_SR1    = 0;
	PWMB_SR2    = 0;
	PWMB_ENO    = 0;		// IO�������
	PWMB_PS     = 0;
	PWMB_IER    = 0;
	
 
    /* PWM5 -- P17 */
	PWMB_CCMR1  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMB_CCR5   = 0;	// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMB_CCER1 |= 0x01;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMB_PS    |= 1;		// ѡ��IO
	PWMB_ENO   |= 0x01;		// IO�������
 
    /* PWM6 -- P54 */
	PWMB_CCMR2  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMB_CCR6   = 0;	// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMB_CCER1 |= 0x10;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMB_PS    |= (1<<2);	// ѡ��IO
	PWMB_ENO   |= 0x04;		// IO�������
 
    /* PWM7 -- P33 */
	PWMB_CCMR3  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMB_CCR7   = 0;	// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMB_CCER2 |= 0x01;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMB_PS    |= (1<<4);	// ѡ��IO
	PWMB_ENO   |= 0x10;		// IO�������
 
    /* PWM8 -- P34 */
	PWMB_CCMR4  = 0x68;		// ͨ��ģʽ����, PWMģʽ1, Ԥװ������
	PWMB_CCR8   = 0;		// �Ƚ�ֵ, ����ռ�ձ�(�ߵ�ƽʱ����)
	PWMB_CCER2 |= 0x10;		// �����Ƚ����, �ߵ�ƽ��Ч
	PWMB_PS    |= (1<<6);	// ѡ��IO
	PWMB_ENO   |= 0x40;		// IO�������
 
	PWMB_EGR    = 0x01;		//����һ�θ����¼�
	PWMB_BRK    = 0x80;		// �����ʹ�� �൱���ܿ���
	PWMB_CR1    = 0x81;		// ʹ�ܼ�����, �����Զ���װ�ؼĴ�������
}

void main() {
    
    P1M0 = 0xcf; P1M1 = 0x00;  
    P3M0 = 0xff; P3M1 = 0x00; 
    P5M0 = 0xff; P5M1 = 0x00; 

    P_SW2 = 0x80;

    I2CCFG = 0x81; // ʹ��I2C�ӻ�ģʽ
    I2CSLADR = 0X30; // ���ôӻ��豸��ַ
    I2CSLST = 0x00;
    I2CSLCR = 0x78; // ��ֹ�ӻ�ģʽ�ж�
		

    R_RGB_r = 1; // Ĭ�Ϲر�RGB��
    R_RGB_g = 1;
    R_RGB_b = 1;

    L_RGB_r = 1;
    L_RGB_g = 1;
    L_RGB_b = 1; 
		
    PS = 1;
    
    // ��ʼ����ʱ��0
    Timer0_Init();
    
    EA = 1; // �������ж�

    isda = 1; // �û�������ʼ��
    isma = 1;
    addr = 1;
    I2CTXD = buffer[addr];
		//motor_init();
PWM_Init();

//setLMotor(1, 100);
//setRMotor(1,100);

    while (1) {
			
        switch (buffer[0]) {
            case 1:
                setLMotor(buffer[1], buffer[2]);
                break;
            case 2:
                setRMotor(buffer[1], buffer[2]);
                break;
            case 3:
                Set_Right_RGB_Brightness(buffer[1], buffer[2], buffer[3]);
                break;
            case 4:
								//break;
            case 5:
                Set_Left_RGB_Brightness(buffer[1], buffer[2], buffer[3]);
                if (buffer[0] == 5) {
                   Set_Right_RGB_Brightness(buffer[1], buffer[2], buffer[3]);
                }
                break;
            default:
                break;
        }  
    }
}