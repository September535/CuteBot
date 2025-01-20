#include "reg51.h"
#include "intrins.h"

sfr     P_SW2   =   0xba;

#define I2CCFG      (*(unsigned char volatile xdata *)0xfe80)
#define I2CMSCR     (*(unsigned char volatile xdata *)0xfe81)
#define I2CMSST     (*(unsigned char volatile xdata *)0xfe82)
#define I2CSLCR     (*(unsigned char volatile xdata *)0xfe83)
#define I2CSLST     (*(unsigned char volatile xdata *)0xfe84)
#define I2CSLADR    (*(unsigned char volatile xdata *)0xfe85)
#define I2CTXD      (*(unsigned char volatile xdata *)0xfe86)
#define I2CRXD      (*(unsigned char volatile xdata *)0xfe87)

sfr     P1M1    =   0x91;
sfr     P1M0    =   0x92;
sfr     P0M1    =   0x93;
sfr     P0M0    =   0x94;
sfr     P2M1    =   0x95;
sfr     P2M0    =   0x96;
sfr     P3M1    =   0xb1;
sfr     P3M0    =   0xb2;
sfr     P4M1    =   0xb3;
sfr     P4M0    =   0xb4;
sfr     P5M1    =   0xc9;
sfr     P5M0    =   0xca;

sfr P5   = 0xC8;


// PWM引脚定义
sbit PWM_A1 = P1^7; // 电机A正转
sbit PWM_A2 = P5^4; // 电机A反转
sbit PWM_B2 = P3^4; // 电机B正转
sbit PWM_B1 = P3^3; // 电机B反转

sbit L_RGB_r = P3^2 ;
sbit L_RGB_g = P3^5 ;
sbit L_RGB_b = P3^6 ;

sbit R_RGB_r = P1^6 ;
sbit R_RGB_g = P1^2 ;
sbit R_RGB_b = P1^0 ;


// 状态标志位
bit isda; // 设备地址标志
bit isma; // 存储地址标志
unsigned char addr; // 存储接收到的内存地址
unsigned char buffer[8]; // 存储接收到的数据
int count_buffer,count,PWM;

void Delay1ms(void)	//@22.1184MHz
{
	unsigned char data i, j;

	i = 29;
	j = 183;
	do
	{
		while (--j);
	} while (--i);
}

void delay_ms(unsigned int ms){
	unsigned int x ;
  for(x = 0 ; x<ms ;++x){Delay1ms();}
		
}

void R_RGBColor(unsigned char red, unsigned char green, unsigned char blue) {
    R_RGB_r = (red > 0) ? 0 : 1;   // 红色灯
    R_RGB_g = (green > 0) ? 0 : 1; // 绿色灯
    R_RGB_b = (blue > 0) ? 0 : 1;  // 蓝色灯
}
void L_RGBColor(unsigned char red, unsigned char green, unsigned char blue) {
    L_RGB_r = (red > 0) ? 0 : 1;   // 红色灯
    L_RGB_g = (green > 0) ? 0 : 1; // 绿色灯
    L_RGB_b = (blue > 0) ? 0 : 1;  // 蓝色灯
}

// 控制电机的速度
void setMotorSpeed(int pwmA, int pwmB) {
    if (pwmA == 1) {
        PWM_A1 = 0; // 控制电机A正转
        PWM_A2 = 1;
    } else if (pwmA == 2) {
        PWM_A1 = 1;
        PWM_A2 = 0; // 控制电机A反转
    } else {
        PWM_A1 = 0; // 停止电机A
        PWM_A2 = 0;
    }

    if (pwmB == 1) {
        PWM_B1 = 0; // 控制电机B正转
        PWM_B2 = 1;
    } else if (pwmB == 2) {
        PWM_B1 = 1;
        PWM_B2 = 0; // 控制电机B反转
    } else {
        PWM_B1 = 0; // 停止电机B
        PWM_B2 = 0;
    }
		
}

void I2C_Isr() interrupt 24
{
    _push_(P_SW2);
    P_SW2 |= 0x80;

    if (I2CSLST & 0x40)
    {
        I2CSLST &= ~0x40;                    //处理START事件
    }
    else if (I2CSLST & 0x20)
    {
        I2CSLST &= ~0x20;                       //处理RECV事件
        if (isda)
        {
            isda = 0;                           //处理RECV事件（RECV DEVICE ADDR）
        }
        else if (isma)
        {
            isma = 0;                           //处理RECV事件（RECV MEMORY ADDR）
            addr = I2CRXD;
            I2CTXD = buffer[addr];
        }
        else
        {   
            buffer[addr++] = I2CRXD;            //处理RECV事件（RECV DATA）
					  if(addr>=8)addr=0;
					 
        }
    }
    else if (I2CSLST & 0x10)
    {
        I2CSLST &= ~0x10;                       //处理SEND事件
        if (I2CSLST & 0x02)
        {
            I2CTXD = 0xff;                      //接收到NAK则停止读取数据
        }
        else
        {
            I2CTXD = buffer[++addr];            //接收到ACK则继续读取数据
        }
    }
    else if (I2CSLST & 0x08)
    {
        I2CSLST &= ~0x08;                       //处理STOP事件
        isda = 1;
        isma = 1;
    }

    _pop_(P_SW2);
}
  
void main()
{		
	// 初始化引脚为低电平
    PWM_A1 = 0;
    PWM_A2 = 0;
    PWM_B1 = 0;
    PWM_B2 = 0;
	
	  R_RGB_r = 1; // 默认关闭RGB灯
    R_RGB_g = 1;
    R_RGB_b = 1;
	
	  L_RGB_r = 1; 
  	L_RGB_g = 1;
	  L_RGB_b = 1;
	
    P0M0 = 0x00;
    P0M1 = 0xFF;
    P1M0 = 0x00;
    P1M1 = 0x00;
    P2M0 = 0x00;
    P2M1 = 0x00;
    P3M0 = 0x7C;
    P3M1 = 0x00;
    P4M0 = 0x00;
    P4M1 = 0x00;
    P5M0 = 0x00;
    P5M1 = 0x00;

    P_SW2 = 0x80;
	

    I2CCFG = 0x81;                              //使能I2C从机模式
    I2CSLADR = 0X30;                            //设置从机设备地址寄存器I2CSLADR=0101_1010B
                                                //即I2CSLADR[7:1]=010_1101B,MA=0B。
                                                //由于MA为0,主机发送的的设备地址必须与
                                                //I2CSLADR[7:1]相同才能访问此I2C从机设备。
                                                //主机若需要写数据则要发送5AH(0101_1010B)
                                                //主机若需要读数据则要发送5BH(0101_1011B)
    I2CSLST = 0x00;
    I2CSLCR = 0x78; // 0x00                     //禁止从机模式中断
		EA = 1;
		
    isda = 1;                                   //用户变量初始化
    isma = 1;
    addr = 1;
    I2CTXD = buffer[addr];
		setMotorSpeed(0,0);
			
    while (1){
						
			if(buffer[0] == 1){
			
				if(count>100)count=0;  // 如果计数器count的值大于100，则将其重置为0
				 count+=1;  // 计数器count的值加1
				 Delay1ms();  // 延时1微秒

				 PWM=buffer[6];  // 从buffer数组的第三个元素（索引为2）读取PWM值

				if(PWM>100)PWM=100;  // 如果PWM值大于100，则将其限制在100
				if(PWM<0)PWM=0;  // 如果PWM值小于0，则将其限制在0

				if(count<PWM)  // 如果计数器count的值小于PWM值
				 {
					setMotorSpeed(buffer[2],buffer[4]);  // 调用setMotorSpeed函数设置电机速度
				 }
				else 
					setMotorSpeed(0,0);  // 否则，将电机速度设置为0（停止电机）
			}
			
			if(buffer[0] == 2){
				
				if(buffer[6] == 1){
				
					R_RGBColor(1,0,0);
					L_RGBColor(1,0,0);
					delay_ms(200);		
					
					R_RGBColor(0,1,0);
					L_RGBColor(0,1,0);
					delay_ms(200);			
					
					R_RGBColor(0,0,1);
					L_RGBColor(0,0,1);
					delay_ms(200);		

					R_RGBColor(0,0,0);
					L_RGBColor(0,0,0);
					delay_ms(50);						
				}
				
				
			}
			
		}   
}