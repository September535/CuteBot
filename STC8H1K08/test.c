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

// PWM占空比的控制变量 (0-255)
unsigned char L_R_pwm = 0;  // 左侧红色亮度
unsigned char L_G_pwm = 0;  // 左侧绿色亮度
unsigned char L_B_pwm = 0;  // 左侧蓝色亮度

unsigned char R_R_pwm = 0;  // 右侧红色亮度
unsigned char R_G_pwm = 0;  // 右侧绿色亮度
unsigned char R_B_pwm = 0;  // 右侧蓝色亮度

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

typedef struct {
    int speed;      // 当前速度（0-100）
    int direction;  // 方向：1（正转），2（反转），0（停止）
} Motor;

Motor leftMotor = {0, 0};  // 左轮电机
Motor rightMotor = {0, 0}; // 右轮电机

void initMotor(Motor *motor) {
    motor->speed = 0;
    motor->direction = 0;
}

// 控制M2电机的速度
void setRMotor(Motor *motor, int direction, int speed) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    motor->speed = speed;
    motor->direction = direction;

    if (direction == 1) { // 正转
        PWM_B1 = 0;
        PWM_B2 = 1;
    } else if (direction == 2) { // 反转
        PWM_B1 = 1;
        PWM_B2 = 0;
    } else { // 停止
        PWM_B1 = 0;
        PWM_B2 = 0;
    }
}
// 控制M1电机的速度
void setLMotor(Motor *motor, int direction, int speed) {
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    motor->speed = speed;
    motor->direction = direction;

    if (direction == 1) { // 正转
        PWM_A1 = 0;
        PWM_A2 = 1;
    } else if (direction == 2) { // 反转
        PWM_A1 = 1;
        PWM_A2 = 0;
    } else { // 停止
        PWM_A1 = 0;
        PWM_A2 = 0;
    }
}

void updateMotors() {
    if (count > 100) count = 0; // 计数器重置
    count += 1;

    // 控制左轮
    if (count < leftMotor.speed) {
        PWM_A1 = (leftMotor.direction == 2) ? 1 : 0;
        PWM_A2 = (leftMotor.direction == 1) ? 1 : 0;
    } else {
        PWM_A1 = 0;
        PWM_A2 = 0;
    }

    // 控制右轮
    if (count < rightMotor.speed) {
        PWM_B1 = (rightMotor.direction == 2) ? 1 : 0;
        PWM_B2 = (rightMotor.direction == 1) ? 1 : 0;
    } else {
        PWM_B1 = 0;
        PWM_B2 = 0;
    }
}

void Timer0_Init(void) {
    TMOD = 0x02;  // 设置定时器0为自动重载模式
    TH0 = 0xFF;   // 设置定时器初值
    TL0 = 0xFF;
    EA = 1;       // 开启总中断
    ET0 = 1;      // 使能定时器0中断
    TR0 = 1;      // 启动定时器0
	
	  PT0 = 0;      // Timer0 中断优先级为低
}

// 定时器0中断服务函数，用于PWM输出
void Timer0_ISR(void) interrupt 1 {
    static unsigned int counter = 0;
    counter++;
    
    if (counter < 255) {
        // 左侧RGB控制
        if (counter < L_R_pwm) L_RGB_r = 0;  // 红色亮度
        else L_RGB_r = 1;
        
        if (counter < L_G_pwm) L_RGB_g = 0;  // 绿色亮度
        else L_RGB_g = 1;
        
        if (counter < L_B_pwm) L_RGB_b = 0;  // 蓝色亮度
        else L_RGB_b = 1;

        // 右侧RGB控制
        if (counter < R_R_pwm) R_RGB_r = 0;  // 红色亮度
        else R_RGB_r = 1;
        
        if (counter < R_G_pwm) R_RGB_g = 0;  // 绿色亮度
        else R_RGB_g = 1;
        
        if (counter < R_B_pwm) R_RGB_b = 0;  // 蓝色亮度
        else R_RGB_b = 1;
    } else {
        counter = 0;
    }
    
    TH0 = 0xFF;   // 重载定时器
    TL0 = 0xFF;
}

// 设置左侧RGB的亮度值
void Set_Left_RGB_Brightness(unsigned char red, unsigned char green, unsigned char blue) {
    L_R_pwm = red;
    L_G_pwm = green;
    L_B_pwm = blue;
}

// 设置右侧RGB的亮度值
void Set_Right_RGB_Brightness(unsigned char red, unsigned char green, unsigned char blue) {
    R_R_pwm = red;
    R_G_pwm = green;
    R_B_pwm = blue;
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
//    unsigned char left_red, left_green, left_blue;
//    unsigned char right_red, right_green, right_blue;
	
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
		PS = 1; 
		EA = 1;
		
    isda = 1;                                   //用户变量初始化
    isma = 1;
    addr = 1;
    I2CTXD = buffer[addr];
		
		initMotor(&leftMotor);
    initMotor(&rightMotor);
    
		Timer0_Init();
			
    while (1){
			
        // 示例：左轮正转，速度为 70；右轮反转，速度为 50
        setLMotor(&leftMotor, buffer[0], buffer[2]);  // 左轮正转，速度 70
        setRMotor(&rightMotor, buffer[1], buffer[3]); // 右轮反转，速度 50

        updateMotors(); // 更新电机状态
        Delay1ms();    // 延时 1ms

if(buffer[3] == 4 || buffer[3] == 6){
	  Set_Left_RGB_Brightness(buffer[0], buffer[1], buffer[2]);
	}
	    
if(buffer[3] == 5 || buffer[3] == 6){
	  Set_Right_RGB_Brightness(buffer[0], buffer[1], buffer[2]);
	}     
 }   
}
