#include "STC8xxxx.H"
#include "intrins.h"

#define PWM5_2      0x01	//P1.7
#define PWM6_2      0x04	//P5.4
#define ENO5P       0x01
#define ENO6P       0x04
#define ENO7P       0x10
#define ENO8P       0x40
#define PWM_PERIOD  1000

unsigned int PWM_Period = 50;  // PWM周期，单位为定时器计数值

// PWM引脚定义
sbit PWM_A1 = P1^7; // 电机A正转
sbit PWM_A2 = P5^4; // 电机A反转
sbit PWM_B2 = P3^4; // 电机B正转
sbit PWM_B1 = P3^3; // 电机B反转

sbit L_RGB_r = P3^2;
sbit L_RGB_g = P3^5;
sbit L_RGB_b = P3^6;

sbit R_RGB_r = P1^6;
sbit R_RGB_g = P1^2;
sbit R_RGB_b = P1^0;

// 状态标志位
bit isda; // 设备地址标志
bit isma; // 存储地址标志
unsigned char addr; // 存储接收到的内存地址
unsigned char buffer[8]; // 存储接收到的数据
unsigned char pwm_counter = 0; // PWM计数器
unsigned char count = 0; // PWM计数器

// PWM占空比的控制变量 (0-255)
unsigned char L_R_pwm = 0;  // 左侧红色亮度
unsigned char L_G_pwm = 0;  // 左侧绿色亮度
unsigned char L_B_pwm = 0;  // 左侧蓝色亮度

unsigned char R_R_pwm = 0;  // 右侧红色亮度
unsigned char R_G_pwm = 0;  // 右侧绿色亮度
unsigned char R_B_pwm = 0;  // 右侧蓝色亮度

unsigned char index;

typedef struct {
    int speed;      // 当前速度（0-100）
    int direction;  // 方向：1（正转），2（反转），0（停止）
    int brake;      // 刹车标志：1=刹车，0=正常
} Motor;

Motor leftMotor = {0, 0};  // 左轮电机
Motor rightMotor = {0, 0}; // 右轮电机
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
	
	PWMB_PSCR = 0x0000;	// 预分频寄存器
	PWMB_DTR  = 0;		// 死区时间配置
	PWMB_ARR = 100;	// 自动重装载寄存器,  控制PWM周期
	PWMB_CCER1  = 0;
	PWMB_CCER2  = 0;
	PWMB_SR1    = 0;
	PWMB_SR2    = 0;
	PWMB_ENO    = 0;		// IO输出允许
	PWMB_PS     = 0;
	PWMB_IER    = 0;

	PWMB_CCMR1 = 0x68; //通道模式配置
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
	PWMB_BrakeOutputEnable();   //使能主输出
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

// 定时器0初始化函数
void Timer0_Init() {
    TMOD |= 0x01;  // 设置定时器0为模式1（16位定时器）
    TH0 = (65536 - PWM_Period) / 256;  // 初始化定时器0高8位
    TL0 = (65536 - PWM_Period) % 256;  // 初始化定时器0低8位
    ET0 = 1;  // 使能定时器0中断
    EA = 1;   // 全局中断使能
    TR0 = 1;  // 启动定时器0
}


// 定时器0中断服务函数
void Timer0_ISR() interrupt 1 {
    static unsigned int count = 0;
    TH0 = (65536 - PWM_Period) / 256;  // 重新加载定时器0高8位
    TL0 = (65536 - PWM_Period) % 256;  // 重新加载定时器0低8位
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
				count = 0;  // 计数器归零
		}
}


// 控制M2电机的速度


void setRMotor(int direction, int speed) {
	
    // 将输入0-100映射到70-100
    int mapped_speed = 70 + (speed * 30 / 100);
    
    // 确保映射后的速度在70-100范围内
    if (mapped_speed < 70) mapped_speed = 70;
    if (mapped_speed > 100) mapped_speed = 100;


    if (direction == 1) { // 正转
			
			PWMB_CCR7 = 0;
			PWMB_CCR8 = mapped_speed*8;
			
    } else if (direction == 2) { // 反转
			
			PWMB_CCR8 =0;
			PWMB_CCR7 = mapped_speed*8;

    } else { // 停止
        PWMB_CCR7 = 0;
			PWMB_CCR8 = 0;
    }
}

// 控制M1电机的速度

int pwm_value = 0;

void setLMotor(int direction, int speed) {
    // 将输入0-100映射到70-100
    int mapped_speed = 70 + (speed * 30 / 100);
    
    // 确保映射后的速度在70-100范围内
    if (mapped_speed < 70) mapped_speed = 70;
    if (mapped_speed > 100) mapped_speed = 100;
    
    // 转换为PWM值（假设PWM范围0-800）
     pwm_value = mapped_speed * 8;

    if (direction == 1) { // 正转
        PWMB_CCR6 = pwm_value;
        PWMB_CCR5 = 0;
    } 
    else if (direction == 2) { // 反转
        PWMB_CCR6 = 0;
        PWMB_CCR5 = pwm_value;
    } 
    else { // 停止
        PWMB_CCR6 = 0;
        PWMB_CCR5 = 0;
    }
}

// 设置左侧RGB的亮度值
void Set_Left_RGB_Brightness(unsigned char red, unsigned char green, unsigned char blue) {
    L_R_pwm = red/5;
    L_G_pwm = green/5;
    L_B_pwm = blue/5;
	
	  if(red <= 10)L_RGB_r = 1;
	  if(green <= 10)L_RGB_g = 1;
	  if(blue <= 10)L_RGB_b = 1;
	
}

// 设置右侧RGB的亮度值
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
        I2CSLST &= ~0x40; // 处理START事件
    } else if (I2CSLST & 0x20) {
        I2CSLST &= ~0x20; // 处理RECV事件
        if (isda) {
            isda = 0; // 处理RECV事件（RECV DEVICE ADDR）
        } else if (isma) {
            isma = 0; // 处理RECV事件（RECV MEMORY ADDR）
            addr = I2CRXD;
            I2CTXD = buffer[addr];
        } else {
            buffer[addr++] = I2CRXD; // 处理RECV事件（RECV DATA）
            if (addr >= 8) addr = 0;
        }
    } else if (I2CSLST & 0x10) {
        I2CSLST &= ~0x10; // 处理SEND事件
        if (I2CSLST & 0x02) {
            I2CTXD = 0xff; // 接收到NAK则停止读取数据
        } else {
            I2CTXD = buffer[++addr]; // 接收到ACK则继续读取数据
        }
    } else if (I2CSLST & 0x08) {
        I2CSLST &= ~0x08; // 处理STOP事件
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
	PWMB_PSCR = 0x0000;	// 预分频寄存器
	PWMB_DTR  = 0;		// 死区时间配置
	PWMB_ARR = 1000;	// 自动重装载寄存器,  控制PWM周期
	PWMB_CCER1  = 0;
	PWMB_CCER2  = 0;
	PWMB_SR1    = 0;
	PWMB_SR2    = 0;
	PWMB_ENO    = 0;		// IO输出允许
	PWMB_PS     = 0;
	PWMB_IER    = 0;
	
 
    /* PWM5 -- P17 */
	PWMB_CCMR1  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMB_CCR5   = 0;	// 比较值, 控制占空比(高电平时钟数)
	PWMB_CCER1 |= 0x01;		// 开启比较输出, 高电平有效
	PWMB_PS    |= 1;		// 选择IO
	PWMB_ENO   |= 0x01;		// IO输出允许
 
    /* PWM6 -- P54 */
	PWMB_CCMR2  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMB_CCR6   = 0;	// 比较值, 控制占空比(高电平时钟数)
	PWMB_CCER1 |= 0x10;		// 开启比较输出, 高电平有效
	PWMB_PS    |= (1<<2);	// 选择IO
	PWMB_ENO   |= 0x04;		// IO输出允许
 
    /* PWM7 -- P33 */
	PWMB_CCMR3  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMB_CCR7   = 0;	// 比较值, 控制占空比(高电平时钟数)
	PWMB_CCER2 |= 0x01;		// 开启比较输出, 高电平有效
	PWMB_PS    |= (1<<4);	// 选择IO
	PWMB_ENO   |= 0x10;		// IO输出允许
 
    /* PWM8 -- P34 */
	PWMB_CCMR4  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许
	PWMB_CCR8   = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMB_CCER2 |= 0x10;		// 开启比较输出, 高电平有效
	PWMB_PS    |= (1<<6);	// 选择IO
	PWMB_ENO   |= 0x40;		// IO输出允许
 
	PWMB_EGR    = 0x01;		//产生一次更新事件
	PWMB_BRK    = 0x80;		// 主输出使能 相当于总开关
	PWMB_CR1    = 0x81;		// 使能计数器, 允许自动重装载寄存器缓冲
}

void main() {
    
    P1M0 = 0xcf; P1M1 = 0x00;  
    P3M0 = 0xff; P3M1 = 0x00; 
    P5M0 = 0xff; P5M1 = 0x00; 

    P_SW2 = 0x80;

    I2CCFG = 0x81; // 使能I2C从机模式
    I2CSLADR = 0X30; // 设置从机设备地址
    I2CSLST = 0x00;
    I2CSLCR = 0x78; // 禁止从机模式中断
		

    R_RGB_r = 1; // 默认关闭RGB灯
    R_RGB_g = 1;
    R_RGB_b = 1;

    L_RGB_r = 1;
    L_RGB_g = 1;
    L_RGB_b = 1; 
		
    PS = 1;
    
    // 初始化定时器0
    Timer0_Init();
    
    EA = 1; // 开启总中断

    isda = 1; // 用户变量初始化
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