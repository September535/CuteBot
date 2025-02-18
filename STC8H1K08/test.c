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
int count_buffer, count, PWM;

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

void delay_ms(unsigned int ms) {
	unsigned int x;
    for (x = 0; x < ms; ++x) {
        Delay1ms();
    }
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

// 软件PWM控制RGB灯
void Software_PWM_Control() {
    static unsigned char pwm_counter = 0;

    // 控制左侧RGB灯
    L_RGB_r = (pwm_counter < L_R_pwm) ? 0 : 1;
    L_RGB_g = (pwm_counter < L_G_pwm) ? 0 : 1;
    L_RGB_b = (pwm_counter < L_B_pwm) ? 0 : 1;

    // 控制右侧RGB灯
    R_RGB_r = (pwm_counter < R_R_pwm) ? 0 : 1;
    R_RGB_g = (pwm_counter < R_G_pwm) ? 0 : 1;
    R_RGB_b = (pwm_counter < R_B_pwm) ? 0 : 1;

    // 更新PWM计数器
    pwm_counter++;
    if (pwm_counter >= 255) {
        pwm_counter = 0;
    }
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

void main() {
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

    I2CCFG = 0x81; // 使能I2C从机模式
    I2CSLADR = 0X30; // 设置从机设备地址
    I2CSLST = 0x00;
    I2CSLCR = 0x78; // 禁止从机模式中断
    PS = 1;
    EA = 1;

    isda = 1; // 用户变量初始化
    isma = 1;
    addr = 1;
    I2CTXD = buffer[addr];

    initMotor(&leftMotor);
    initMotor(&rightMotor);

    while (1) {
			
	setLMotor(&leftMotor, buffer[0], buffer[2]);  // 左轮正转，速度 70
        setRMotor(&rightMotor, buffer[1], buffer[3]); // 右轮反转，速度 50

        updateMotors(); // 更新电机状态
			
        // 设置左侧RGB亮度
        Set_Left_RGB_Brightness(119, 180, 119); // 红色最亮，绿色中等，蓝色较暗

        // 设置右侧RGB亮度
        Set_Right_RGB_Brightness(180, 119, 135); // 红色较暗，绿色中等，蓝色最亮

        // 更新RGB灯状态
        Software_PWM_Control();

    }
}
