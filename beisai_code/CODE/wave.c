#include "headfile.h"


int8 trigger_flag = 0;

uint16 count[3] = {0};

uint16 dis_count = 0,get_count = 0;

float wave_dis = 0;


//初始化把P1_1配置为外部中断模式
void ultrasonic_waveinit(void)
{
	EAXFR = 1;
	CKCON = 0x00;
	WTST = 0X00;
	
	P1M0 = 0x00;
	P1M1 = 0x00;
	
	P1IM0 = 0x00;
	P1IM1 = 0x00;
	P1INTE = 0x00;
	
	EA = 1;
	
	gpio_mode(P1_0,GPO_PP);	//触发PIN
	gpio_mode(P1_1,GPIO);	//接受PIN
	
	P10 = 0;
}
//选择中断触发模式
void trigger_mode(int8 mode)
{
	switch(mode)
	{
		case 0:	P1IM1 = 0x00;P1IM0 = 0x00;break;	// P1.1设置为下降沿触发
		case 1: P1IM1 = 0x00;P1IM0 = 0x02;break;	// P1.1设置为上升沿触发
		case 2:	P1IM1 = 0x02;P1IM0 = 0x00;break;	// P1.1设置为低电平触发
		case 3: P1IM1 = 0x02;P1IM0 = 0x02;break;	// P1.1设置为高电平触发
	}
}
//中断使能失能
void waveIT_cmd(int8 cmd)
{
	switch(cmd)
	{
		case 0:	P1INTE = 0x00;break;//中断失能
		case 1: P1INTE = 0x02;break;//使能中断
	}	
}

void wave_trigger(void)
{
	//10ms触发
	if(trigger_flag == 0)
	{
		P10 = 1;
		trigger_flag = 1;
		waveIT_cmd(0);
	}
	else
	{
		P10 = 0;
		waveIT_cmd(1);
		trigger_flag=0;
	}	
}

void wave_exti(void)
{
	if(get_count == 0)
	{
		trigger_mode(0);//下降沿触发

		get_count = 1;
		
		TMOD |= 0x00; // 模式 0
		TL1 = 0; 	
		TH1 = 0;
		TR1 = 1; // 启动定时器
		ET1 = 1; // 使能定时器中断
	}
	else
	{
		get_count = 0;

		count[1] = (uint16)TH1 << 8;
		count[1] = ((uint8)TL1) | count[1];

		TR1 = 0;
		TH1 = 0;
		TL1 = 0;
		
		count[2] = (count[1]/10+dis_count*6553);

		dis_count = 0;
		wave_dis = (float)count[2]*0.0615;


		trigger_mode(1);//上升沿触发
	}	
}
