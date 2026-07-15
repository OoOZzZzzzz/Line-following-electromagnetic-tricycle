/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ790875685)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-12-18
 ********************************************************************************************************************/

#include "basefile.h"


/*
 *关于内核频率的设定，可以查看board.h文件
 *在board_init中,已经将P54引脚设置为复位
 *如果需要使用P54引脚,可以在board.c文件中的board_init()函数中删除SET_P54_RESRT即可
//sys_clk可选值:33177600, 30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600。
//设置系统频率，此频率需要跟STC-ISP软件中的 <输入用户程序运行时的IRC频率>选项的频率一致。
//如果频率设置不对，将会导致串口的数据不正常,PWM的工作不正常等等。
*/


void main()
{
	board_init();


	All_Init();

	trigger_mode(1);
	
	//NVIC优先级选择，1到3递增
	NVIC_SetPriority(TIMER1_IRQn,3);
	pit_timer_ms(TIM_4,10);
    while(1)
	{
		ChuanKou_Start();
		Key_Control();
		boma();
		if(clear_flag == 0)
		{
			IPS_Show();
		}

    }
}

//-----------------------------------------------------------------------------------------------------------------------/
//中断函数
void TM1_Isr() interrupt 3
{
	dis_count++;
}

void IOINT_Isr(void) interrupt 13
{
	uint8 intf;
	intf = P1INTF;
	if (intf)
	{
		P1INTF = 0x00;

		if (intf & 0x02)	//P1.1 口中断
		{
			wave_exti();
			if(wave_dis <= wave_limit)
			{
				bz_count++;
			}
			else
			{
				bz_count = 0;
			}
		}

	}
}

void TM4_Isr() interrupt 20
{
	TIM4_CLEAR_FLAG; //清除中断标志
//	ccd_collect();	 //CCD采集数据
	wave_trigger();
	
	//数据采集处理
	Value_Update();
	
	//元素处理
	Element_Run();

	//各环计算控制
	PID_Count();

	//差速处理
	curve();

	//电机输出处理
	Motorout_Control();

	//输出
	Pwm_Out(PWM_L1,PWM_L0,Motor_Lout);
	Pwm_Out(PWM_R1,PWM_R0,Motor_Rout);
	

}


