该目录可以存放用户代码。
//-----------------------------------------------------------------------------------------------------------------------------------/
逐飞方案修善完整篇，可以简单跑
初赛道长26米左右，不加环岛
到苯环为1.5米左右
电磁接口05->06,17->14

大大优化电感处理代码，修改了错误
丢线有问题，还有速度不分开处理，均速作为速度环输入有问题，按理说没问题，不知道为什么，有待研究

大大优化了蜂鸣器的问题，蜂鸣器中断触发函数完全写好了

优化按键调参菜单页面,IPS144一行最大30个字符

加重一侧的电感值有效果，但是还是不能达到内切效果——不行

放弃两斜电感改为横电感



/*
//----------------------------------------------------------------------------------------------------------------------------------
老行调参函数列
oled_init();

#define Max_Par										(12)//最大页面
#define Time_ms										(50)
#define bark_time									(20)//必须小于Time_ms

count[1] = direction_P;
count[2] = direction_I;
count[3] = direction_D;
count[4] = inductance_a;
count[5] = inductance_b;
count[6] = inductance_c;
count[7] = dirangle_D;
count[8] = STOP;
count[9] = 0;
count[10] = 0;
count[11] = 0;
count[12] = 0;

//参数控制
extern int8 precision;
extern float count[20];
int8 all_parameter = 0;
float count[20] = {0};
int8 precision = 0;//调节控制精度
//简约菜单
void Oled_Show(void)
{
		switch(row)
	{	
		case 7://标志位行
//			oled_uint16(0,6,ringdir_flag);oled_uint16(8*5,6,inring_flag);oled_int16(8*10,6,Fring_control);
//			oled_printf_float(8*5,6,disdance,1,2);oled_int16(8*10,6,start_Fring);
		case 6://角度环行
			oled_printf_float(0,5,angle_now,3,1);oled_printf_float(8*6,5,angle_x_inter,4,1);
		case 5://方向环行
			oled_printf_float(0,4,direction_pid_out,4,1);oled_printf_float(8*7,4,direction_error,3,1);
		case 4://电机输出行,参数行
			//刷新页面
			if(clear_now != clear_old)
			{
				oled_fill(0x00);
				clear_old = clear_now;
			}
			//参数显示
			if(all_parameter == 0)
			{
			oled_printf_float(8,3,Motor_Lout,4,1);oled_printf_float(8*7,3,Motor_Rout,4,1);
			}
			else if(all_parameter == 1)
			{
				oled_p6x8str(8,3,"dir_kp");oled_printf_float(8*6,3,direction_Kp,4,1);				
			}
			else if(all_parameter == 2)
			{
				oled_p6x8str(8,3,"dir_ki");oled_printf_float(8*6,3,direction_Ki,4,1);			
			}
			else if(all_parameter == 3)
			{
				oled_p6x8str(8,3,"dir_kd");oled_printf_float(8*6,3,direction_Kd,4,1);			
			}
			else if(all_parameter == 4)
			{
				oled_p6x8str(8,3,"err_pa");oled_printf_float(8*6,3,Pa,4,1);	
			}
			else if(all_parameter == 5)
			{
				oled_p6x8str(8,3,"err_pb");oled_printf_float(8*6,3,Pb,4,1);			
			}
			else if(all_parameter == 6)
			{
				oled_p6x8str(8,3,"err_pc");oled_printf_float(8*6,3,Pc,4,1);			
			}
			else if(all_parameter == 7)
			{
				oled_p6x8str(8,3,"ang_kd");oled_printf_float(8*6,3,dirangle_Kd,2,1);			
			}			
			else if(all_parameter == 8)
			{
				oled_p6x8str(8,3,"lim_s");oled_printf_float(8*6,3,distance,2,1);
			}
			else if(all_parameter == 9)
			{
				oled_p6x8str(8,3,"pre_n");oled_printf_float(8*6,3,precision,2,1);
			}
			else if(all_parameter == 10)
			{
				oled_p6x8str(8,3,"dir_fg");oled_printf_float(8*6,3,direction_flag,1,1);
			}
			else if(all_parameter == 11)
			{
				oled_p6x8str(8,3,"upr_fg");oled_printf_float(8*6,3,upright_flag,1,1);
			}
			else if(all_parameter == Max_Par)
			{
				oled_p6x8str(8,3,"chuankou");oled_printf_float(8*7,3,chuankou_button,1,1);
			}
		case 3://电感行
			if(zuizhi_button == OFF)
			{
				oled_uint16(0,2,MYADC[2][0]);oled_uint16(8*5,2,MYADC[3][0]);
			}
			else
			{
				oled_uint16(0,2,ADC_Max[2]);oled_uint16(8*5,2,ADC_Max[3]);//最值
			}				
		case 2://电感行
			if(zuizhi_button == OFF)
			{
				oled_int16(0,1,MYADC[0][0]);oled_int16(8*5,1,MYADC[1][0]);
			}
			else
			{
				oled_uint16(0,1,ADC_Max[0]);oled_uint16(8*5,1,ADC_Max[1]);//最值
			}
		case 1://基础行
			oled_uint16(0,0,voltage);oled_printf_float(8*5,0,S,2,2);
//			oled_printf_float(0,0,speed_left,1,1);oled_printf_float(8*5,0,speed_right,1,1);
			break;
		case 0:
			oled_fill(0x00);
			break;
		default:oled_p6x8str(0,0,"There are only 7 row!");
	}
}
//----------------------------------------------------按键---------------------------------------------------------/

//按键调参,简约菜单版
void Key_Control(void)
{
	//调参精度控制
	float num = 0;
	if(precision == 0)
	{
		num = 1;
	}
	else if(precision == -1)
	{
		num = 0.1;
	}
	else if(precision == 1)
	{
		num = 10;
	}
	else
	{
		num = 1;
	}
	//翻页按钮
	if(Key3 == 0)
	{
		
		delay_ms(Time_ms-bark_time);
		buzzer_bark(bark_time);
		if(Key3 == 0)
		{
			all_parameter++;
			//页面刷新
			clear_old = clear_now;
			clear_now++;
			if(clear_now > 1)
			{
				clear_now = 0;
			}
			//限制
			if(all_parameter > Max_Par)
			{
				all_parameter = 0;
			}
			delay_ms(Time_ms);
		}
	}
	if(Key4 == 0)
	{
		delay_ms(Time_ms-bark_time);
		buzzer_bark(bark_time);
		if(Key4 == 0)
		{
			all_parameter--;

			clear_old = clear_now;
			clear_now++;
			if(clear_now > 1)
			{
				clear_now = 0;
			}
			
			if(all_parameter < 0)
			{
				all_parameter = Max_Par;
			}
			delay_ms(Time_ms);
		}
	}
	//调参按钮
	if(Key2 == 0)
	{
		
		delay_ms(Time_ms-bark_time);
		buzzer_bark(bark_time);
		if(Key2 == 0)
		{
			if(all_parameter == 1)
			{
				count[all_parameter] = count[all_parameter] + num;
				direction_Kp = count[all_parameter];
			}
			else if(all_parameter == 2)
			{
				count[all_parameter] = count[all_parameter] + num;
				direction_Ki = count[all_parameter];
			}
			else if(all_parameter == 3)
			{
				count[all_parameter] = count[all_parameter] + num;
				direction_Kd = count[all_parameter];
			}
			else if(all_parameter == 4)
			{
				count[all_parameter] = count[all_parameter] + num;
				Pa = count[all_parameter];
			}
			else if(all_parameter == 5)
			{
				count[all_parameter] = count[all_parameter] + num;
				Pb = count[all_parameter];
			}
			else if(all_parameter == 6)
			{
				count[all_parameter] = count[all_parameter] + num;
				Pc = count[all_parameter];
			}
			else if(all_parameter == 7)
			{
				count[all_parameter] = count[all_parameter] + num;
				dirangle_Kd = count[all_parameter];
			}
			else if(all_parameter == 8)
			{
				count[all_parameter] = count[all_parameter] + num;
				distance = count[all_parameter];
			}
			else if(all_parameter == 9)
			{
				count[all_parameter] = count[all_parameter] + 1;
				precision = count[all_parameter];
			}
			else if(all_parameter == 10)
			{
				count[all_parameter] = 1;
				direction_flag = count[all_parameter];
			}
			else if(all_parameter == 11)
			{
				count[all_parameter] = count[all_parameter] + 1;
				upright_flag = count[all_parameter];
			}
			else if(all_parameter == Max_Par)
			{
				count[all_parameter] = ON;
				chuankou_button = count[all_parameter];
			}
			delay_ms(Time_ms);
		}
	}
	if(Key1 == 0)
	{
		
		delay_ms(Time_ms-bark_time);
		buzzer_bark(bark_time);
		if(Key1 == 0)
		{
			if(all_parameter == 1)
			{
				count[all_parameter] = count[all_parameter] - num;
				direction_Kp = count[all_parameter];
			}
			else if(all_parameter == 2)
			{
				count[all_parameter] = count[all_parameter] - num;
				direction_Ki = count[all_parameter];
			}
			else if(all_parameter == 3)
			{
				count[all_parameter] = count[all_parameter] - num;
				direction_Kd = count[all_parameter];
			}
			else if(all_parameter == 4)
			{
				count[all_parameter] = count[all_parameter] - num;
				Pa = count[all_parameter];
			}
			else if(all_parameter == 5)
			{
				count[all_parameter] = count[all_parameter] - num;
				Pb = count[all_parameter];
			}
			else if(all_parameter == 6)
			{
				count[all_parameter] = count[all_parameter] - num;
				Pc = count[all_parameter];
			}
			else if(all_parameter == 7)
			{
				count[all_parameter] = count[all_parameter] - num;
				dirangle_Kd = count[all_parameter];
			}
			else if(all_parameter == 8)
			{
				count[all_parameter] = count[all_parameter] - num;
				distance = count[all_parameter];
			}
			else if(all_parameter == 9)
			{
				count[all_parameter] = count[all_parameter] - 1;
				precision = count[all_parameter];
			}
			else if(all_parameter == 10)
			{
				count[all_parameter] = 0;
				direction_flag = count[all_parameter];
			}
			else if(all_parameter == 11)
			{
				count[all_parameter] = count[all_parameter] - 1;
				upright_flag = count[all_parameter];
			}
			else if(all_parameter == Max_Par)
			{
				count[all_parameter] = OFF;
				chuankou_button = count[all_parameter];
			}
			delay_ms(Time_ms);
		}
	}
}

//开机(速度环)复位按钮
void Key_Speedcontrol(void)
{
	if(Key5 == 0)
	{
		delay_ms(Time_ms-bark_time);
		buzzer_bark(bark_time);
		if(Key5 == 0)
		{
			delay_ms(Time_ms);
			if(stop_flag == 0)
			{
				speed_flag++;
				clear_flag = 1;
			}
			else
			{
				stop_flag = 0;
				S = 0;
			}

			if(speed_flag == 2)
			{
				speed_flag = 0;
			}
		}
	}
}

void Key_PIDstart(void)
{
	Key_Speedcontrol();
	Key_Control();
}

//	if(direction_error >= 50)
//	{
//		fold_flag = 1;
//	}
//	else if(direction_error <= -50)
//	{
//		fold_flag = 2;
//	}
//	
//	if(direction_error >= 80)
//	{
//		fold_flag = 3;
//	}
//	else if(direction_error <= -80)
//	{
//		fold_flag = 4;
//	}
//	
//	if(direction_error >= 80)
//	{
//		fold_flag = 5;
//	}
//	else if(direction_error <= -80)
//	{
//		fold_flag = 6;
//	}
//	else
//	{
//		fold_flag = 0;
//	}

//			if(fold_flag == 1)
//			{
//				Foldline_Control(30);
//				Motor_Lout = L_OUT+angle_inter_out;
//				Motor_Rout = R_OUT-angle_inter_out;			
//			}
//			else if(fold_flag == 2)
//			{
//				Foldline_Control(-30);
//				Motor_Lout = L_OUT+angle_inter_out;
//				Motor_Rout = R_OUT-angle_inter_out;	
//			}
//			else if(fold_flag == 3)
//			{
//				Foldline_Control(60);
//				Motor_Lout = L_OUT+angle_inter_out;
//				Motor_Rout = R_OUT-angle_inter_out;	
//			}
//			else if(fold_flag == 4)
//			{
//				Foldline_Control(-60);
//				Motor_Lout = L_OUT+angle_inter_out;
//				Motor_Rout = R_OUT-angle_inter_out;	
//			}
//			else if(fold_flag == 5)
//			{
//				Foldline_Control(90);
//				Motor_Lout = L_OUT+angle_inter_out;
//				Motor_Rout = R_OUT-angle_inter_out;	
//			}
//			else if(fold_flag == 6)
//			{
//				Foldline_Control(-90);
//				Motor_Lout = L_OUT+angle_inter_out;
//				Motor_Rout = R_OUT-angle_inter_out;	
//			}

//----------------------------------------------------------------------------------------------------------
ADC误差环，放弃案
void ADC_Control(void);

extern int16 ADC_PID_OUT0,ADC_PID_OUT1;

int16 ADC_ERR0 = 0,ADC_ERR1 = 0;
int16 ADC_Last0 = 0;ADC_Last1 = 0;
int16 ADC_PID_OUT0 = 0,ADC_PID_OUT1 = 0;

void ADC_Control(void)
{
	float KP0,KD0,KP1,KD1;
	KP0 = 2.5;KP1 = 2.5;
	KD0 = 0;KD1 = 0;
	
	
	ADC_ERR0 = abs(MYADC[1][0] - MYADC[0][0]);
	ADC_ERR1 = abs(MYADC[3][0] - MYADC[2][0]);
	
	
	ADC_PID_OUT0 = KP0*ADC_ERR0 + KD0*(ADC_ERR0 - ADC_Last0);
	ADC_PID_OUT1 = KP1*ADC_ERR1 + KD1*(ADC_ERR1 - ADC_Last1);
	func_limit_ab(ADC_PID_OUT0,-6000,6000);
	func_limit_ab(ADC_PID_OUT1,-6000,6000);
	
	ADC_Last0 = ADC_ERR0;
	ADC_Last1 = ADC_ERR1;
}

right_Ki = right_speed_I;
left_Ki = left_speed_I;

//------------------------------------------------------------------------------------------------------------	
	//增量式
//	//左轮PID输出
//	speed_left_error[0] = (Left_Tspeed - (speed_left+speed_right)/2);
//	func_limit_ab(speed_left_error[0],-speed_error_limit,speed_error_limit);	
//	left_pid_out += left_Kp*(speed_left_error[0] - speed_left_error[1]) + left_Ki*speed_left_error[0] + left_Kd*(speed_left_error[0] - 2*speed_left_error[1] + speed_left_error[2]);
//	func_limit_ab(left_pid_out,-speed_out_limit,speed_out_limit);
//	speed_left_error[2] = speed_left_error[1];
//    speed_left_error[1] = speed_left_error[0];
//	
//	//右轮PID输出
//	speed_right_error[0] = (Right_Tspeed - (speed_left+speed_right)/2);
//	func_limit_ab(speed_right_error[0],-speed_error_limit,speed_error_limit);
//	right_pid_out += right_Kp*(speed_right_error[0] - speed_right_error[1]) + right_Ki*speed_right_error[0] + right_Kd*(speed_right_error[0] - 2*speed_right_error[1] + speed_right_error[2]);
//	func_limit_ab(right_pid_out,-speed_out_limit,speed_out_limit);
//	speed_right_error[2] = speed_right_error[1];
//	speed_right_error[1] = speed_right_error[0];
//------------------------------------------------------------------------------------------------------------
老计时方案的避障，配合红外激光模块
if(difficult_flag == 2)
{
	difficult_count++;
	if(difficult_count <= 30)//避开打角30
	{
		Motor_Rout = -2000;
		Motor_Lout = 3000;
	}
	else if((difficult_count > 30) && (difficult_count <= 65))//35
	{
		Motor_Rout = 2000;
		Motor_Lout = 2000;
	}
	else if((difficult_count > 65) && (difficult_count <= 95))//打角30
	{
		Motor_Rout = 3000;
		Motor_Lout = -2500;
	}
	else if((difficult_count > 95) && (difficult_count <= 125))//30
	{
		Motor_Rout = 2000;
		Motor_Lout = 2000;
	}
	else if((difficult_count > 125) && (difficult_count <= 155))//打角30
	{
		Motor_Rout = 3000;
		Motor_Lout = -2500;
	}
	else if((difficult_count > 155) && (difficult_count <= 180))//25
	{
		Motor_Rout = 2000;
		Motor_Lout = 2000;
	}
	else if(difficult_count <= 220)//回赛道40
	{
		Motor_Rout = -2000;
		Motor_Lout = 3000;
	}
	else
	{
		difficult_count = 0;
		left_inter_out = 0;
		right_inter_out = 0;
	}
}
else//正常走
{
	Motor_Lout = L_OUT;
	Motor_Rout = R_OUT;
}
//-----------------------------------------------------------------------------------------------------------------------------------
直角处理
//			if(fold_flag == 7)
//			{
//				Foldline_Control((direction_error*1.5+52));
//				Motor_Lout = left_pid_out + direction_pid_out-angle_inter_error*angle_inter_out;
//				Motor_Rout = right_pid_out - direction_pid_out-angle_inter_error*angle_inter_out;
//				
//				if(angle_x_inter >= 90)
//				{
//					fold_flag = 0;
//				}
//			}
//			else if(fold_flag == -7)
//			{
//				Foldline_Control(-(direction_error*1.5+52));
//				Motor_Lout = left_pid_out + direction_pid_out+angle_inter_error*angle_inter_out;
//				Motor_Rout = right_pid_out - direction_pid_out+angle_inter_error*angle_inter_out;
//				
//				
//				if(angle_x_inter >= 90)
//				{
//					fold_flag = 0;
//				}
//			}
//			if(fold_flag == 7)
//			{
//				Motor_Lout = left_pid_out + 1.5*direction_pid_out;
//				Motor_Rout = right_pid_out - 1.5*direction_pid_out;
//				
//				if(angle_x_inter >= 90)
//				{
//					fold_flag = 0;
//				}
//			}
//			else if(fold_flag == -7)
//			{
//				Motor_Lout = left_pid_out + 1.5*direction_pid_out;
//				Motor_Rout = right_pid_out - 1.5*direction_pid_out;
//				
//				
//				if(angle_x_inter >= 90)
//				{
//					fold_flag = 0;
//				}
//			}
//-----------------------------------------------------------------------------------------------------------------------------------
折线配合角度环调整输出
//			if(fold_flag == 1)
//			{
//				diffspeed_control();
//				Motor_Lout = left_pid_out + direction_pid_out + diffspeed_pid_out;
//				Motor_Rout = right_pid_out - direction_pid_out + diffspeed_pid_out;
//			}
//			else if(fold_flag == 2)
//			{
//				diffspeed_control();
//				Motor_Lout = left_pid_out + direction_pid_out + diffspeed_pid_out;
//				Motor_Rout = right_pid_out - direction_pid_out + diffspeed_pid_out;
//			}
//-----------------------------------------------------------------------------------------------------------------------------------
折线直角标志位处理，有问题
//	if(direction_error >= 100)
//	{
//		fold_flag = 7;
//		fold_count++;
//		if(angle_x_inter > 90)
//		{
//			fold_flag = 0;
//		}
//	}
//	else if(direction_error <= -100)
//	{
//		fold_flag = -7;
//		fold_count++;
//		if(angle_x_inter > 90)
//		{
//			fold_flag = 0;
//		}		
//	}
//	else
//	{
//		fold_count = 0;
//		fold_flag = 0;
//	}

//-----------------------------------------------------------------	蜂鸣器------------------------------------------------------------------
//有问题的放弃案
void buzzer_count(int8 mode)
{
	buzzer_mode = mode;
	if(buzzer_mode != 0)
	{
		if(buzzerIT_count <= bark_time)//中断10ms叫0.2s
		{
			buzzerIT_count++;
		}
		else
		{
			if(buzzer_mode == 1)
			{
				buzzerIT_count = 102;//蜂鸣器叫一段时间
			}
			else
			{
				buzzerIT_count = 0;
			}
		}
	}
}

void Buzzer_Show(void)
{
//	if(((direction_error >= 150)||(direction_error <= -150)))
//	{
//		buzzer_count(1);
//	}
//	else
//	{
//		if(buzzer_mode != 0)
//		{
//			buzzerIT_count = -1;
//		}
//	}
//----------------------------------------------------------buzzer_bark----------------------------------------------------------//
//中断蜂鸣器函数有缺陷，一次只能叫一次，一个标志叫
	if(Key1 == 0)
	{
		buzzerIT_count = 0;
	}
	else if(Key2 == 0)
	{
		buzzerIT_count = 0;
	}
	else if(Key3 == 0)
	{
		buzzerIT_count = 0;
	}
	else if(Key4 == 0)
	{
		buzzerIT_count = 0;
	}
	else if(Key5 == 0)
	{
		buzzerIT_count = 0;
	}

	if((buzzerIT_count > 0)&&(buzzerIT_count <= bark_time))
	{
		P77 = 1;
	}
	else if(buzzerIT_count == 102)
	{
		P77 = 0;
	}
	else if(buzzerIT_count == -1)
	{
		P77 = 0;
		buzzerIT_count = 0;
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------
速度改限幅，减小速度
//	if((direction_error>=50)&&(direction_error<=80))
//	{
//		Right_Tspeed = speed_target-0.2;
//		Left_Tspeed = speed_target-0.2;
//	}
//	if((direction_error>=80)&&(direction_error<=150))
//	{
//		Right_Tspeed = speed_target-0.4;
//		Left_Tspeed = speed_target-0.4;
//	}
//	else if((direction_error>150))
//	{
//		Right_Tspeed = speed_target-2;
//		Left_Tspeed = speed_target-2;
//	}
	
//	if((direction_error>=50)&&(direction_error<=80))
//	{
//		speed_out_limit = 0;
//	}
//	if((direction_error>=80)&&(direction_error<=150))
//	{
//		speed_out_limit = 0;
//	}
//	else if((direction_error>150))
//	{
//		speed_out_limit = 2000;
//	}

//	if((direction_error>=50)&&(direction_error<=60))
//	{
//		speed_out_limit = 0;
//	}
//	else if((direction_error>60)&&(direction_error<=70))
//	{
//		speed_out_limit = 4000;
//	}
//	else if((direction_error>70)&&(direction_error<=80))
//	{
//		speed_out_limit = 0;
//	}
//	else if((direction_error>80)&&(direction_error<=90))
//	{
//		speed_out_limit = 4000;
//	}
//	else if((direction_error>90)&&(direction_error<=100))
//	{
//		speed_out_limit = 0;
//	}
//	else if((direction_error>100)&&(direction_error<=110))
//	{
//		speed_out_limit = 4000;
//	}
//	else if((direction_error>110)&&(direction_error<=120))
//	{
//		speed_out_limit = 0;
//	}
//	else if((direction_error>120)&&(direction_error<=130))
//	{
//		speed_out_limit = 4000;
//	}
//	else if((direction_error>130)&&(direction_error<=140))
//	{
//		speed_out_limit = 0;
//	}
//	else if((direction_error>140))
//	{
//		speed_out_limit = 4000;
//	}

//----------------------------------------------------------------------------------------------------------------------------------

//					Motor_Lout = left_pid_out + angle_inter_out*MYADS((diffangle0+30)-angle_x_inter)/(diffangle0+30);
//					Motor_Rout = right_pid_out - angle_inter_out*MYADS((diffangle0+30)-angle_x_inter)/(diffangle0+30);
避障二次打角案一
//				//避障第二次打角
//				if(difficult_count == 1)
//				{
//					angle_flag = 1;
//					Foldline_Control(-100);
//					Motor_Lout = left_pid_out + angle_inter_out;
//					Motor_Rout = right_pid_out - angle_inter_out;
//					if((angle_x_inter <= -120)&&(angle_x_inter >= -140))
//					{
//						difficult_count = 2;
//					}
//				}
//				//一点距离后直走
//				if(difficult_count == 2)
//				{
//					Bee(1);
////					speed_target = target_speed;
//					direction_flag = 1;
//					angle_flag = 0;
//					angle_inter_out = 0;
//					angle_x_inter = 0;

//					Motor_Lout = left_pid_out;
//					Motor_Rout = right_pid_out;

//					//到与第一次打角反向时误差回归正常
//					if((difficult_count0 == 2)&&(direction_error <= -50))
//					{
//						Bee(0);
//						difficult_flag = 0;
//						difficult_count = 0;
////						difficult_count0 = 4;						
//					}
//					else if((difficult_count0 == 3)&&(direction_error >= 50))
//					{
//						Bee(0);
//						difficult_flag = 0;
//						difficult_count = 0;
////						difficult_count0 = 4;
//					}
//				}

//避障回复有问题
//		if(MYADS(direction_error) <= 20)
//		{
//			difficult_count++;//二次利用
//		}
//		if(difficult_count >= 3)
//		{
//			difficult_count = 0;
//			nojudge_flag = 0;
//			difficult_count0 = 5;
//		}
//------------------------------------------------------------------------------------------------------------------
电感修正
//	if((direction_error>=100)&&(angle_x_inter<=90))
//	{
////		MYADC[0][0] = MYADC[0][0];
////		MYADC[2][0] = MYADC[2][0]-200+direction_error;
//		
//		MYADC[1][0] = MYADC[1][0] + direction_error + 1.5*fold_count;
//		MYADC[3][0] = MYADC[3][0] + direction_error + 1.5*fold_count;
//	}
//	else if((direction_error<=-100)&&(angle_x_inter<=90))
//	{
//		
//		MYADC[0][0] = MYADC[0][0] + direction_error + 1.5*fold_count;
//		MYADC[2][0] = MYADC[2][0] + direction_error + 1.5*fold_count;
//		
////		MYADC[1][0] = MYADC[1][0];
////		MYADC[3][0] = MYADC[3][0]-200+direction_error;
//	}
//-------------------------------------------------------------------------------------------------------------------
问题处理

//	if(difficult_flag == 1)
//	{
//		diuxian_flag = 0;
//		diuxian_count1 = 0;

//		prering_flag = 0;
//		ringdir_flag = 0;
//		staring_flag = 0;
//		inring_flag = 0;
//		angle_x_inter = 0;
//		ring_angle_inter = 0;
//		ring_out_dis = 0;
//		outring_flag = 0;
//	}

//--------------------------------------------------------------------------------------------------------------------
tof模块避障
//	if(dl1a_finsh_flag == 1)//有障碍
//	{
//		difficult_dis = dl1a_distance_mm;
//		
//		difficult_flag = 1;
//		
//		dl1a_finsh_flag = 0;
//	}
//---------------------------------------------------------------------------------------------------------------------
//折线
void fold_line(void)
{
	if(direction_error >= 100)//右打
	{
		fold_flag = 1;
	}
	else if(direction_error <= -100)//左打
	{
		fold_flag = 2;
	}
	else
	{
		fold_flag = 0;
	}
}

//		fold_line();
//			//折线控制
//			if(fold_flag == 1)
//			{
//				
//				Kr = 1/(MYADS(speed_right));
//				Kl = 1/(MYADS(speed_left));		
//		
//				Motor_Lout = left_pid_out + direction_pid_out*Kl;
//				Motor_Rout = right_pid_out - direction_pid_out*Kr;
//			}
//			else if(fold_flag == 2)
//			{
//				Kr = 1/(MYADS(speed_right));
//				Kl = 1/(MYADS(speed_left));
//				
//				Motor_Lout = left_pid_out + direction_pid_out*Kl;
//				Motor_Rout = right_pid_out - direction_pid_out*Kr;
//			}

{120, 100, 200, 160, 100, 150, 140, 100, 200},
{120, 100, 200, 160, 100, 150, 120, 100, 200},
{140, 100, 200, 160, 100, 180, 140, 100, 220}

//入环          环内          出环
void Elementparameter_Init(void)
{
//1
	ring_parameter[0][0] = Kp_start;
	ring_parameter[0][1] = 100;
	ring_parameter[0][2] = 200;

	ring_parameter[0][3] = 250;
	ring_parameter[0][4] = 50;
	ring_parameter[0][5] = 275;

	ring_parameter[0][6] = Kp_out;
	ring_parameter[0][7] = 100;
	ring_parameter[0][8] = 200;
//1	
	ring_parameter[0][0] = 450;
	ring_parameter[0][1] = 100;
	ring_parameter[0][2] = 200;

	ring_parameter[0][3] = 250;
	ring_parameter[0][4] = 50;
	ring_parameter[0][5] = 275;

	ring_parameter[0][6] = 120;
	ring_parameter[0][7] = 100;
	ring_parameter[0][8] = 200;
//2	
	ring_parameter[2][0] = 550;
	ring_parameter[2][1] = 100;
	ring_parameter[2][2] = 200;
	
	ring_parameter[2][3] = 250;
	ring_parameter[2][4] = 100;
	ring_parameter[2][5] = 300;
	
	ring_parameter[0][6] = 120;
	ring_parameter[0][7] = 100;
	ring_parameter[0][8] = 200;
}

*/
