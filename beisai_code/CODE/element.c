//参数元素设置判断
#include "basefile.h"

#define OUTUPLIMIT																(8000)
#define OUTDOWNLIMIT															(8000)


//元素参数数组
int8 mode = 0;
//并极输出
float R_OUT = 0,L_OUT = 0;
uint8 Ka = 0,Kb = 0;
//标志位
uint8 pid_control = 0;
uint8 fold_flag = 0;																			//折线
uint8 ringdir_flag = 0,outring_flag = 0,inring_flag = 0,staring_flag = 0,prering_flag = 0;		//环岛
uint8 difficult_flag = 0,nojudge_flag = 0;														//避障
uint8 diuxian_flag = 0,buxian_flag = 0;															//丢线
uint8 indicate_flag = 0;																		//指示位
//计数
uint16 leftr_count = 0,rightr_count = 0,ring_count0 = 0,ring_count1 = 0,ring_count2 = 0;
uint16 difficult_count = 0,difficult_count0 = 0;
uint16 fold_count = 0,fold_count0 = 0;
int16 diuxian_count1 = 0,diuxian_count2 = 0,diuxian_count3 = 0;
uint16 speed_count = 0,speed_downcount = 0,speedup_flag = 0;
//环岛元素
int16 ring_angle_inter = 0;
float ring_out_dis = 0,ring_in_dis = 0;
float ring_prer_dis = 0,ring_prel_dis = 0;
uint16 Kp_out = 0,Kd1_out = 0,Kd0_out = 0,Kp_start = 0,Kd1_start = 0,Kd0_start = 0;
int16 ADC_Mlimt = 0;
//避障元素
int8 diff_dir = 0;
int8 bz_count = 0,bz_count0 = 0;
float difficult_dis = 0,difficult_lastdis = 0,run_dis = 0;
float difficult_rdis = 0;
float rampway_dis = 0;
float addspeed_dis = 0;
int8 bz_ci = 0;
int8 bz_mode = -1;
float bz_dis = 0;
uint16 wave_limit = 0;
uint8 dis_limit = 0;
int8 agl1 = 0,agl2 = 0,agl1_limit = 0,agl2_limit = 0;
int16 agl2_Kp = 0,agl2_Kd = 0;
int8 error_limit = 0;
//折线
float Kr = 1.0,Kl = 1.0;
uint8 speed_downtime = 0;
uint8 fold_button = 0;
//电机输出
float Motor_Lout = 0,Motor_Rout = 0;
int16 Buxian_out = 0;
uint8 bingji_button = 0;
uint16 bingji_limit = 0;
int16 sub_limit = 0; 
//PID控制标志位
uint8 direction_flag = 0,speed_flag = 0,angle_flag = 0;
int8 car_mode = 0;
//蜂鸣器开关
int8 buzzer_button = 0;
//停车
float S = 0;
float distance = 0;
uint8 stop_flag = 0;
//清理过多计算
int8 clear_flag = 0;
//发车
int8 Go = 0;

//发车
void car_go(void)
{
	if(Go > 0)
	{
		direction_flag = 1;
		
		if(speed_count <= 200)
		{
			speed_count++;
		}
		else
		{
			speed_count = 0;
			Go = 0;
			speed_flag = 1;
		}
	}
}
//复位函数
void rest(void)
{
	stop_flag = 0;

	speed_flag = 0;
	direction_flag = 0;
	angle_flag = 0;

	clear_flag = 0;
	pid_control = 0;
	
	speed_count = 0;

	S = 0;

	direction_pid_out = 0;
	
	chuankou_button = OFF;
	//丢线
	diuxian_flag = 0;
	diuxian_count1 = 0;
	diuxian_count2 = 0;
	diuxian_count3 = 0;
	//环岛
	prering_flag = 0;
	ringdir_flag = 0;
	staring_flag = 0;
	inring_flag = 0;
	angle_x_inter = 0;
	ring_angle_inter = 0;
	ring_out_dis = 0;
	outring_flag = 0;
	//避障
	difficult_count0 = 0;
	difficult_flag = 0;
	difficult_count = 0;
	difficult_dis = 0;
	difficult_rdis = 0;
}
//低压报警
void Lowpressure_alarm(float V)
{
	if(direction_flag == 0)
	{
		if((voltage <= V*100)&&(voltage > 600))
		{
			rest();
			Bee(1);
		}
	}
}
//标志位控制PID计算
void PID_Count(void)
{
	if(direction_flag == 1)
	{
		Direction_Control(0.0);
	}
	
	if(speed_flag == 1)
	{
		clear_flag = 1;
		Speed_Control();
	}
	else
	{
		left_pid_out = 0;
		right_pid_out = 0;
	}
}

//蜂鸣器标志位判断
void buzzer_bark(void)
{	
	P77 = 1;
	delay_ms(bark_time);
	P77 = 0;
}

//转弯方案
void curve(void)
{
	uint16  Jasudu_out = 0;
	//弯道处理
	if(diuxian_flag == 0)
	{
		R_OUT = right_pid_out - direction_pid_out;
		L_OUT = left_pid_out + direction_pid_out;

		if(bingji_button == 1)
		{
			Buxian_out = MYADC[3][0]-MYADC[2][0];
			if(((Buxian_out>sub_limit)||(Buxian_out<-sub_limit))&&(inring_flag == 0)&&(outring_flag == 0))
			{
				if((MYADC[0][0]+MYADC[1][0])>=bingji_limit)//防止十字出问题
				{
					Buxian_out = 0;
				}
				R_OUT = R_OUT - Buxian_out*Kb;
				L_OUT = L_OUT + Buxian_out*Kb;
			}
		}


		if((inring_flag == 0)||(outring_flag == 0))
		{
			if(inductance>=20)
			{
//				Angle_Control();
//				R_OUT = R_OUT + direction_diff_out*direction_diff_out*Ka*speed_target + MYADS(angle_error_inter);
//				L_OUT = L_OUT - direction_diff_out*direction_diff_out*Ka*speed_target - MYADS(angle_error_inter);
				Jasudu_out = direction_diff_out*direction_diff_out*Ka*speed_target;
				R_OUT = R_OUT + Jasudu_out;
				L_OUT = L_OUT - Jasudu_out;	
			}
			else if(inductance<=-20)
			{
//				Angle_Control();
//				R_OUT = R_OUT - direction_diff_out*direction_diff_out*Ka*speed_target - MYADS(angle_error_inter);
//				L_OUT = L_OUT + direction_diff_out*direction_diff_out*Ka*speed_target + MYADS(angle_error_inter);
				R_OUT = R_OUT - Jasudu_out;
				L_OUT = L_OUT + Jasudu_out;				
			}
		}
		
	}
}

//丢线
void out_track(void)
{
	//丢线标志位判断

	if((MYADC[3][0]-MYADC[2][0])>500)
	{
		buxian_flag = 1;
	}
	else if((MYADC[2][0]-MYADC[3][0])>500)
	{
		buxian_flag = 2;
	}
	
	if((difficult_flag == 0)&&(speed_flag == 1)&&(MYADC[0][0] <= 100) && (MYADC[1][0]  <= 100) && (MYADC[2][0]  <= 100) && (MYADC[3][0]  <= 100) && (MYADC[4][0]  <= 100))
	{
		diuxian_count1++;
		if(buxian_flag == 1)
		{
			buxian_flag = 3;
		}
		else
		{
			buxian_flag = 4;
		}
		
		if((diuxian_count1 >= 2)&&(diuxian_count1 < 100))
		{
			diuxian_flag = 1;
		}
		else if(diuxian_count1 >= 100)
		{
			Motor_Rout = 0;
			Motor_Lout = 0;
			rest();
			diuxian_count2 = 0;
			diuxian_count1 = 0;
			diuxian_flag = 0;
		}
	}
	else
	{
		if(MYADC[4][0] >= 1500)
		{
			diuxian_count3++;
			if(diuxian_count3 >= 3)
			{
				diuxian_flag = 0;
				diuxian_count1 = 0;
				diuxian_count2 = 0;
				diuxian_count3 = 0;
				buxian_flag = 0;
			}
		}
	}
	
	if((difficult_flag == 0) && (diuxian_flag == 0) && (speed_flag == 1)&&(MYADC[0][0] <= 0) && (MYADC[1][0]  <= 0) && (MYADC[2][0]  <= 0) && (MYADC[3][0]  <= 0)&&(MYADC[4][0] <= 0))
	{
		if(diuxian_count1<=10)
		{
			diuxian_count1++;
			Motor_Rout = -5000;
			Motor_Lout = -5000;
		}
		else
		{
			diuxian_count1 = 0;
			Motor_Rout = 0;
			Motor_Lout = 0;
			rest();
		}
	}
}

//停车
void Car_Stop(float dis)
{
	if((S >= dis)&&(speed_flag == 1))
	{
		rest();
		stop_flag = 1;
		direction_flag = 1;
	}
}

//环岛
void rotary_island(void)
{
	//预入环
	if((prering_flag == 0)&&(outring_flag == 0))
	{
		if((MYADC[0][0] >= 2300)&&(MYADC[1][0] >= 2500))
		{
			ring_prer_dis += (speed_left+speed_right)*0.005;
		}

		if((MYADC[1][0] >= 2300)&&(MYADC[0][0] >= 2500))
		{
			ring_prel_dis += (speed_left+speed_right)*0.005;
		}

		if((ring_prer_dis>=0.2)||(ring_prel_dis>=0.2))
		{
			prering_flag = 1;
			ring_prer_dis = 0;
			ring_prel_dis = 0;
		}
	}
	
	//入环方向
	if((prering_flag == 1)&&(staring_flag == 0)&&(outring_flag == 0))
	{
		ring_out_dis += (speed_left+speed_right)*0.005;
		if((MYADC[3][0]-MYADC[2][0]) >= 80)
		{
			rightr_count++;
		}
		else
		{
			rightr_count = 0;
		}
		if((MYADC[2][0]-MYADC[3][0]) >= 80)
		{
			leftr_count++;
		}
		else
		{
			leftr_count = 0;
		}

		if((leftr_count>=2)||(rightr_count>=2))
		{
			if(leftr_count > rightr_count)//左环
			{
				ringdir_flag = 2;

				if((MYADC[2][0]<ADC_Mlimt)&&(MYADC[3][0]<ADC_Mlimt)&&((MYADC[0][0]+MYADC[1][0]) >= 4000))
				{
					ring_count1++;
				}
				if(ring_count1 >= 2)
				{
					staring_flag = 1;
					ring_count1 = 0;
				}
			}
			else if(leftr_count < rightr_count)//右环
			{
				ringdir_flag = 1;
				
				if((MYADC[2][0]<ADC_Mlimt)&&(MYADC[3][0]<ADC_Mlimt)&&((MYADC[0][0]+MYADC[1][0]) >= 4000))
				{
					ring_count1++;
				}
				if(ring_count1 >= 2)
				{
					staring_flag = 1;
					ring_count1 = 0;					
				}
			}
			else
			{
				leftr_count = rightr_count = 0;
			}
		}
	}
	
	//误判复位，十字那里前面可能会误判
	if((ring_out_dis >= 2)&&(staring_flag == 0)&&(outring_flag == 0)&&(inring_flag == 0))
	{
		prering_flag = 0;
		ringdir_flag = 0;
		leftr_count = rightr_count = 0;
		ring_out_dis = 0;
	}
	else if((staring_flag == 1)&&(outring_flag == 0)&&(inring_flag == 0))
	{
		ring_out_dis = 0;
	}


	//入环
	if((staring_flag == 1)&&(inring_flag == 0)&&(outring_flag == 0))
	{
		ring_angle_inter = angle_x_inter;
		ring_in_dis += (speed_left+speed_right)*0.005;
		pid_control = 1;
		if((ring_angle_inter >= 20)&&(ring_in_dis>=0.5))
		{
			if((MYADC[2][0]+MYADC[3][0])<1500)
			{
				inring_flag = 1;
				ring_in_dis = 0;
			}
		}
		else if((ring_angle_inter <= -20)&&(ring_in_dis>=0.5))
		{
			if((MYADC[2][0]+MYADC[3][0])<1500)
			{
				inring_flag = 1;
				ring_in_dis = 0;
			}
		}
		else
		{
			inring_flag = 0;
		}
	}
	
	if((inring_flag == 1)&&(outring_flag == 0))
	{
		Bee(1);
		ring_angle_inter = angle_x_inter;
		pid_control = 2;
	}

	//出环
	if((ring_angle_inter >= 315)&&(outring_flag == 0))
	{
		Bee(0);
		
		if((MYADC[0][0] >= 2300)&&(MYADC[1][0] >= 2500))
		{
			ring_out_dis = 0;
			outring_flag = 1;	
		}
	}
	else if((ring_angle_inter <= -315)&&(outring_flag == 0))
	{
		Bee(0);
		
		if((MYADC[1][0] >= 2300)&&(MYADC[0][0] >= 2500))
		{
			ring_out_dis = 0;
			outring_flag = 1;
		}
	}
	
	if(outring_flag == 1)
	{
		ring_out_dis += (speed_left+speed_right)*0.005;
	}
	
	if((outring_flag == 1)&&(ring_out_dis<=1.2))
	{
		Bee(1);
		
		prering_flag = 0;
		ringdir_flag = 0;
		staring_flag = 0;
		inring_flag = 0;

		rightr_count = 0;
		leftr_count = 0;		

		pid_control = 3;
		if(ring_out_dis>=1.4)
		{
			pid_control = 0;
		}
	}

	if((outring_flag == 1)&&(ring_out_dis>1.2))
	{
		Bee(0);
		ring_out_dis = 0;

		staring_flag = 0;
		inring_flag = 0;
		prering_flag = 0;
		ringdir_flag = 0;

		rightr_count = 0;
		leftr_count = 0;

		ring_angle_inter = 0;
		angle_x_inter = 0;
		
		outring_flag = 0;
		ring_count0 = 1;
	}

	//稳定后置位
	if(ring_count0 == 1)
	{
		ring_out_dis = 0;

		staring_flag = 0;
		inring_flag = 0;
		prering_flag = 0;
		ringdir_flag = 0;

		ring_angle_inter = 0;
		angle_x_inter = 0;
		
		outring_flag = 0;
//		ring_count2 = 1;

		if(direction_error >= -20)
		{
			leftr_count++;//二次利用
		}
		else if(direction_error <= 20)
		{
			rightr_count++;//二次利用
		}
		
		if((leftr_count+rightr_count) >= 4)
		{
			ring_count0 = 0;
			rightr_count = 0;
			leftr_count = 0;
		}
	}

	
	//各环参数调整
	if((prering_flag == 1)||(outring_flag == 1))
	{
		if(pid_control == 1)//入环
		{
			direction_Kp = Kp_start;
			direction_Kd = Kd1_start;
			dirangle_Kd = Kd0_start;		
		}
		else if(pid_control == 2)//环内
		{
			direction_Kp = 250;
			direction_Kd = 50;
			dirangle_Kd = 275;
		}
		else if(pid_control == 3)//出环
		{
			direction_Kp = Kp_out;
			direction_Kd = Kd1_out;
			dirangle_Kd = Kd0_out;
		}
		else//出环回复
		{
			direction_Kp = direction_P;
			direction_Kd = direction_D;
			dirangle_Kd = dirangle_D;		
		}
	}
}

//避障 
void obstacle_avoidance(void)
{
	//记距离避障
	if(difficult_count0 == 0)
	{
		difficult_dis = S;

//		if(((difficult_dis-difficult_lastdis)>=1)&&((difficult_dis-difficult_lastdis)<2))
//		{
//			Bee(1);

//		}
//		else if((difficult_dis-difficult_lastdis)>=2)
//		{
//			Bee(0);
//			difficult_lastdis = difficult_dis;
//		}
		
		if(bz_mode == 1)
		{
			if(difficult_dis >= bz_dis)
			{
				difficult_count0 = 1;//只一次

				difficult_flag = diff_dir;
				angle_flag = 1;
				nojudge_flag = 1;//其他元素停止判断
			}
		}
		else if(bz_mode == -1)
		{
			if(speed_flag == 1)
			{
				if(difficult_dis >= dis_limit)
				{
					if(bz_count >= 3)
					{
						if(bz_ci == 1)
						{
							bz_count0 = 1;
						}
						else if(bz_ci == 2)
						{
							bz_count0 = 1;
							if(rampway_dis >= 1.5)
							{
								bz_count0 = 2;
								rampway_dis = 0;
							}
						}
					}


					if((bz_count0 == 1)&&(bz_ci == 2))
					{
						rampway_dis += (speed_left + speed_right)*0.005;
					}
					
					if(bz_count0 == bz_ci)
					{
						difficult_count0 = 1;//只一次

						difficult_flag = diff_dir;
						angle_flag = 1;
						nojudge_flag = 1;//其他元素停止判断
						bz_count0 = 0;
						bz_count = 0;
					}
				}
			}
		}
	}
	else if(difficult_count0 == 4)//稳定后置位，开始判断其他环节
	{
		direction_Kp = 350;
		direction_Ki = direction_I;
		direction_Kd = 100;
		dirangle_Kd = 150;
		//误差稳定后标志位可置
		difficult_rdis += (speed_left+speed_right)/2;
		if(difficult_rdis >= 0.5)
		{
			direction_Kp = direction_P;
			direction_Ki = direction_I;
			direction_Kd = direction_D;
			dirangle_Kd = dirangle_D;
		}
		if(difficult_rdis >= 2)
		{
			difficult_rdis = 0;
			difficult_count = 0;
			nojudge_flag = 0;
			speedup_flag = 1;
			difficult_count0 = 5;			
		}
	}
	else if(difficult_count0 == 5)//重置避障环节
	{
		if(difficult_dis == road_dis)
		{
			difficult_count0 = 0;
		}
	}
}

//折线
void fold_line(void)
{
	if((fold_count0 == 0)&&(fold_flag == 0))
	{
		if(direction_error >= 80)//右打
		{
//			Bee(1);
			fold_flag = 1;
			fold_count0 = 1;
		}
		else if(direction_error <= -80)//左打
		{
//			Bee(1);
			fold_flag = 2;
			fold_count0 = 1;
		}
	}
	else if((fold_count0 == 1)&&(fold_flag == 0))
	{
		if(MYADS(direction_error)<=20)
		{
			fold_count++;
		}
		else
		{
			fold_count = 0;
		}
		
		if(fold_count>=3)
		{
//			Bee(0);
			fold_count = 0;
			fold_count0 = 0;
		}
	}
	else if((inring_flag == 1)||(staring_flag == 1))
	{
		fold_count0 = 0;
		fold_flag = 0;
	}
}
//标志位判断处理
void Element_Run(void)
{
	car_go();
	if(nojudge_flag == 0)
	{
		out_track();
//		if(ring_count2 == 0)
//		{

//		}
		if(inring_flag == 0)
		{
			if(fold_button == 1)
			{
				fold_line();
			}
		}
		rotary_island();		

	}
	if((bizhang_button == 1)&&(MYADS(direction_error) <= 50))
	{
//		obstacle_avoidance();
	}

	Car_Stop(distance);
//	Lowpressure_alarm(7.4);
}

void Motorout_Control(void)
{
	if(stop_flag == 0)
	{
		if(diuxian_flag == 0)
		{
			//正常走
			Motor_Lout = L_OUT;
			Motor_Rout = R_OUT;

			//折线控制
			if(fold_flag == 1)//右边
			{
				fold_count++;
				
				if(fold_count < speed_downtime)//40ms差速减速
				{
					Kr = 1.5;
					Kl = -2.5;
					Motor_Lout = (right_pid_out - direction_pid_out)*Kr;
					Motor_Rout = (left_pid_out + direction_pid_out)*Kl;
				}
				else if((fold_count >= speed_downtime)&&(fold_count < 10))
				{
					Kr = 2.7;
					Kl = 2.7;
					Motor_Lout = (left_pid_out + direction_pid_out)*Kl;
					Motor_Rout = (right_pid_out - direction_pid_out)*Kr;
				}
				else if(fold_count >= 10)
				{
					fold_count = 0;
					fold_flag = 0;
				}
			}
			else if(fold_flag == 2)//左边
			{
				fold_count++;

				if(fold_count < speed_downtime)//40ms差速减速
				{
					Kr = -2.5;
					Kl = 1.5;
					Motor_Lout = (right_pid_out - direction_pid_out)*Kr;
					Motor_Rout = (left_pid_out + direction_pid_out)*Kl;
				}
				else if((fold_count >= speed_downtime)&&(fold_count < 10))
				{
					Kr = 2.7;
					Kl = 2.7;
					Motor_Lout = (left_pid_out + direction_pid_out)*Kl;
					Motor_Rout = (right_pid_out - direction_pid_out)*Kr;
				}
				else if(fold_count >= 10)
				{
					fold_count = 0;
					fold_flag = 0;
				}
			}

			//避障控制
			if((difficult_flag == 1)&&(nojudge_flag == 1))
			{
				//避障第一次打角
				if(difficult_count == 0)
				{
					Bee(1);

					direction_flag = 0;//关闭方向环
					angle_flag = 1;//开启角度积分
					Foldline_Control(agl1);
					difficult_count0 = 2;//二次利用回正方向指示,打负角为2，打正角为3

					Motor_Lout = left_pid_out + angle_inter_out;
					Motor_Rout = right_pid_out - angle_inter_out;				
	
					if(speed_downcount <= 4)
					{
						speed_downcount++;
						Motor_Lout = 1.2*(right_pid_out - angle_inter_out);
						Motor_Rout = -2.5*(left_pid_out + angle_inter_out);
					}

					difficult_rdis += (speed_left + speed_right)*0.005;

					if((difficult_rdis >= run_dis)&&(angle_x_inter >= agl1_limit))
					{
						speed_downcount = 0;
						angle_inter_out = 0;
						angle_x_inter = 0;

						difficult_rdis = 0;

						difficult_count = 1;
					}
				}

				//避障第二次打角
				if(difficult_count == 1)
				{	
					difficult_rdis += (speed_left + speed_right)*0.005;
					pid_control = 1;

					if(angle_x_inter>=-agl2_limit)//转角
					{
						if(pid_control == 1)
						{
							angle_Kp = agl2_Kp;
							angle_Kd = agl2_Kd;
						}
						Foldline_Control(-agl2);
						addspeed_dis = difficult_rdis;
					}
					else if((angle_x_inter<=-agl2_limit)&&(difficult_rdis>addspeed_dis)&&(difficult_rdis<=(addspeed_dis+0.2)))
					{
						indicate_flag = 1;
						
						direction_flag = 1;

						pid_control = 0;
						angle_Kp = angle_P;
						angle_Kd = angle_D;
						angle_inter_out = 0;
						if(angle_x_inter<=-130)
						{
							Foldline_Control(-(agl2-10));							
						}
					}
					else if((difficult_rdis>=(addspeed_dis+0.2))&&(angle_x_inter<=-agl2_limit))
					{
						difficult_count = 2;
					}

					Motor_Lout = left_pid_out + angle_inter_out;
					Motor_Rout = right_pid_out - angle_inter_out;
				}
				
				//回正
				if(difficult_count == 2)
				{
					Motor_Lout = left_pid_out;
					Motor_Rout = right_pid_out;					
					//到与第一次打角反向时误差回归正常
					if((difficult_count0 == 2)&&(inductance <= -error_limit)&&(angle_x_inter <= -(agl2_limit-15)))
					{
						Bee(0);	
						angle_flag = 0;
						angle_x_inter = 0;

						difficult_rdis = 0;
						addspeed_dis = 0;

						difficult_flag = 0;
						difficult_count = 0;
						pid_control = 0;

						difficult_count0 = 4;				
					}			
				}
			}
			else if((difficult_flag == -1)&&(nojudge_flag == 1))
			{
				//避障第一次打角
				if(difficult_count == 0)
				{
					Bee(1);

					direction_flag = 0;//关闭方向环
					angle_flag = 1;//开启角度积分
					Foldline_Control(-agl1);
					difficult_count0 = 3;//二次利用回正方向指示,打负角为2，打正角为3

					Motor_Lout = left_pid_out + angle_inter_out;
					Motor_Rout = right_pid_out - angle_inter_out;

					difficult_rdis += (speed_left + speed_right)*0.005;

					if((difficult_rdis >= run_dis)&&(angle_x_inter <= -agl1_limit))
					{
						angle_inter_out = 0;
						angle_x_inter = 0;

						difficult_rdis = 0;

						difficult_count = 1;
					}
				}

				//避障第二次打角
				if(difficult_count == 1)
				{	
					difficult_rdis += (speed_left + speed_right)*0.005;
					pid_control = 1;

					if(angle_x_inter<=agl2_limit)//转角
					{
						if(pid_control == 1)
						{
							angle_Kp = agl2_Kp;
							angle_Kd = agl2_Kd;
						}
						Foldline_Control(agl2);
						addspeed_dis = difficult_rdis;
					}
					else if((angle_x_inter>=agl2_limit)&&(difficult_rdis>addspeed_dis)&&(difficult_rdis<=(addspeed_dis+0.2)))
					{
						indicate_flag = 1;
						
						direction_flag = 1;

						pid_control = 0;
						angle_Kp = angle_P;
						angle_Kd = angle_D;
						angle_inter_out = 0;
						if(angle_x_inter>=130)
						{
							Foldline_Control(agl2-10);							
						}
					}
					else if((difficult_rdis>=(addspeed_dis+0.2))&&(angle_x_inter>=agl2_limit))
					{
						difficult_count = 2;
					}

					Motor_Lout = left_pid_out + angle_inter_out;
					Motor_Rout = right_pid_out - angle_inter_out;
				}
				
				//回正
				if(difficult_count == 2)
				{
					Motor_Lout = left_pid_out;
					Motor_Rout = right_pid_out;					
					//到与第一次打角反向时误差回归正常
					if((difficult_count0 == 3)&&(inductance >= error_limit)&&(angle_x_inter >= (agl2_limit-15)))
					{
						Bee(0);
						angle_flag = 0;
						angle_x_inter = 0;

						difficult_rdis = 0;
						addspeed_dis = 0;

						difficult_flag = 0;
						difficult_count = 0;
						pid_control = 0;

						difficult_count0 = 4;
					}
				}
			}
		}
		else//丢线
		{
//			if(diuxian_count3 == 0)
//			{
//				diuxian_count2++;

//				Motor_Lout = (left_pid_out + direction_pid_out)*1.5;
//				Motor_Rout = (right_pid_out - direction_pid_out)*1.5;
//			}
//			else
//			{
//				Motor_Lout = (left_pid_out + direction_pid_out)*diuxian_count3;
//				Motor_Rout = (right_pid_out - direction_pid_out)*diuxian_count3;
//			}

			diuxian_count2++;
			if(buxian_flag == 3)
			{
				Motor_Lout = (left_pid_out + direction_pid_out)*1.5;
				Motor_Rout = (right_pid_out - direction_pid_out)*1;
			}
			else
			{
				Motor_Lout = (left_pid_out + direction_pid_out)*1;
				Motor_Rout = (right_pid_out - direction_pid_out)*1.5;				
			}
			
		}
	}
	else
	{
		Motor_Rout = -direction_pid_out;
		Motor_Lout = direction_pid_out;
	}

	func_limit_ab(Motor_Lout,-OUTDOWNLIMIT,OUTUPLIMIT);
	func_limit_ab(Motor_Rout,-OUTDOWNLIMIT,OUTUPLIMIT);
	//转弯输出速度限幅
//	if((MYADS(direction_error) >= 50)&&(difficult_flag == 0))
//	{
//		func_limit_ab(Motor_Lout,-6000,6000);
//		func_limit_ab(Motor_Rout,-6000,6000);
//	}
	
//	if((MYADS(direction_error) >= 50)&&(difficult_flag == 0))
//	{
//		func_limit_ab(Motor_Lout,-7000,7000);
//		func_limit_ab(Motor_Rout,-7000,7000);
//	}
}
