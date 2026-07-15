/*方向环控制*/
#include "basefile.h"
#include "math.h"

//方向环PD计算变量
float direction_error = 0;
float direction_last_error = 0;
float direction_inter_out = 0;
float direction_diff_out = 0;
float direction_pid_out = 0;
float dirangle_Kd = 0;
float dir_err[5] = {0};
float err_ave = 0;
//方向环PD参数
float direction_Kp = 0;
float direction_Kd = 0;
float direction_Ki = 10;
//方向环限幅参数
int16 direction_error_limit = 100;
int16 direction_inter_limit = 10;
int16 direction_out_limit = 8000;
//电感采集变量
//int16 ADC_L1 = 0,ADC_R7 = 0,ADC_L2 = 0,ADC_R6 = 0;

float ADC_Lvalue = 0,ADC_Rvalue = 0;
int16 MYADC[5][5] = {0};
int16 ADC_Max[5] = {0};
int16 ADC_Min = 0;
int16 K = 0;

float Pa = 0,Pb = 0,Pc = 0,Pp = 0;

uint16 normalizatio_time = 0;
//差比和计算变量
int16 inductance = 0;
//归一化开关
int8 guiyihua_button = 0,zuizhi_button = 0;
//陀螺仪
float angle_now = 0;
float angle_x[4] = {0};
float angle_x_inter = 0;
int16 angle_error_inter = 0;
float angle_pre = 0;

int8 inring_count = 0;

float angle_inter_error = 0,angle_inter_last = 0,angle_inter_out = 0;
float angle_Kp = 0,angle_Kd = 0;
//float direction_Kp3 = 0;

float angle_Ki = 0;

//积分标志位
uint8 angle_inter_flag = 0;



//归一化函数寻最值
void Min_Max_Normalization(uint16 time)
{
	int8 i;
	if(normalizatio_time >= time)
	{
		for(i=0;i<5;i++)
		{
			if(ADC_Max[i] < MYADC[i][0])
			{
				ADC_Max[i] = MYADC[i][0];
			}
		}
	}
	else
	{
		normalizatio_time++;
	}
}

//电感采集处理
void ADC_Getvalue(void)
{	
	int8 i,j,k;
	int16 temp;
	
//电感采集
	for(i=0;i<5;i++)
	{
		MYADC[0][i] = adc_once(ADC1,ADC_12BIT);
		MYADC[1][i] = adc_once(ADC7,ADC_12BIT);

		MYADC[2][i] = adc_once(ADC2,ADC_12BIT);
		MYADC[3][i] = adc_once(ADC6,ADC_12BIT);

		MYADC[4][i] = adc_once(ADC4,ADC_12BIT);
	}
//冒泡排序	
	for(i=0;i<5;i++)
    {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
              if(MYADC[i][k] > MYADC[i][k+1])
              {
                 temp = MYADC[i][k+1];
                 MYADC[i][k+1] = MYADC[i][k];
                 MYADC[i][k] = temp;
              }
           }
        }
    }
//取中间均值
	MYADC[0][0] = ((MYADC[0][1]+MYADC[0][2]+MYADC[0][3])/30)*10;
	MYADC[1][0] = ((MYADC[1][1]+MYADC[1][2]+MYADC[1][3])/30)*10;
	MYADC[2][0] = ((MYADC[2][1]+MYADC[2][2]+MYADC[2][3])/30)*10;
	MYADC[3][0] = ((MYADC[3][1]+MYADC[3][2]+MYADC[3][3])/30)*10;
	MYADC[4][0] = ((MYADC[4][1]+MYADC[4][2]+MYADC[4][3])/30)*10;

//找最值
	if(zuizhi_button == ON)
	{
		guiyihua_button = OFF;
		Min_Max_Normalization(100);
	}
//归一化
	if(guiyihua_button == ON)
	{
		for(i=0;i<5;i++)
		{
			MYADC[i][4] = 0;
		}
		for(i=0;i<5;i++)
		{
			
			if(MYADC[i][0] >= ADC_Min)
			{
				temp = (MYADC[i][0] - ADC_Min);
			}
			else
			{
				temp = 0;
			}
			MYADC[i][0] = GTSD((int16)(K*(float)temp/(float)(ADC_Max[i] - ADC_Min)));
			if(((MYADC[i][0]-MYADC[i][4])<=20)&&(MYADC[i][4]>20)&&(MYADC[i][0]>20))
			{
				MYADC[i][0] = MYADC[i][4];
			}
			MYADC[i][4] = MYADC[i][0];
		}
	}
//差比和计算
//----------------------------------------------------------------------------//
//	ADC_L1 = MYADC[0][0];
//	ADC_R7 = MYADC[1][0];
//	ADC_L2 = MYADC[2][0];
//	ADC_R6 = MYADC[3][0];
//差比和差加权，两横加两竖

//	inductance = (Pp+50)*(float)((MYADC[0][0] - MYADC[1][0]))/(float)(MYADC[0][0] + MYADC[1][0]);
//	inductance = (Pp+50)*((float)Pa*(MYADC[0][0] - MYADC[1][0]) + (float)Pb*(MYADC[2][0] - MYADC[3][0]))/((float)Pa*(MYADC[0][0] + MYADC[1][0]) + (float)Pb*(MYADC[2][0] + MYADC[3][0]) + 0.1);

	if(outring_flag == 1)
	{
		inductance = Pp*(float)(MYADC[0][0] - MYADC[1][0])/((float)(MYADC[0][0] + MYADC[1][0])+(float)1.5*abs((MYADC[2][0] - MYADC[3][0])));
	}
	else if((staring_flag == 1)&&(inring_flag == 0))
	{
		inductance = Pp*((float)8*(MYADC[0][0] - MYADC[1][0]) + (float)inring_count*(MYADC[2][0] - MYADC[3][0]))/((float)8*(MYADC[0][0] + MYADC[1][0]) + (float)2*abs((MYADC[2][0] - MYADC[3][0])) + 0.1);
	}
	else if(inring_flag == 1)
	{
		inductance = Pp*((float)8*(MYADC[0][0] - MYADC[1][0]) + (float)12*(MYADC[2][0] - MYADC[3][0]))/((float)8*(MYADC[0][0] + MYADC[1][0]) + (float)2*abs((MYADC[2][0] - MYADC[3][0])) + 0.1);		
	}
	else if(diuxian_flag == 1)
	{
		if(buxian_flag == 3)
		{
			inductance = -(120+diuxian_count2*15);
		}
		else
		{
			inductance = 120+diuxian_count2*15;
		}
	}
	else
	{
//		ADC_Lvalue = sqrt((MYADC[2][0]/100)*(MYADC[2][0]/100) + (MYADC[0][0]/100)*(MYADC[0][0]/100))*100;
//		ADC_Rvalue = sqrt((MYADC[3][0]/100)*(MYADC[3][0]/100) + (MYADC[1][0]/100)*(MYADC[1][0]/100))*100;

//		inductance = Pp*((float)Pa*(ADC_Lvalue- ADC_Rvalue) + (float)Pb*(MYADC[2][0] - MYADC[3][0]))/((float)Pa*(ADC_Lvalue + ADC_Rvalue) + (float)Pc*abs((MYADC[2][0] - MYADC[3][0])) + 0.1);
			
		inductance = Pp*((float)Pa*(MYADC[0][0] - MYADC[1][0]) + (float)Pb*(MYADC[2][0] - MYADC[3][0]))/((float)Pa*(MYADC[0][0] + MYADC[1][0]) + (float)2*MYADC[4][0] + (float)Pc*abs((MYADC[2][0] - MYADC[3][0])) + 0.1);
	}

//----------------------------------------------------------------------------//
}

//陀螺仪数据初始数据读取和处理函数
void IMU660RA_Kdread(void)
{
	imu660ra_get_gyro();
	
	angle_x[2] = angle_x[1];	
	angle_x[1] = angle_x[0];
	angle_x[0] = imu660ra_gyro_x;

	angle_x[3] = (angle_x[0]+angle_x[1]+angle_x[2])/3;
	if((staring_flag == 1)||(angle_flag == 1))
	{
		angle_x[3] = imu660ra_gyro_transition(angle_x[3]);
		angle_x_inter += angle_x[3]*0.01;
	}
}

//角度积分环
void Foldline_Control(int16 angle)
{
	angle_inter_error = angle - angle_x_inter;
	func_limit_ab(angle_inter_error,-360,360);
	angle_inter_out = angle_Kp*angle_inter_error + angle_Kd*(angle_inter_error - angle_inter_last);
	func_limit_ab(angle_inter_out,-8000,8000);
	angle_inter_last = angle_inter_error;
}

//角度环
void Angle_Control(void)
{
	angle_error_inter += angle_Ki*(angle_x[3] - angle_pre);
	
	angle_pre = angle_x[3];
}

//方向环位置式PD计算
void Direction_Control(float target)
{
	float x = 0;
	direction_out_limit = limit_direction_out;
	
	direction_error = target - inductance;
	func_limit_ab(direction_error,-direction_error_limit,direction_error_limit);

	if(car_mode == 1)
	{
		if(MYADS(direction_error)<=30)
		{
			direction_Kp = 50;
			direction_Kd = 100;
			dirangle_Kd = 300;
		}
		else if((MYADS(direction_error)>30)&&(MYADS(direction_error)<=50))
		{
			direction_Kp = 100;
			direction_Kd = 100;
			dirangle_Kd = 250;		
		}
		else if((MYADS(direction_error)>50)&&(MYADS(direction_error)<=80))
		{
			direction_Kp = 275;
			direction_Kd = 50;
			dirangle_Kd = 275;		
		}
		else if((MYADS(direction_error)>80)&&(MYADS(direction_error)<=100))
		{
			direction_Kp = 320;
			direction_Kd = 50;
			dirangle_Kd = 250;		
		}
		else if((MYADS(direction_error)>100)&&(MYADS(direction_error)<=200))
		{
			direction_Kp = 450;
			direction_Kd = 150;
			dirangle_Kd = 150;		
		}
	}
	else if(car_mode == 0)
	{
		x = MYADS(direction_error);
		direction_Kp = (x/14-1)*(x/14-1)*(x/14-1)+50;
		direction_Kp = x*x*x/2744+30;
		dirangle_Kd = (-x/10+5)*(-x/10+5)*(-x/10+5)+250;
		func_limit_ab(direction_Kp,50,450);
		func_limit_ab(dirangle_Kd,150,350);
	}


	
	if((diuxian_flag == 1)&&(diuxian_count3 == 0))
	{
		if((diuxian_count1 <= 50))
		{
			direction_out_limit = 10000;
			
			direction_Kp = 500;
			direction_Kd = 200;
			dirangle_Kd = 150;
		}
		else if((diuxian_count1 > 50))
		{
			direction_out_limit = 9000;

			direction_Kp = 400;
			direction_Kd = 100;
			dirangle_Kd = 275;
		}

	}
	else if((diuxian_flag == 1)&&(diuxian_count3 > 0))
	{
		direction_out_limit = 9000;
		
		direction_Kp = 350;
		direction_Kd = 100;
		dirangle_Kd = 275;
	}
	else if(nojudge_flag == 1)
	{
		direction_Kp = 300;
		direction_Kd = 100;
		dirangle_Kd = 275;
		direction_out_limit = 8000;
	}

	if(MYADS(direction_error) >= 50)
	{
		direction_inter_out += direction_error;
		func_limit_ab(direction_inter_out,-direction_inter_limit,direction_inter_limit);
	}
	else
	{
		direction_inter_out = 0;
	}
	
	
	direction_diff_out = direction_error - direction_last_error;
	direction_pid_out = direction_Kp*direction_error + direction_Ki*direction_inter_out - (dirangle_Kd/100)*angle_x[3] + direction_Kd*direction_diff_out;
//	direction_pid_out = direction_Kp3*direction_error*direction_error*direction_error + direction_Kp*direction_error + direction_Ki*direction_inter_out - (dirangle_Kd/100)*angle_x[3] + direction_Kd*direction_diff_out;
	func_limit_ab(direction_pid_out,-direction_out_limit,direction_out_limit);

	dir_err[4] = dir_err[3];
	dir_err[3] = dir_err[2];
	dir_err[2] = dir_err[1];
	dir_err[1] = dir_err[0];
	dir_err[0] = direction_error;
	err_ave = (dir_err[4]+dir_err[3]+dir_err[2]+dir_err[1]+dir_err[0])/5;

	direction_last_error = err_ave;
}
