/*速度环控制*/
#include "basefile.h"
#include "math.h"

//编码器采集值
int16 Encoder1 = 0,Encoder2 = 0;
//速度
float speed_left = 0.0,speed_right = 0.0;
float speed_average = 0.0;
//速度环PI变量
float left_error = 0,right_error = 0;
float speed_left_error[3] = {0};
float speed_right_error[3] = {0};
float left_inter_out = 0,right_inter_out = 0;
float left_pid_out = 0,right_pid_out = 0;
//速度环限幅参数
int16 speed_error_limit = 1000;
int16 speed_inter_limit = 1000;
int16 speed_out_limit = 8000;
//速度环PI参数
float left_Kp = 0,left_Ki = 0,left_Kd = 0;
float right_Kp = 0,right_Ki = 0,right_Kd = 0;
float speed_target = 0;


int16 AverageEncoder(int16 carspeed)
{
	int16 average_encoder=0,average_encoder1=0,average_encoder2=0;
	static int16 Pre7_Error[6];

	Pre7_Error[5]=Pre7_Error[4];
	Pre7_Error[4]=Pre7_Error[3];
	Pre7_Error[3]=Pre7_Error[2];
	Pre7_Error[2]=Pre7_Error[1];
	Pre7_Error[1]=Pre7_Error[0];
	Pre7_Error[0]=carspeed;

	average_encoder1 = Pre7_Error[3]*0.7+Pre7_Error[4]*0.2+Pre7_Error[5]*0.1;
	average_encoder2 = Pre7_Error[0]*0.7+Pre7_Error[1]*0.2+Pre7_Error[2]*0.1;

	average_encoder=0.75*average_encoder2+0.25*average_encoder1;
	return average_encoder;
}

//速度读取转化
void Speed_Measure(void)				
{
	Encoder1 = ctimer_count_read(Encoder_L);
	Encoder2 = ctimer_count_read(Encoder_R);

    ctimer_count_clean(Encoder_L);
	ctimer_count_clean(Encoder_R);
	
    if(Dir_L == 0)    
    {
		Encoder1 = -Encoder1;
    }
	if(Dir_R == 1)    
    {
        Encoder2 = -Encoder2;
    }

//加了速度平滑，有点莫名其妙的问题
//	Encoder1 = AverageEncoder(Encoder1);
//	Encoder2 = AverageEncoder(Encoder2);
	
	speed_left = Encoder1*transform;
	speed_right = Encoder2*transform;
	if(speed_flag == 1)
	{
		S += (speed_left+speed_right)*0.005;
	}
}

//速度环增量式PI计算
void Speed_Control(void)
{
	float Left_Tspeed = 0,Right_Tspeed = 0;

	Right_Tspeed = speed_target;
	Left_Tspeed = speed_target;
	speed_out_limit = limit_speed_out;

	//起步加速
	if(S <= 2)
	{
		if(S <= 0.5)
		{
			speed_out_limit = 10000;
			right_Kp = 1000;
			left_Kp = 1000;
		}
		else if((S > 0.5)&&(S <= 1))
		{
			speed_out_limit = 4000;
			right_Kp = 250;
			left_Kp = 250;			
		}
		else
		{
			speed_out_limit = limit_speed_out;
			right_Kp = right_speed_P;
			left_Kp = left_speed_P;
		}
	}

	//速度处理
	if(diuxian_flag == 0)
	{
		if((inring_flag == 1)&&(outring_flag == 0))
		{
			speed_out_limit = 6000;
			Right_Tspeed = 2.2;
			Left_Tspeed = 2.2;
			right_Kp = 250;
			left_Kp = 250;
		}
		else if(fold_count >= 5)
		{
			right_Kp = 900;
			left_Kp = 900;
		}
	}
	else if((diuxian_count1 <= 20)&&(diuxian_flag == 1))
	{
		speed_out_limit = 5000;
		right_Kp = 250;
		left_Kp = 250;
		Right_Tspeed = 0;
		Left_Tspeed = 0;
	}
	else if((diuxian_count1 > 20)&&(diuxian_flag == 1))
	{
		speed_out_limit = speed_out_limit + 100;
		func_limit_ab(speed_out_limit,0,6000);
		right_Kp = 250;
		left_Kp = 250;	
		Right_Tspeed = 2.0;
		Left_Tspeed = 2.0;
	}
	else if(nojudge_flag == 1)
	{
		speed_out_limit = 4000;
		Right_Tspeed = 2.2;
		Left_Tspeed = 2.2;
	}

	//左轮PID输出
	speed_left_error[0] = (Left_Tspeed - speed_left);
	func_limit_ab(speed_left_error[0],-speed_error_limit,speed_error_limit);	
	left_pid_out += left_Kp*(speed_left_error[0] - speed_left_error[1]) + left_Ki*speed_left_error[0] + left_Kd*(speed_left_error[0] - 2*speed_left_error[1] + speed_left_error[2]);
	func_limit_ab(left_pid_out,-speed_out_limit,speed_out_limit);
	speed_left_error[2] = speed_left_error[1];
    speed_left_error[1] = speed_left_error[0];
	
	//右轮PID输出
	speed_right_error[0] = (Right_Tspeed - speed_right);
	func_limit_ab(speed_right_error[0],-speed_error_limit,speed_error_limit);
	right_pid_out += right_Kp*(speed_right_error[0] - speed_right_error[1]) + right_Ki*speed_right_error[0] + right_Kd*(speed_right_error[0] - 2*speed_right_error[1] + speed_right_error[2]);
	func_limit_ab(right_pid_out,-speed_out_limit,speed_out_limit);
	speed_right_error[2] = speed_right_error[1];
	speed_right_error[1] = speed_right_error[0];
}

//电机PWM输出控向
void Pwm_Out(PWMCH_enum pwmch1,PWMCH_enum pwmch0,int32 Out)		
{
	if(Out >= 0)
	{
		pwm_duty(pwmch1,Out);
		pwm_duty(pwmch0,0);
	}
	else
	{
		pwm_duty(pwmch1,0);
		pwm_duty(pwmch0,-Out);
	}
}

