#include "basefile.h"

uint16 voltage = 0;

//电压显示
void Voltage_Protect(void)
{

	uint16 ad_result = 0;
	uint32 temp = 0;

	ad_result = adc_once(protect,ADC_12BIT);
	temp = (((uint32)ad_result * 5000) / 4096);
	
	voltage = temp*0.3;
}

//ADC初始化
void MyADC_Init(void)
{
	adc_init(ADC1,ADC_SYSclk_DIV_2);
	adc_init(ADC7,ADC_SYSclk_DIV_2);
	adc_init(ADC4,ADC_SYSclk_DIV_2);	
	adc_init(ADC2,ADC_SYSclk_DIV_2);
	adc_init(ADC6,ADC_SYSclk_DIV_2);
	
	adc_init(protect,ADC_SYSclk_DIV_32);
}

//编码器初始化
void Encoder_Init(void)
{
	ctimer_count_init(Encoder_L);	
	ctimer_count_init(Encoder_R);
	ctimer_count_clean(Encoder_L);
	ctimer_count_clean(Encoder_R);
}

//电机初始化
void MyPwm_Init(void)
{
	pwm_init(PWM_L1, 17000, 0);
	pwm_init(PWM_L0, 17000, 0);
	pwm_init(PWM_R1, 17000, 0);
	pwm_init(PWM_R0, 17000, 0);
}

//按键初始化
void Key_Init(void)
{	
	gpio_mode(Key5,GPIO);
	gpio_mode(Key4,GPIO);
	gpio_mode(Key3,GPIO);
	gpio_mode(Key2,GPIO);
	gpio_mode(Key1,GPIO);
	
	gpio_mode(Boma2,GPIO);
	gpio_mode(Boma1,GPIO);
}

void Buzzer_Init(void)
{
	gpio_mode(buzzer,GPO_PP);
	P77 = 0;
}

//参数初始化设置
void Parameter_Init(void)
{
	left_Kp = left_speed_P;
	left_Ki = left_speed_I;
	left_Kd = left_speed_D;

	right_Kp = right_speed_P;
	right_Ki = right_speed_I;
	right_Kd = right_speed_D;

	inring_count = 20;
	speed_target = target_speed;

	distance = STOP;

	direction_Kp = direction_P;
	direction_Ki = direction_I;
	direction_Kd = direction_D;
	dirangle_Kd = dirangle_D;

	angle_Kp = angle_P;
	angle_Kd = angle_D;
	angle_Ki = angle_I;

	Pa = inductance_a;
	Pb = inductance_b;
	Pc = inductance_c;
	Pp = inductance_p;
	
	direction_error_limit = limit_direction_error;
	direction_inter_limit = limit_direction_inter;
	direction_out_limit = limit_direction_out;
	
	speed_error_limit = limit_speed_error;
	speed_inter_limit = limit_speed_inter;
	speed_out_limit = limit_speed_out;
	//归一化系数
	K = K_guiyi;
	if(zuizhi_button == OFF)
	{
		ADC_Max[0] = ADC_SLm;
		ADC_Max[1] = ADC_SRm;
		ADC_Max[2] = ADC_MLm;
		ADC_Max[3] = ADC_MRm;
		ADC_Max[4] = (ADC_MRm+ADC_SRm)/2;		
	}
	ADC_Min = 10;
	
	diff_dir = DIR;
	bz_dis = DIS;
	bz_mode = -1;
	bz_ci = BZCI;
	wave_limit = 70;
	dis_limit = 0;
	
	agl1 = 60;
	agl2 = 120;
	
	agl2_Kp = 200;
	agl2_Kd = 200;
	
	agl1_limit = 50;
	agl2_limit = 110;
	
	error_limit = 10;
	
	speed_downtime = 4;

	Ka = 5;
	Kb = 4;
	bingji_limit = 3000;
	sub_limit = 300;
	
	Kp_start = 350;
	Kp_out = 120;
	Kd0_start = 200;
	Kd0_out = 200;
	Kd1_start = 100;
	Kd1_out = 100;
	
	run_dis = 0.7;
	ADC_Mlimt = 1500;
	
	car_mode = 0;

}

//开关设置
void Button_Set(void)
{
	fold_button = 0;
	bingji_button = 1;
	chuankou_button = CK;	
	guiyihua_button = GYH;
	zuizhi_button = ZZ;
	bizhang_button = BIZ;
}

//基本初始化
void All_Init(void)
{
	ultrasonic_waveinit();
	
	Button_Set();
	
	wireless_uart_init();
	imu660ra_init();
//	dl1a_init();
	ips114_init();
	Key_Init();
	Encoder_Init();
	MyPwm_Init();
    MyADC_Init();
	Buzzer_Init();


	Parameter_Init();
	Channel_Set();
	
	buzzer_bark();
}

//数据采集
void Value_Update(void)
{
	IMU660RA_Kdread();
//	dl1a_get_distance();
	Speed_Measure();
	ADC_Getvalue();
	Channel_Set();
	if(speed_flag == 0)
	{
		Voltage_Protect();
	}
}
