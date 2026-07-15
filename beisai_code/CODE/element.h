#ifndef __ELEMENT_H
#define __ELEMENT_H

//开关
#define ON														(1)
#define OFF														(0)
#define bark_time												(20)
//------------------------------目标值-------------------------------------//
//设定目标速度
#define target_speed											(2.4-0.1)
//------------------------------固定值-------------------------------------//
//停车
#define STOP													(35)
//赛道长
#define road_dis												(32)
//避障
#define DIS														(3)
//归一化系数
#define K_guiyi													(2750)
#define ADC_SLm													(2500)
#define ADC_SRm													(2500)
#define ADC_MLm													(2000)
#define ADC_MRm													(2000)
//------------------------------PID参数-----------------------------------//
//方向环PD参数
#define direction_P												(320)//200
#define direction_I												(20)
#define direction_D												(50)//100
#define dirangle_D												(275)//120
//角度积分环
#define angle_P													(350)//100
#define angle_D													(150)//120
//角度环
#define angle_I													(0.5)
//差比和差加权系数
#define inductance_b											(12)



#define inductance_a											(8)
#define inductance_c											(2)
#define inductance_p											(100)


//串口
#define CK														(0)
//归一化
#define GYH														(1)
//找电感最值
#define ZZ														(0)
//避障
#define BIZ														(0)
#define BZCI													(1)
#define DIR														(1)//左发车:1内打，-1外打
//------------------------------速度环------------------------------------//
//左速度环PI参数
#define left_speed_P											(4000)//4800
#define left_speed_I											(500)//250
#define left_speed_D											(2000)
//右速度环PI参数
#define right_speed_P											(4000)//4000
#define right_speed_I											(500)//500
#define right_speed_D											(2000)
//------------------------------限幅--------------------------------------//
//速度环限幅参数
#define limit_speed_error										(10.0)
#define limit_speed_inter										(15)
#define limit_speed_out											(4000)
//方向环限幅参数
#define limit_direction_error									(200)//150
#define limit_direction_inter									(400)
#define limit_direction_out										(8000)//12000
//------------------------------串口--------------------------------------//
//串口发送数据设置
#define channel1												(MYADC[0][0])//Motor_Lout/1000//MYADC[0][0]
#define channel2												(MYADC[2][0])//Motor_Rout/1000//MYADC[2][0]
#define channel3												(MYADC[1][0])//speed_right//MYADC[1][0]
#define channel4												(MYADC[3][0])//speed_left//MYADC[3][0]
#define channel5												(direction_error/10)//direction_error/10//
#define channel6												(wave_dis)//speed_target
#define channelsum												(6)
//------------------------------开关--------------------------------------//





void PID_Count(void);
void curve(void);
void Element_Run(void);
void buzzer_bark();
void Motorout_Control(void);
void rest(void);


	


#endif
