#ifndef __BASEFILE_H
#define __BASEFILE_H

//宏定义规则
#define func_limit_ab(x, a, b)							((x) = ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x))))					//双边限幅
#define NORMALIZATION(x)								((x) = limit*((x) - value_min)/(value_max - value_min))				//归一化
#define ADS(x)											((x) = ((x) < 0 ? (-x) : (x)))										//取绝对值
#define GTSD(x)											(((x)/10)*10)													//除去个位										
#define MYADS(x)										((x) < 0 ? (-x) : (x))
#define Bee(x)											(P77 = (x))

//头文件
#include "headfile.h"
#include "speed.h"
#include "element.h"
#include "display.h"
#include "direction.h"
#include "wave.h"

//编码器引脚及参数
#define Encoder_L				(CTIM0_P34)
#define Encoder_R				(CTIM3_P04)
#define Dir_L					(P35)
#define Dir_R					(P53)
#define transform				(0.00820)							//中断周期10毫秒下

//三电感ADC通道及参数
#define ADC7					(ADC_P14)
#define ADC1					(ADC_P17)
#define ADC4					(ADC_P06)
#define ADC6					(ADC_P16)
#define ADC2					(ADC_P05)

//电机引脚
#define PWM_R0 					(PWMA_CH3P_P64)
#define PWM_R1 					(PWMA_CH4P_P66)
#define PWM_L0 					(PWMA_CH1P_P60)
#define PWM_L1 					(PWMA_CH2P_P62)

//按键引脚(引脚电平的判断用Pxx，不能Px_x)
#define Key5					(P36)
#define Key4					(P71)
#define Key3					(P70)
#define Key2					(P72)
#define Key1					(P73)
//拨码开关
#define Boma1					(P75)
#define Boma2					(P76)
//避障
#define BZ						(P61)

//电压保护引脚
#define protect					(ADC_P15)

//蜂鸣器引脚
#define buzzer					(P7_7)

//外部变量声明
//参数
extern float left_Kp,left_Ki,right_Kp,right_Ki,left_Kd,right_Kd,direction_Kp,direction_Kd,direction_Ki,dirangle_Kd,right_inter_out,left_inter_out;
extern int16 speed_error_limit,speed_inter_limit,speed_out_limit,direction_error_limit,direction_out_limit,direction_inter_limit;
extern float speed_target;
extern int16 K;
extern float speed_left_error[3],speed_right_error[3];
extern float Pa,Pb,Pc,Pp;
//PID输出
extern float left_pid_out,right_pid_out,direction_pid_out,direction_diff_out;
//电机输出
extern float Motor_Lout,Motor_Rout;
//原始处理数据
extern float speed_left,speed_right;
//extern int16 ADC_L1,ADC_R7,ADC_L2,ADC_R6;
extern int16 MYADC[5][5];
//extern float ADC_Lvalue,ADC_Rvalue;
extern float angle_now;
extern float direction_error;
extern float dir_err[5];
extern int16 Encoder1,Encoder2;
extern uint16 voltage;
extern int16 ADC_Max[5],ADC_Min;
//陀螺仪
extern float angle_x[4],angle_x_inter;
//距离
extern float S,distance,ring_out_dis,difficult_dis;
//标志位
extern uint8 diuxian_flag,nojudge_flag,buxian_flag,stop_flag,fold_flag,difficult_flag,ringdir_flag,outring_flag,inring_flag,staring_flag,prering_flag;
extern uint8 direction_flag,speed_flag,angle_flag;
extern uint8 angle_inter_flag;
extern int8 spechange_flag;
extern int8 clear_flag;
extern uint8 indicate_flag;
//并极输出
extern float R_OUT,L_OUT;
extern uint8 Ka,Kb;
extern uint8 bingji_button;
extern uint16 bingji_limit;
extern int16 sub_limit;
//电感差比和输出
extern int16 inductance;
//开关
extern int8 guiyihua_button,chuankou_button,zuizhi_button,buzzer_button;
//参数设置
extern int8 car_mode;
extern uint16 Kp_out,Kd1_out,Kd0_out,Kp_start,Kd1_start,Kd0_start;
extern int16 ADC_Mlimt;
//计数
extern uint16 speed_count;
extern int8 inring_count;
extern uint16 fold_count;
extern int16 diuxian_count2,diuxian_count3,diuxian_count1;
extern uint16 difficult_count,difficult_count0;
//角度积分环
extern float angle_inter_error,angle_inter_last,angle_inter_out,angle_Kp,angle_Kd;
//角度环
extern int16 angle_error_inter;
extern float angle_Ki;
//避障
extern int8 diff_dir;
extern int8 bz_mode;
extern int8 bizhang_button;
extern float bz_dis,run_dis;
extern int8 bz_ci;
extern int8 bz_count;
extern uint16 wave_limit;
extern uint8 dis_limit;
extern int8 agl1,agl2,agl1_limit,agl2_limit;
extern int16 agl2_Kp,agl2_Kd;
extern int8 error_limit;
//折线
extern float Kr,Kl;
extern uint8 speed_downtime;
extern uint8 fold_button;
//拨码开关
extern int8 page_flag;
extern int8 Go;


void All_Init(void);
void Value_Update(void);



#endif
