#ifndef __WAVE_H
#define __WAVE_H

extern int8 trigger_flag;
extern uint16 count[3];
extern uint16 dis_count,get_count;
extern float wave_dis;



void ultrasonic_waveinit(void);
void trigger_mode(int8 mode);
void waveIT_cmd(int8 cmd);
void wave_trigger(void);
void wave_exti(void);



#endif
