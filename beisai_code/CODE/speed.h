#ifndef __SPEED_H
#define __SPEED_H

void Speed_Measure(void);
void Speed_Control(void);
void Pwm_Out(PWMCH_enum pwmch1,PWMCH_enum pwmch0,int32 Out);
int16 AverageEncoder(int16 carspeed);



#endif
