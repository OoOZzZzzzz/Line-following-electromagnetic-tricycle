#ifndef __DIRECTION_H
#define __DIRECTION_H

void ADC_Getvalue(void);
void IMU660RA_Kdread(void);
void IMU660RA_Read(void);
void Direction_Control(float target);
void Upright_Control(float target);


void Foldline_Control(int16 angle);
void Angle_Control(void);



#endif
