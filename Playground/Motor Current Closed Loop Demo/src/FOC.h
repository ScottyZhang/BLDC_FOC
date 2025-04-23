#ifndef FOC_h
#define FOC_h

#include "Arduino.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "TMAG5273.h"


// --- 辅助函数声明 ---
float _electricalAngle();
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

void initPWM();
void cali_zero_electric_angle();
float pos_closedLoop(float motor_target);
float vel_closedLoop(float motor_target_vel);
void vel_UserCommand(float *target_vel);

void start_smart_knob(float damping_coefficient);


#endif
