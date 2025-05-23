#include "FOC.h"

#define MOTOR_POLE_PAIR 7
#define _3PI_2 4.71238898038f
#define WIND_UP 12.0f
#define _SQRT3_2 0.866025403784438f


// 三对互补引脚：U相 (UH,UL)、V相 (VH,VL)、W相 (WH,WL)
int UH = 16, UL = 17;
int VH = 18, VL = 23;
int WH = 19, WL = 33;

// 电源电压 (用于计算占空比)
float voltage_power_supply = 5.0;

// 一些与开环控制相关的变量
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;
float dc_a = 0, dc_b = 0, dc_c = 0;

//**********************闭环控制****************************** */
int DIR = -1;
float Kp = 1.1;
float Ki = 1.4;

extern TMAG5273 Tsensor; // Initialize hall-effect sensor;

// 死区时长(微秒)。你可根据需求调整
static const uint32_t DEADTIME_RISING  = 10;  // 上升沿死区
static const uint32_t DEADTIME_FALLING = 10;  // 下降沿死区

void initPWM(){
    // --- 1) 绑定 MCPWM 到对应引脚 ---
   // U相：对应 MCPWM_TIMER_0 -> MCPWM0A / MCPWM0B
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, UH);  // 高侧(UH)
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, UL);  // 低侧(UL)
  
   // V相：对应 MCPWM_TIMER_1 -> MCPWM1A / MCPWM1B
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, VH);  // 高侧(VH)
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, VL);  // 低侧(VL)
  
   // W相：对应 MCPWM_TIMER_2 -> MCPWM2A / MCPWM2B
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, WH);  // 高侧(WH)
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, WL);  // 低侧(WL)
  
  
   // --- 2) 设置 MCPWM 的频率、占空比等 ---
   //    这里给 3 个定时器使用相同的配置；你可按需调整频率或其它参数。
   //    占空比起始值为 50%，计数方式为向上，Duty模式为 Active High。
   mcpwm_config_t pwm_config;
   pwm_config.frequency = 20000;       // PWM频率
   pwm_config.cmpr_a = 50.0;          // 通道A初始占空比(%)
   pwm_config.cmpr_b = 50.0;          // 通道B初始占空比(%)
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active High
  
   // 初始化 U相 (Timer0)
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
   // 将 B 通道设为互补(Active Low)
   mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
  
   // 初始化 V相 (Timer1)
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
   mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
  
   // 初始化 W相 (Timer2)
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
   mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
  
  
   // --- 3) 启用硬件死区 ---
   // 在互补波形切换时插入 1us / 1us 的死区(仅示例，可根据需要调整)
   // mode 使用 MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
   mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                         MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                         DEADTIME_RISING,   // 上升沿死区
                         DEADTIME_FALLING); // 下降沿死区
  
   mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,
                         MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                         DEADTIME_RISING,
                         DEADTIME_FALLING);
  
   mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2,
                         MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                         DEADTIME_RISING,
                         DEADTIME_FALLING);
  
   // 说明：MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE表示A通道为主动高，B通道互补，插入死区
   // 如果你需要其它极性，也可用 MCPWM_ACTIVE_LOW_COMPLIMENT_MODE 等模式
  
  }
  
  
  // (仅把 LEDC 调用替换成 MCPWM 调用，尽量少改动)
  
  // 计算并输出 3 相PWM (Ua, Ub, Uc)
  void setPwm(float Ua, float Ub, float Uc) {
    // 1) 限制占空比在 0~1
    float dc_a = constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
    float dc_b = constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
    float dc_c = constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
  
    // 2) MCPWM 的占空比设置范围是 (0~100.0)
    float dutyA = dc_a * 100.0f;
    float dutyB = dc_b * 100.0f;
    float dutyC = dc_c * 100.0f;
  
    // U相 (Timer0) -> A(高侧), B(低侧=互补)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyA);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutyA);
  
    // V相 (Timer1)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyB);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, dutyB);
  
    // W相 (Timer2)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dutyC);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, dutyC);
  }
  
  // 计算三相的 (Ua, Ub, Uc) 并调用 setPwm
  void setPhaseVoltage(float Uq, float Ud, float angle_el) {
    // 归一化电角度
    angle_el = fmod(angle_el + zero_electric_angle, 2*PI);
    if(angle_el < 0) angle_el += 2*PI;
  
    // 逆帕克变换
    Ualpha = -Uq * sin(angle_el);
    Ubeta  =  Uq * cos(angle_el);
  
    // 克拉克逆变换
    Ua = Ualpha + voltage_power_supply/2;
    Ub = (_SQRT3_2*Ubeta - 0.5f*Ualpha) + voltage_power_supply/2;
    Uc = (-Ualpha*0.5f - _SQRT3_2*Ubeta) + voltage_power_supply/2;


    setPwm(Ua, Ub, Uc);
 
  }
  
  
  float _electricalAngle() {
    // return  _normalizeAngle((float)(DIR *  MOTOR_POLE_PAIR) * Tsensor.getAngleResult() - zero_electric_angle);
    return  _normalizeAngle((float)(DIR *  MOTOR_POLE_PAIR) * Tsensor.getAngleResult()-zero_electric_angle);
  }
  
  float _normalizeAngle(float angle){
    float a = fmod(angle, 2*PI) ;
    return a >= 0 ? a : (a + 2*PI);
  } 
  // float serialReceiveUserCommand() {
    
  //   while (Serial.available()) {
  //     // get the new byte:
  //       Ki = Serial.parseFloat();
       
  //           Serial.print("Ki = ");
  //           Serial.println(Ki);

  //     }
  //     return Ki;
  //   }
    
    void cali_zero_electric_angle(){
        setPhaseVoltage(0.5*voltage_power_supply, 0,_3PI_2);
        delay(500);
        zero_electric_angle=_electricalAngle();
        setPhaseVoltage(0, 0,_3PI_2);
    }


  
/**
 * 位置闭环，PI控制
 */
    float pos_closedLoop(float motor_target) {
      unsigned long now_us = micros();
      float Ts = (now_us - open_loop_timestamp) * 1e-6f;
      if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f; // 防止时间异常
  
      // 1) 记录当前角度 (带滤波)
      static float filtered_angle = 0;
      float alpha = 0.5; // 滤波系数 (0~1)
      filtered_angle = alpha * filtered_angle + (1 - alpha) * Tsensor.getAngleResult();
  
      // 2) 计算位置误差 (目标 - 当前)
      float err = motor_target - filtered_angle;
  
      // 3) Tustin离散积分法
      static float integral_err = 0.0f;
      static float old_err = 0.0f;
      integral_err += 0.5f * Ts * (err + old_err);
  
      // 4) 防止积分失控 (Anti-Windup)
      if (integral_err > WIND_UP)  integral_err = WIND_UP;
      if (integral_err < -WIND_UP) integral_err = -WIND_UP;
      // 更新 old_err
      old_err = err;
  
      // 5) 计算输出 (P + I)
      float raw_target_Uq = Kp * err + Ki * integral_err;
      raw_target_Uq = constrain(raw_target_Uq, -(voltage_power_supply / 2), (voltage_power_supply / 2));
  
      if ((fabs(err) < 0.1)) integral_err = 0;
      if (fabs(err) < 0.05) return -1; // 误差小于一定阈值就不调整电压
      
      // 7) 设置相电压
      setPhaseVoltage(raw_target_Uq, 0, _electricalAngle());
  
      // 8) 更新时间戳和上次输出
      open_loop_timestamp = now_us;

      return raw_target_Uq;
  }
  