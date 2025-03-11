/******************************************************************************
    IoT Motor Driver Example - MCPWM 3-phase complementary + Dead Time
    Velocity Open Loop Control
    通过给定期望速度，计算出Uq的大小。之后和一个模拟出来的电角度一起通过“帕克逆变换”得到Ualpha和Ubeta，
    再经过克拉克逆变换得到Ua,Ub和Uc三个相的电压，就可以控制无刷电机的转动了。
    
    ******************************************************************************
    [注意]：“Uq = voltage_power_supply / 3” 只是“固定给电机施加一个与电源电压成固定比例的电压幅值”，
    并没有和给定速度做任何算法关联。在这段演示代码里，它被写死成常数，以保证输出三相电压波形不会饱和或超出供电范围。
******************************************************************************/
#include "Arduino.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// =========== 你的核心变量及函数声明 ===========

// 三对互补引脚：U相 (UH,UL)、V相 (VH,VL)、W相 (WH,WL)
int UH = 16, UL = 17;
int VH = 18, VL = 23;
int WH = 19, WL = 33;

// 电源电压 (假设用于计算占空比)
float voltage_power_supply = 8.0;

// 一些与开环控制相关的变量
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;
float dc_a = 0, dc_b = 0, dc_c = 0;

// 死区时长(微秒)。你可根据需求调整
static const uint32_t DEADTIME_RISING  = 1;  // 上升沿死区
static const uint32_t DEADTIME_FALLING = 1;  // 下降沿死区


// --- 辅助函数声明 ---
float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
float velocityOpenloop(float target_velocity);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

void initPWM();




// =========== setup()：初始化 MCPWM 输出 ===========
void setup() {
  Serial.begin(115200);
  initPWM();
 

  Serial.println("MCPWM (3-phase complementary) + Dead Time Init Finished.");
  delay(3000);
}


// =========== loop()：开环让电机以 5rad/s 旋转 ===========
void loop() {
  float _Uq = velocityOpenloop(40);  // 目标速度(示例)
  Serial.print("Uq = ");
  Serial.println(_Uq);
}

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
 pwm_config.frequency = 20000;       // PWM频率：20kHz示例(可改为5kHz,10kHz等)
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
  Ub = (sqrt(3)*Ubeta - Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha - sqrt(3)*Ubeta)/2 + voltage_power_supply/2;

  setPwm(Ua, Ub, Uc);
}

// 简易开环速度控制示例
float velocityOpenloop(float target_velocity){
  unsigned long now_us = micros();
  float Ts = (now_us - open_loop_timestamp)*1e-6f;
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // 累积计算轴角度(机械角度)
  shaft_angle += target_velocity*Ts;
  shaft_angle = fmod(shaft_angle, 2*PI);
  if(shaft_angle < 0) shaft_angle += 2*PI;

  // 给定一个固定的 Uq (等效驱动电压占电源电压的一部分)
  float Uq = voltage_power_supply/3;

  // 设极对数=8，计算电角度并调用 setPhaseVoltage
  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));

  open_loop_timestamp = now_us;
  return Uq;
}

float _electricalAngle(float shaft_angle, int pole_pairs) {
  return shaft_angle * pole_pairs;
}

float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);
  return a >= 0 ? a : (a + 2*PI);
}
