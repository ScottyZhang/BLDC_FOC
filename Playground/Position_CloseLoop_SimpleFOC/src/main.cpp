
#include <SimpleFOC.h>
#include "TMAG5273.h"

// 定义BLDC电机（极对数）
BLDCMotor motor = BLDCMotor(7);  // 7对极
BLDCDriver6PWM driver = BLDCDriver6PWM(16, 17, 18, 23, 19, 33);

// 创建 TMAG5273 传感器对象
TMAG5273 tmag5273;
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

// 自定义 TMAG5273 传感器类
class TMAG5273Sensor : public Sensor {
public:
  void init() override {
      Wire.begin();
      if (!tmag5273.begin(i2cAddress, Wire)) {
          Serial.println("TMAG5273 初始化失败！");
      } else {
          Serial.println("TMAG5273 初始化成功！");
      }
  }

  float getSensorAngle() override {
      return tmag5273.getAngleResult();  // 直接获取角度
  }
};

// 创建 TMAG5273 传感器对象
TMAG5273Sensor sensor;

// 目标角度
float target_angle = 1.5;

void setup() {
    Serial.begin(115200);
    Wire.begin();  // 初始化 I2C

    // 初始化编码器
    sensor.init();
    motor.linkSensor(&sensor);

    // 配置驱动器
    driver.voltage_power_supply = 5;
    driver.init();
    motor.linkDriver(&driver);

    // 设置 FOC 模式
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::angle;  // 位置控制

    // **调整 PID 控制参数**
    motor.P_angle.P = 1.0;  // 位置 P 增益
    motor.P_angle.I = 0.1;  // 位置 I 增益 (可选)
    
    motor.P_angle.output_ramp = 1000;  // 限制控制输出变化率 (防止突变)
    motor.voltage_limit = 5;   // 最大电压
    motor.velocity_limit = 50; // 最大速度
    motor.LPF_velocity.Tf = 0.05; // 低通滤波

    motor.useMonitoring(Serial); // 串口监视

    // 初始化 FOC
    motor.init();
    motor.initFOC();

    Serial.println("ESP32 BLDC 位置闭环控制初始化完成");
}

void loop() {
    // 运行 FOC
    motor.loopFOC();

    // 计算位置控制量 (PID)
    float control_output = motor.P_angle(target_angle - motor.shaftAngle());

    // **设定目标角度**
    motor.move(target_angle);

    // **打印调试信息**
    Serial.print("目标角度: ");
    Serial.print(target_angle);
    Serial.print(" | 当前角度: ");
    Serial.print(motor.shaftAngle());
    Serial.print(" | 控制输出: ");
    Serial.println(control_output);

    delay(10);
}