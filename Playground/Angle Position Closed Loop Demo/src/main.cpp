#include <Arduino.h>

#include <Wire.h>

#define TMAG5273_ADDR 0x35  // 7-bit I2C 设备地址
#define SCL_PIN 22          // ESP32 SCL
#define SDA_PIN 21          // ESP32 SDA

// TMAG5273 相关寄存器地址
#define TMAG5273_REG_DEVICE_ID      0x0D  // 设备 ID
#define TMAG5273_REG_ANGLE_MSB      0x19  // 角度高字节
#define TMAG5273_REG_ANGLE_LSB      0x1A  // 角度低字节
#define TMAG5273_REG_X_MSB          0x12  // X 轴高字节
#define TMAG5273_REG_X_LSB          0x13  // X 轴低字节

// I2C 读取 1 字节寄存器
uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(TMAG5273_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(TMAG5273_ADDR, 1);
    return Wire.available() ? Wire.read() : 0;
}

// 读取 12-bit 数据（MSB + LSB）
int16_t readRegister12(uint8_t msbReg, uint8_t lsbReg) {
    Wire.beginTransmission(TMAG5273_ADDR);
    Wire.write(msbReg);
    Wire.endTransmission(false);
    Wire.requestFrom(TMAG5273_ADDR, 2);

    if (Wire.available() < 2) return 0;

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return (msb << 4) | (lsb >> 4);  // 12-bit 数据
}

// 读取设备 ID
void readDeviceID() {
    uint8_t device_id = readRegister(TMAG5273_REG_DEVICE_ID);
    Serial.print("TMAG5273 Device ID: 0x");
    Serial.println(device_id, HEX);
}

// 读取角度数据（0° ~ 360°）
float readAngle() {
    int16_t raw_angle = readRegister12(TMAG5273_REG_ANGLE_MSB, TMAG5273_REG_ANGLE_LSB);
    return raw_angle * 360.0 / 4096.0;  // 转换为角度
}

// 读取 X 轴磁场数据
int16_t readXAxis() {
    return readRegister12(TMAG5273_REG_X_MSB, TMAG5273_REG_X_LSB);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    readDeviceID();  // 读取设备 ID
}

void loop() {
    int16_t x_axis = readXAxis();
    float angle = readAngle();

    Serial.print("X Axis: ");
    Serial.print(x_axis);
    Serial.print(" | Angle: ");
    Serial.print(angle, 2);
    Serial.println("°");

    delay(500);
}
