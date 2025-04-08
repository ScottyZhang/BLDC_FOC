#include "INA240.h"

INA240::INA240(){}

void INA240::INA240_init(){
    // 配置引脚为输入（ESP32这些引脚默认是输入）
    pinMode(U_READ_PIN, INPUT);
    pinMode(V_READ_PIN, INPUT);
    pinMode(W_READ_PIN, INPUT);

    // 设置ADC分辨率为12位（0~4095）
    analogReadResolution(12);

    // 设置ADC衰减（适应0~3.3V输入范围）
    analogSetAttenuation(ADC_11db);

    calibrateOffsets();
}

// 通用的读取函数（内部用）
static float readCurrent(int pin) {
    int adcValue = analogRead(pin);
    float voltage = (adcValue / 4095.0f) * V_REF;  // 转换为电压
    float current = voltage / (SHUNT_RES * INA240_GAIN);           // 计算电流（假设增益=1）
    return current;
}

float INA240::getUCurrentValue() {
    return (readCurrent(U_READ_PIN) - _offset_iu);
}

float INA240::getVCurrentValue() {
    return (readCurrent(V_READ_PIN) - _offset_iv) * -1;
}

float INA240::getWCurrentValue() {
    return (readCurrent(W_READ_PIN) - _offset_iw) * -1;
}

void INA240::calibrateOffsets(){
    const int calibration_rounds = 1000;
    _offset_iu = 0;
    _offset_iv = 0;
    _offset_iw = 0;

    for(int i = 0; i < calibration_rounds; i++){
        _offset_iu += readCurrent(U_READ_PIN);
        _offset_iv += readCurrent(V_READ_PIN);
        _offset_iw += readCurrent(W_READ_PIN);
        delay(1);
    }

    _offset_iu = (float)_offset_iu/calibration_rounds;
    _offset_iv = (float)_offset_iv/calibration_rounds;
    _offset_iw = (float)_offset_iw/calibration_rounds;

}