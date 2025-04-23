#include "Wire.h"
#include "TMAG5273.h"
#include "Arduino.h"
#include "FOC.h"
#include "INA240.h"
#include "lowpassfilter.h"


#define SCL_PIN 22          // ESP32 SCL
#define SDA_PIN 21          // ESP32 SDA

TMAG5273 Tsensor; // Initialize hall-effect sensor
INA240    ISensor; // Initialize current reading sensor

// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

void setup() 
{
  Wire.begin();
  // Start serial communication at 115200 baud
  Serial.begin(115200);  

  delay(200);
  // If begin is successful (0), then start example
  if(Tsensor.begin(i2cAddress, Wire) == 1)
  {
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    while(1); // Runs forever
  }

  
  Tsensor.setConvAvg(TMAG5273_X8_CONVERSION);


  // Choose new angle to calculate from
  // Can calculate angles between XYX, YXY, YZY, and XZX
  Tsensor.setMagneticChannel(TMAG5273_XYX_ENABLE);

  // Enable the angle calculation register
  // Can choose between XY, YZ, or XZ priority
  Tsensor.setAngleEn(TMAG5273_XY_ANGLE_CALCULATION);
  // Tsensor.setGlitchFilter(TMAG5273_GLITCH_ON);
  // Tsensor.setLowPower(TMAG5273_LOW_NOISE_MODE);

  cali_zero_electric_angle();
  initPWM();

  ISensor.INA240_init();



}


float velocity = 0.0;
float target_Uq = 0.0;
float currents[3] = {0,0,0}; //{u,v,w}

extern LowPassFilter filter;

void loop() 
{
  Tsensor.Sensor_update();
  // velocity = Tsensor.getVelocityResult();
  // Serial.print("Velocity: ");
  // Serial.println(velocity);
  // delay(100);

  // target_Uq = vel_closedLoop(velocity);
  // vel_UserCommand(&velocity);

  start_smart_knob(0.2);


  // target_Uq = pos_closedLoop(3.14);

  // float position = Tsensor.getAngleResult();
  // float vel = (Tsensor.getVelocityResult());
  // Serial.print("Angle: ");
  // Serial.print(position);
  // Serial.print(" | ");
  // Serial.print("Velocity: ");
  // Serial.println(vel);
  // delay(100);

  // currents[0] = ISensor.getUCurrentValue();
  // currents[1] = ISensor.getVCurrentValue();
  // currents[2] = ISensor.getWCurrentValue();

  // Serial.print("U = ");
  // Serial.print(currents[0]);
  // Serial.print(" | ");
  // Serial.print("V = ");
  // Serial.print(currents[1]);
  // Serial.print(" | ");
  // Serial.print("W = ");
  // Serial.println(currents[2]);
 
}