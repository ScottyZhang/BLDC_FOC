#include "Wire.h"
#include "TMAG5273.h"
#include "Arduino.h"
#include "FOC.h"



#define SCL_PIN 22          // ESP32 SCL
#define SDA_PIN 21          // ESP32 SDA

TMAG5273 Tsensor; // Initialize hall-effect sensor

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

  
  Tsensor.setConvAvg(TMAG5273_X16_CONVERSION);


  // Choose new angle to calculate from
  // Can calculate angles between XYX, YXY, YZY, and XZX
  Tsensor.setMagneticChannel(TMAG5273_YXY_ENABLE);

  // Enable the angle calculation register
  // Can choose between XY, YZ, or XZ priority
  Tsensor.setAngleEn(TMAG5273_XY_ANGLE_CALCULATION);
  Tsensor.setGlitchFilter(TMAG5273_GLITCH_ON);
  Tsensor.setLowPower(TMAG5273_LOW_NOISE_MODE);

  cali_zero_electric_angle();
  initPWM();



}


float velocity = 0.0;


void loop() 
{
  Tsensor.Sensor_update();
  velocity = Tsensor.getVelocityResult();
  Serial.print("Velocity: ");
  Serial.println(velocity);
}