/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

long timer = 0;
float gyro_angle = 0;
float fltered_angle =0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  

  if(millis() - timer > 100){ // print data every second
    mpu.update();

    // Serial.print("\tGyroAngleZ:");Serial.println(mpu.getAngleZ());
  
    // Serial.print(F("\tACCAngleX:"));Serial.println(mpu.getAccAngleX());

    Serial.print("\tFilteredY:"); Serial.println(mpu.getAngleY());
    timer = millis();
  }

}
