#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];
float pitch,roll,yaw;

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

#define ph1 D3  //Dir for left motor
#define en1 D2  //pwm for left motor    (if we see the bot face to face and antenna part upside)
#define en2 D9  //pwm for right motor
#define ph2 D8  //Dir for right motor

#define EA_r D1   //Encoder pin1 of right motor
#define EB_r D6   //Encoder pin2 of right motor
#define EA_l D10  //Encoder pin1 of left motor
#define EB_l D7   //Encoder pin2 of left motor

int freq1 = 5000;
float u_r = 0;
float u_l = 0;
int reso1 = 8;

void setup() {
  pinMode(ph1, OUTPUT);
  pinMode(ph2, OUTPUT);
  
  ledcAttachChannel(en1, freq1, reso1, 1);
  ledcAttachChannel(en2, freq1, reso1, 2);
  ledcWrite(en1, 0);
  ledcWrite(en2, 0);

  digitalWrite(ph1, LOW);
  digitalWrite(ph2, LOW);
  motor_r(0, u_r);
  motor_l(0, u_l);
  delay(1500);

  Serial.begin(115200);

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("DMP ready!"));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    #endif  // put your main code here, to run repeatedly:
    pitch = ypr[1] * 180 / M_PI;
    roll =  ypr[2] * 180 / M_PI;
    yaw = ypr[0] * 180 / M_PI;
}
     
    // Serial.print(pitch);
    // Serial.print(",");
    // Serial.print(roll);
    // Serial.print(",");
    // Serial.print(yaw);
    // Serial.println(" ");


    if (pitch<= 0){
    u_r = -255;
    u_l = -255;
    }
    else{
    u_r = 255;
    u_l = 255;
    }
    float mapout_r = mapf(fabs(u_r), 0, 255, 15, 255);      //The output of pid is maped for motors pwm.
    float mapout_l = mapf(fabs(u_l), 0, 255, 15, 255);
    float final_pwm_r = constrain(mapout_r, 0, 255);        //Pwm is contrained till 255 strictly.
    float final_pwm_l = constrain(mapout_l, 0, 255);
    motor_r(final_pwm_r, u_r);                            //The pwm and the Direction is passes to Right motor function.
    motor_l(final_pwm_l, u_l);  
    // motor_r(0, 255);                            //The pwm and the Direction is passes to Right motor function.
    // motor_l(0, 255);

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {          //This function is similar to map function, only it can process decimal values.
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void motor_r(int pwm, float target_radps) {        //Right motor function takes input as PWM and Direction.

  if (target_radps < 0) {
    digitalWrite(ph2, LOW);              //Rotate anticlkwise if the direction is negative
    ledcWrite(en2, pwm);
    target_radps = -target_radps;
  } else if (target_radps > 0) {         //Rotate clkwise if the direction is positive
    digitalWrite(ph2, HIGH);
    ledcWrite(en2, pwm);
    target_radps = target_radps;
  } else if (target_radps = 0) {

    ledcWrite(en2, 0);                 //if passed 0 as direction then pwm for the motors will be also 0.
  }
}


void motor_l(int pwm1, float target_radps1) {       // Same as right motor Function, only the PWM and Direction pins are changed.

  if (target_radps1 < 0) {

    digitalWrite(ph1, HIGH);                    
    ledcWrite(en1, pwm1);                         //Rotate anticlkwise if the direction is negative
    target_radps1 = -target_radps1;
  } else if (target_radps1 > 0) {                   

    digitalWrite(ph1, LOW);                   //Rotate clkwise if the direction is positive
    ledcWrite(en1, pwm1);
    target_radps1 = target_radps1;
  } else if (target_radps1 = 0) {

    ledcWrite(en1, 0);               //if passed 0 as direction then pwm for the motors will be also 0.
  }
}
