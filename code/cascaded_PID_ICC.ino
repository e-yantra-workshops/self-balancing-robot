#include <Arduino_JSON.h>
#include <Preferences.h>
#include <Arduino.h>
#include <Encoder.h>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

//Refer circuit schematics.
#define ph1 D3  //Dir for left motor
#define en1 D2  //pwm for left motor    (if we see the bot face to face and antenna part upside)
#define en2 D9  //pwm for right motor
#define ph2 D8  //Dir for right motor

#define EA_r D1   //Encoder pin1 of right motor
#define EB_r D6   //Encoder pin2 of right motor
#define EA_l D10  //Encoder pin1 of left motor
#define EB_l D7   //Encoder pin2 of left motor


bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;   
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double input_pitch, Output, setpoint_for_pitch;
double Input_wheel, Output_pos, u_wheel, setpoint_for_wheel;                       //PID variables
float Output2, lastTime, ierror_pitch, u, last_input_wheel;
float ierror_wheel, last_input_pitch,lastTime2;

float setangle=0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float pitch;

//////////////////////////////////////////////////////////////////

int freq1 = 5000;
int reso1 = 8;
float wheel_r, wheel_l ; 
long pos_r;
long pos_l;
float u_r = 0;
float u_l = 0;

////////////////////////////////////////////////////////////////////////////////////////////////
Encoder DC_Encoder_r(EA_r, EB_r);  //Right motor Encoder
Encoder DC_Encoder_l(EA_l, EB_l);  //Left motor Encoder


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
  Serial.print("start");
  
  /////////////////////////////////////////////////////////////////////////////////////////////////

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
  //supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

double output=0;


void loop() {

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
    }          

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);

    if(timeChange>=10)
    {
    //////////////////////////// THE CASCADED PID ////////////////
    pitch = (ypr[1]);                            // get pitch
    pos_r = DC_Encoder_r.read();                 // get encoder reading
    wheel_r = pos_r * 0.0044;                    // convert to wheel angle

    // define
    double error_wheel, error_pitch, derror_wheel, derror_pitch;
    // available global variable: Line no. 42

    float pitchKP = 0; float pitchKI = 0;  float pitchKD = 0;
    float wheelKP = 0;  float wheelKI = 0; float wheelKD = 0.; 

    setpoint_for_wheel = 0;
    Input_wheel = wheel_r; 

    error_wheel =  ;  // calculate error for position 
    derror_wheel = ; // calculate finite difference error (use: last_input_wheel for previous value)
    ierror_wheel = ; // calculate sum error

    u_wheel = ; // Enter PID logic  Kp*error + Ki*error + Kd*error

    input_pitch = pitch;       
    setpoint_for_pitch = ;  // if cascaded use outer loop value as setpoint for inner loop

    error_pitch =  ;  // calculate error 
    derror_pitch = ; // calculate finite difference error (use: last_input_pitch for previous value)
    ierror_pitch = ; // calculate sum error

    u = ;  // Enter PID logic  Kp*error + Ki*error + Kd*error

    // copy vars for next iteration
    last_input_pitch = error_pitch;                         
    last_input_wheel = error_wheel;
    lastTime = now;
    //////////////////////////////////////////////////////////////
    }

    u_r =  u*100;// + u_wheel;        //If no button is pressed then the Computed pid output will be sent as it is to the motor.
    u_l =  u*100;// + u_wheel; 
  
  
    float mapout_r = mapf(fabs(u_r), 0, 255, 15, 255);      //The output of pid is maped for motors pwm.
    float mapout_l = mapf(fabs(u_l), 0, 255, 15, 255);
    float final_pwm_r = constrain(mapout_r, 0, 255);        //Pwm is contrained till 255 strictly.
    float final_pwm_l = constrain(mapout_l, 0, 255);
    motor_r(final_pwm_r, u_r);                            //The pwm and the Direction is passes to Right motor function.
    motor_l(final_pwm_l, u_l);                            //The pwm and the Direction is passes to Left motor function.

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
  } else if (target_radps == 0) {

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
  } else if (target_radps1 == 0) {

    ledcWrite(en1, 0);               //if passed 0 as direction then pwm for the motors will be also 0.
  }
}

