#include "Wire.h"
#include "MPU6050_light.h"

// Define MPU6050 object
MPU6050 mpu(Wire);

// Variables for tilt angle
float tilt=0.0, tilt_old=0.0, tilt_dot=0.0,integral_tilt=0.0;

// Variable for wheel angle
int wheel_pulse_count = 0;
float wheel_angle = 0.0, wheel_angle_old = 0.0, wheel_dot = 0.0, integral_wheel =0.0;

// Variables for timing
float dt, tnow, tprev = 0;
bool continuous = false;

// Variables for control
bool PID_or_LQR = true; // true = PID , false = lqr;
int control_output = 0.0;

// Tilt PID gains
float kp_tilt=100;
float ki_tilt=0.0;
float kd_tilt=150;

// Position PID gains
float kp_wheel=0;
float ki_wheel=0;
float kd_wheel=50;

// LQR gains (wheel, tilt, wheel_dot, tilt_dot)
float K[] = { -0.0010466,  -0.3946552,  -0.0049367,  -0.0439378}; 

// Define motor pins
#define InL1 4    // INA motor pin
#define PWML 5    // PWM motor pin
#define InL2 7    // INB motor pin
#define InR1 10   // INA motor pin
#define PWMR 6    // PWM motor pin
#define InR2 12   // INB motor pin 

// Define encoder pins
#define encodPinAR 2  // Encoder A pin
#define encodPinBR 3  // Encoder B pin

// Motor initialization function
void motor_init() {
    // Left motor pins
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, LOW);
    analogWrite(PWML, 0);
    
    // Right motor pins
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
    analogWrite(PWMR, 0);
}

// Motor control function for left motor
void motor_control_L(int pwm) {
    if (pwm < 0) {
        digitalWrite(InL1, HIGH);
        digitalWrite(InL2, LOW);
        pwm = -pwm;
    } else {
        digitalWrite(InL1, LOW);
        digitalWrite(InL2, HIGH);
    }
    analogWrite(PWML, pwm);
}

// Motor control function for right motor
void motor_control_R(int pwm) {
    if (pwm < 0) {
        digitalWrite(InR1, HIGH);
        digitalWrite(InR2, LOW);
        pwm = -pwm;
    } else {
        digitalWrite(InR1, LOW);
        digitalWrite(InR2, HIGH);
    }
    analogWrite(PWMR, pwm);
}

// Timer1 ISR for IMU
void timer1_init() {
    cli();  // Clear global interrupts
    TIMSK1 = 0x01;  // Timer5 overflow interrupt enable
    TCCR1B = 0x00;  // Stop
    TCNT1H = 0xA2;  // Counter higher 8 bit value
    TCNT1L = 0x3F;  // Counter lower 8 bit value
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x02;  // Start Timer, prescaler 8
    sei();   // Enable global interrupts
}

// ISR for Timer1 overflow
ISR(TIMER1_OVF_vect) {
    sei();  
    //----set timer value so that it overflows at every 12ms.---//
    //--------counting is from 0xA23F to 0xFFFF-----------------//
    TCNT1H = 0xA2;  // Counter higher 8 bit value
    TCNT1L = 0x3F;  // Counter lower 8 bit value
    mpu.update();   // I2C needs global interrupt to be enabled
    cli();
    
    
    feedback();
    control_eqn();
}


void feedback(){
    tilt = mpu.getAngleY();
    tilt_dot = tilt - tilt_old;  // Current - Previous
    integral_tilt += tilt;  
    tilt_old = tilt;    
 
    wheel_angle = wheel_pulse_count * 180 / 350;
    wheel_dot = wheel_angle - wheel_angle_old;
    integral_wheel += wheel_angle;
    wheel_angle_old = wheel_angle;

    if(continuous && dt != 0)
    {
      tilt_dot = tilt_dot*1000/dt;
      integral_tilt = integral_tilt*dt/1000;
      wheel_dot = wheel_dot*1000/dt;
      integral_wheel = integral_wheel*dt/1000;
    }  
}

void control_eqn(){
  if(continuous && dt != 0){
    ki_tilt = (ki_tilt)*1000/dt;
    kd_tilt = kd_tilt*dt/1000;
    ki_wheel = ki_wheel*1000/dt;
    kd_wheel = kd_wheel*dt/1000;
  }

  if(PID_or_LQR){
    control_output = -1*constrain(kp_tilt*tilt + kd_tilt*tilt_dot + ki_tilt*integral_tilt - (kd_wheel*wheel_dot + kp_wheel*wheel_angle +ki_wheel*integral_wheel) , -255, 255);
  }
  else{
    if(continuous)
      control_output = 100*constrain(-K[0]*wheel_angle + K[1]*tilt - K[2]*wheel_dot + K[3]*tilt_dot, -255, 255);
    else
      control_output = 100*constrain(-K[0]*wheel_angle + K[1]*tilt - K[2]*wheel_dot + K[3]*tilt_dot, -255, 255);
  }

  motor_control_R(control_output);
  motor_control_L(control_output); 
}

// Setup function
void setup() {
    Serial.begin(115200);
    
    //-----------MPU6050 setup----------//
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {}  // Stop everything if could not connect to MPU6050
    Serial.println("MPU begin done!\n");
    delay(1000);  // Required for offset calculation of IMU
    mpu.calcOffsets(true, true);

    //---------Motor setup--------//
    Serial.println("Begin Device initialization:\n");
    motor_init();
    Serial.println("DC Motor initialized\n");
    
    //--------Timer(ISR) Setup--------//
   timer1_init();
    Serial.println("Timer initialized\n");
    
    //------Encoder HW Interrupt setup-----//
    attachInterrupt(digitalPinToInterrupt(2), mot_rencoder, RISING);  
    tprev = millis(); 
}

// Loop function
void loop() {  
    tnow = millis();
    dt = (tnow - tprev);

    //Uncomment if want to use void loop insted of ISR
    // mpu.update();
    // if(continuous){  
    //   //dt = dt/1000; //convert to seconds
    //   feedback();
    //   control_eqn();
    //   tprev = tnow;
    // }
    // else{
    //   if(dt>12){
    //     feedback();
    //     control_eqn();
    //     tprev = tnow;
    //   }
    // }
    
    // Serial.print(control_output);
    // Serial.print("\t");
    // Serial.print(tilt);
    // Serial.print("\t");
    // Serial.print(tilt_dot);
    // Serial.print("\t");
    // Serial.print(kp_tilt);
    // Serial.print("\t");
    // Serial.print(kd_tilt);
    // Serial.print("\t");
    // Serial.print(ki_tilt);
    // Serial.print("\t");
    // Serial.println(dt);
     
}

// ISR for motor encoder
void mot_rencoder() {                                  
    if (digitalRead(encodPinBR) == HIGH) {
         wheel_pulse_count = wheel_pulse_count + 1;
    } else {
         wheel_pulse_count = wheel_pulse_count - 1;
    }
}
