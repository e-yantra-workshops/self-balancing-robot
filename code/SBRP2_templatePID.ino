#include "Wire.h"
#include "MPU6050_light.h"

// Define MPU6050 object
MPU6050 mpu(Wire);

// Variables for tilt angle
float tilt=0.0, tilt_old=0.0, tilt_dot=0.0, integral_tilt=0.0;

// Variable for wheel angle
int wheel_pulse_count = 0;

// final output for wheel velocity
int control_output = 0.0;

// Tilt PID gains
float kp_tilt =
float ki_tilt =
float kd_tilt =


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
    //---------(pre-scaler * timesteps) / frequency -----------//
    TCNT1H = 0xA2;  // Counter higher 8 bit value 162
    TCNT1L = 0x3F;  // Counter lower 8 bit value 63
    mpu.update();   // I2C needs global interrupt to be enabled
    cli();
    
    
    feedback();
    control_eqn();
}


void feedback(){
    tilt = mpu.getAngleY();
    // fill-in
 
    wheel_angle = // for every 180 degree there is 350 pulse counts or 1 full rotation = 700 pulses

}

void control_eqn(){

  // fill-in
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
}

// Loop function
void loop() {  

     
}

// ISR for motor encoder
void motor_encoder() {                                  
    if (digitalRead(encodPinBR) == HIGH) {
         wheel_pulse_count = wheel_pulse_count + 1;
    } else {
         wheel_pulse_count = wheel_pulse_count - 1;
    }
}
