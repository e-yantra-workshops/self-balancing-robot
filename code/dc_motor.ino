
// Define motor pins
#define InL1 4    // INA motor pin
#define PWML 5    // PWM motor pin
#define InL2 7    // INB motor pin
#define InR1 10   // INA motor pin
#define PWMR 6    // PWM motor pin
#define InR2 12   // INB motor pin 


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

// Setup function
void setup() {
    
    //---------Motor setup--------//
    Serial.println("Begin Device initialization:\n");
    motor_init();
    Serial.println("DC Motor initialized\n");
    
}

// Loop function
void loop() {  
  motor_control_R(100);
  motor_control_R(100);
  delay(2000);
  motor_control_R(100);
  motor_control_R(-100);
  delay(2000);
  motor_control_R(-100);
  motor_control_R(100);
  delay(2000);
  motor_control_R(-100);
  motor_control_R(-100);
  delay(2000);
}
