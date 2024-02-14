

// Define encoder pins
#define encodPinA 2  // Encoder A pin
#define encodPinB 3  // Encoder B pin

// Variable for wheel angle
int wheel_pulse_count = 0, prev_count=0;

// Setup function
void setup() {
    Serial.begin(115200);
    
    //------Encoder HW Interrupt setup-----//
    attachInterrupt(digitalPinToInterrupt(encodPinA), mot_rencoder, RISING);  
}


// Loop function
void loop() {  
    
    if(wheel_pulse_count != prev_count)
    Serial.println(wheel_pulse_count);
     
}

// ISR for motor encoder
void mot_rencoder() {                                  
    if (digitalRead(encodPinB) == HIGH) {
         wheel_pulse_count = wheel_pulse_count + 1;
    } else {
         wheel_pulse_count = wheel_pulse_count - 1;
    }
}
