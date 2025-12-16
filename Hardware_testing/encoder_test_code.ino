
#include <Encoder.h>
#define EA_r D1   //Encoder pin1 of right motor
#define EB_r D6   //Encoder pin2 of right motor
#define EA_l D10  //Encoder pin1 of left motor
#define EB_l D7   //Encoder pin2 of left motor
#define en1 D2  //pwm for left motor 
#define en2 D9  //pwm for right motor
#define ph1 D3  //Dir for left motor
#define ph2 D8  //Dir for right motor

Encoder DC_Encoder_r(EA_r, EB_r);  //Right motor Encoder
Encoder DC_Encoder_l(EA_l, EB_l);  //Left motor Encoder

//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
    
  pinMode(ph1, OUTPUT);
  pinMode(ph2, OUTPUT);
  ledcAttachChannel(en1, 5000, 8, 1);
  ledcAttachChannel(en2, 5000, 8, 2);
  ledcWrite(en1, 0);
  ledcWrite(en2, 0);

  digitalWrite(ph1, LOW);
  digitalWrite(ph2, LOW);

  Serial.println("Basic Encoder Test:");
}

long old_right_Position  = -999;
long old_left_Position  = -999;

void loop() {

  long right_Position = DC_Encoder_r.read();
  long left_Position = DC_Encoder_l.read();
  if (right_Position != old_right_Position) {
   old_right_Position = right_Position;
    Serial.println(right_Position);
  }
    if (left_Position != old_left_Position) {
   old_left_Position = left_Position;
    Serial.println(left_Position);
  }
}
