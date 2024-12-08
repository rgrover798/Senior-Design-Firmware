#include <Servo.h>

#define PWM_Tail 9 // --> pin 15 (PB1)

Servo tailServo;

void setup() {
  pinMode(PWM_Tail, OUTPUT);
  tailServo.attach(PWM_Tail);  // Attach tail servo to its PIN
}


void loop() {
  tailServo.writeMicroseconds(1500);
  delay(1000);  // controls speed between switches
  tailServo.writeMicroseconds(2000); // controls angle
  delay(1000);
}