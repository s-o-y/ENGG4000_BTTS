#include <Servo.h>

int pwmPin = 10;
Servo motor;

void setup() {
  int setpoint = 1321; //setpoint = t_on for PWM signal
  motor.attach(pwmPin);
  Serial.begin(9600);
  delay(6000); //time motor waits before spinning
  motor.writeMicroseconds(setpoint);
  delay(4000); //time motor spins for
  motor.writeMicroseconds(1500); //stopping motor
  
}
