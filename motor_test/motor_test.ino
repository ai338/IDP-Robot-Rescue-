#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
void setup() {
  AFMS.begin();
  myMotor->setSpeed(255);
  myMotor->run(FORWARD);
  delay(10000);
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);
}

void loop() {
  // put your main code here, to run repeatedly:

}
