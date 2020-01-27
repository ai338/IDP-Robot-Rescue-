#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);

const int MOTOR_SPEED = 200;
int led_phase = 0;
const int START_SWITCH = 12;
int t = 30;

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(START_SWITCH, INPUT);

}

void motor(int l, int r, int t) {
  //set motors running for time t while flashing LED
  left_motor->setSpeed(abs(l));
  right_motor->setSpeed(abs(r));
  left_motor->run(l < 0 ? BACKWARD : FORWARD);
  right_motor->run(r < 0 ? BACKWARD : FORWARD);
  for (int ds = 0; ds < t; ds++) {
    led_phase++;
    led_phase %= 10;
    digitalWrite(LED_BUILTIN, led_phase < 5 ? LOW : HIGH);
    delay(100);
  }
  left_motor->setSpeed(0);
  right_motor->setSpeed(0);
}
void test_left_motor_forwards(int t)
{
  motor(MOTOR_SPEED, 0, t);
}

void test_left_motor_backwards(int t)
{
  motor(-MOTOR_SPEED, 0, t);
}

void test_right_motor_forwards(int t)
{
  motor(0, MOTOR_SPEED, t);
}

void test_right_motor_backwards(int t)
{
  motor(0, -MOTOR_SPEED, t);
}


void loop() {
  while (digitalRead(START_SWITCH))
  {}
  test_right_motor_forwards(t);
  test_right_motor_backwards(t);
  test_left_motor_forwards(t);
  test_left_motor_backwards(t);
}
