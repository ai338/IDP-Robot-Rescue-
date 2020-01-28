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
int t = 300;

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

void turn(int bias, int t) {
  //turn robot with one wheel turning at speed MOTOR_SPEED-|bias|
  if (bias < 0) {
    motor(MOTOR_SPEED + bias, MOTOR_SPEED, t);
  } else {
    motor(MOTOR_SPEED, MOTOR_SPEED - bias, t);
  }
}
void straight(float m) {
  //go forward/backward for m meters
  int sign = m < 0 ? -1 : 1;
  motor(MOTOR_SPEED * sign, MOTOR_SPEED * sign, abs(m) / mps * 10);
  prev_distance+=m;
}
void spin(float deg) {
  //turn on the spot
  int sign = deg < 0 ? -1 : 1;
  motor(MOTOR_SPEED * -sign, MOTOR_SPEED * sign, deg / dps * 10);
}

void make_sqare(float m)
{
  straight(m); 
  spin(90); 
  straight(m); 
  spin(90); 
  straight(m); 
  spin(90); 
  straight(m); 
  
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
  make_sqare(0.4); 
}
