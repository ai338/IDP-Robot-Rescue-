#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
const int MOTOR_SPEED = 128;
const float mps = 1; // meters per second motor time
const float dps = 90; // degrees per second motor time
int led_phase=0;
const int FOLLOW_TURN = 20;
const int lsense_pins[3] = {0,0,0};
int sum(int arr[]){
  int s=0;
  for (int x=0;x<sizeof(arr)/sizeof(int);x++){
    s+=arr[x];
  }
  return s;
}
void motor(int l, int r, int t){
  left_motor->setSpeed(abs(l));
  right_motor->setSpeed(abs(r));
  left_motor->run(l<0?BACKWARD:FORWARD);
  right_motor->run(r<0?BACKWARD:FORWARD);
  for (int ds=0;ds<t;ds++){
    led_phase++;
    led_phase%=10;
    digitalWrite(LED_BUILTIN,led_phase<5?LOW:HIGH);
    delay(100);
  }
  left_motor->setSpeed(0);
  right_motor->setSpeed(0);
}
void turn(int bias, int t){
  if (bias<0){
    motor(MOTOR_SPEED+bias,MOTOR_SPEED,t);
  }else{
    motor(MOTOR_SPEED,MOTOR_SPEED-bias,t);
  }
}
void straight(float m){
  motor(MOTOR_SPEED,MOTOR_SPEED,m/mps*10);
}
void spin(float degrees){
  int sign=degrees<0?-1:1;
  motor(MOTOR_SPEED*sign,MOTOR_SPEED*(1-sign),degrees/dps*10);
}
int get_line_pos(){
  int results[3];
  for (int l=0;l<3;l++){
    results[l]=digitalRead(lsense_pins[l]);
  }
  if (sum(results)>1){
    return 2;
  }else if (results[0]){
    return -1;
  }else if (results[1]){
    return 0;
  }
  return 1;
}
void line_follow(int bias, int follow_time){
  for (int t=0;t<follow_time;t++){
    int result = get_line_pos();
    if (result==2){
      turn(bias,1);
    }else{
      turn(FOLLOW_TURN*result,1);
    }
  }
}
void setup() {
  AFMS.begin();
  // change to actual LED at some point :P
  pinMode(LED_BUILTIN, OUTPUT);
  for (int l=0;l<3;l++){
    pinMode(lsense_pins[l], INPUT);
  }
}

void loop() {
  straight(1);
  spin(90);
}
