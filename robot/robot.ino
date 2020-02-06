#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
const float SLOWDOWN = 0.8; //slowdown for testing
const int MOTOR_SPEED = 200 * SLOWDOWN; // default motor speed, don't change PLEASE
const float mps = 0.151 * SLOWDOWN; // meters per second motor time
const float dps = 63 * SLOWDOWN; // degrees per second motor time
int led_phase = 0; //keep track of LED state
const int FOLLOW_TURN = MOTOR_SPEED*1.5; //default turning power subtracted from inside wheel
const int lsense_pins[4] = {A1, A2, A3, A0}; //line sensor pins
const int llights[4] = {3, 4, 5, 2}; //debug LEDs for line sensor
const int START_SWITCH = 6; // switch to start robot
const int FOV_CORRECTION=3;
int last_result = 0; //keep track of last turn to return robot to line
const int trigPin = 13; // ultrasound stuff
const int echoPin = 12; // yeah
const int IR_INPUT = 7;
const int IR_DISTANCE = A5;
float prev_distance = 0;
const int LIFT_ANGLE = 15;
Servo grabber;
Servo lifter;


bool victim_detect() {
  for (int i = 0; i < 10; i++) {
    if (!digitalRead(IR_INPUT)) {
      return true;
    }
    delayMicroseconds(100);
  }
  return false;
}
float ultrasound() {
  float duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 3000);
  distance = (duration / 2) / 29.1;
  if (distance >= 200 || distance <= 0) {
    Serial.println("Out of range");
    return 999;
  }
  else {
    return distance;
  }
}
int sum(int arr[], int l) {
  //add numbers in an array of length l
  int s = 0;
  for (int x = 0; x < l; x++) {
    s += arr[x];
  }
  return s;
}
void confirmatory_flash() {
  //debug purposes only
  for (int i = 0; i < 3; i++) {
    digitalWrite(llights[i], HIGH);
  }
  delay(10);
  for (int i = 0; i < 3; i++) {
    digitalWrite(llights[i], LOW);
  }
  delay(10);
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  // change to actual LED at some point :P
  pinMode(LED_BUILTIN, OUTPUT);
  for (int l = 0; l < 4; l++) {
    pinMode(lsense_pins[l], INPUT);
  }
  for (int l = 0; l < 4; l++) {
    pinMode(llights[l], OUTPUT);
  }
  pinMode(START_SWITCH, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IR_INPUT, INPUT);
  pinMode(IR_DISTANCE, INPUT);
  grabber.attach(10);
  lifter.attach(9);
  pick_robot();
}


void loop() {
  while (digitalRead(START_SWITCH)) {
    get_line_pos();
    Serial.println(analogRead(IR_DISTANCE));
    delay(100);
  }
  follow_line(0, 125 / SLOWDOWN);
  for (int i=0; i<5; i++){
    confirmatory_flash();
    straight(0.15);
    drop_robot();
    spin(120);
    bool scan_success=spin_scan(270, 0.45)!=270;
    if (scan_success&&i!=4){
      prev_distance = 0;
      approach_victim(2,0);
      pick_robot();
      spin(180);
      straight(prev_distance);
    }else{
      //return home :D
      pick_robot();
      if (find_line()) {
        follow_line(0, 100 / SLOWDOWN);
        straight(0.22);
      }
      break;
    }
    if (find_line()) {
      follow_line(-MOTOR_SPEED, 100 / SLOWDOWN);
      drop_robot();
      straight(-0.2);
      spin(-90);
      spin_until(lsense_pins[1], 180);
      pick_robot();
      follow_line(0, 80 / SLOWDOWN);
    }else{
      break;
    }
  }
}
