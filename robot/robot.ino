#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
const float SLOWDOWN = 1.0; //slowdown for testing
const int MOTOR_SPEED = 200 * SLOWDOWN; // default motor speed, don't change PLEASE
const float mps = 0.155 * SLOWDOWN; // meters per second motor time
const float dps = 67 * SLOWDOWN; // degrees per second motor time
int led_phase = 0; //keep track of LED state
const int FOLLOW_TURN = MOTOR_SPEED*1.5; //default turning power subtracted from inside wheel
const int lsense_pins[4] = {A1, A2, A3, A0}; //line sensor pins
const int llights[4] = {3, 4, 5, 2}; //debug LEDs for line sensor, llights[3] is now amber movement LED
const int START_SWITCH = 6; // switch to start robot
const int FOV_CORRECTION=2; // field of view correction to point directly at victim
const int ULTRA_FOV=7; // same as above but for ultrasound
const int ULTRA_SCAN=50; // max dist for ultrasound detection
int last_result = 0; //keep track of last turn to return robot to line (unused?)
const int trigPin = 13; // ultrasound trigger pin
const int echoPin = 12; // ultrasound echo pin
const int IR_INPUT = 7; // digital feed from IR sensor
const int IR_DISTANCE = A5; // IR distance sensor pin (unused)
float prev_distance = 0; // cumulative counter for straight() calls 
bool ultra_scan=false;
unsigned long start_time;
Servo grabber;
Servo lifter;


bool victim_detect() {
  //monitor IR pin for any low signal over 2ms (as input could be square wave)
  for (int i = 0; i < 20; i++) {
    if (!digitalRead(IR_INPUT)) {
      return true;
    }
    delayMicroseconds(100);
  }
  return false;
}
float ultrasound() {
  // read ultrasound sensor, should give up at ~60cm for faster readings
  float duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 4000);
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
  //flash all line_following LEDs briefly
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
  //reset grabber
  pick_robot();
}


void loop() {
  // wait for switch
  while (digitalRead(START_SWITCH)) {
    //read line pos to update debug LEDs
    get_line_pos();
    delay(100);
    if (ultrasound()==999){
      //flash if ultrasound out of range
      confirmatory_flash();
      delay(100);
    }
  }
  start_time=millis();
  //enter cave
  follow_line(0, 120 / SLOWDOWN);
  for (int i=0; i<5; i++){
    ultra_scan=false;
    confirmatory_flash();
    //get to optimum scanning position
    straight(0.12);
    drop_robot();
    //prepare the claw
    spin(130);
    //reset if 4 pickup attempts or 4 minutes passed
    bool auto_return=i==4 || millis()-start_time>4*60*1000.0;
    bool scan_success=!auto_return && spin_scan(270, 0.45, false)!=270;
    if (!scan_success && !auto_return){
      //IR failed, try ultrasound
      ultra_scan=true;
      spin(260);
      scan_success=spin_scan(270, 0.45, true)!=270;
    }
    if (scan_success){
      prev_distance = 0;
      if (approach_victim(2,ultra_scan)){
        //flash LED before pickup
        digitalWrite(llights[3],HIGH);
        delay(1000);
        digitalWrite(llights[3],LOW);
        pick_robot();
      }else{
        scan_success=false;
        //try and return home at this point?
      }
      spin(180);
      straight(prev_distance*1.05); //fudge factor to get closer to T
    }if (!scan_success){
      //return home :D
      pick_robot();
      if (find_line()) {
        follow_line(0, 100 / SLOWDOWN);
        straight(0.2);
      }
      break;
    }
    if (find_line()) {
      //go to triage area
      follow_line(-MOTOR_SPEED, 100 / SLOWDOWN);
      drop_robot();
      straight(-0.2);
      spin(-90);
      //reacquire line
      spin_until(lsense_pins[1], 180);
      //retract claw to avoid tunnel collisions
      pick_robot();
      follow_line(0, 80 / SLOWDOWN);
    }else{
      //we couldn't find the line, give up
      break;
    }
  }
}
