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
const float dps = 67 * SLOWDOWN; // degrees per second motor time
int led_phase = 0; //keep track of LED state
const int FOLLOW_TURN = MOTOR_SPEED; //default turning power subtracted from inside wheel
const int lsense_pins[4] = {A1, A2, A3, A0}; //line sensor pins
const int llights[4] = {3, 4, 5, 2}; //debug LEDs for line sensor
const int START_SWITCH = 6; // switch to start robot
const int FOV_CORRECTION=5;
int last_result = 0; //keep track of last turn to return robot to line
const int trigPin = 13; // ultrasound stuff
const int echoPin = 12; // yeah
const int IR_INPUT = 7;
float prev_distance = 0;
const int LIFT_ANGLE = 15; 
Servo myservo_grab; 
Servo myservo_lift; 


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
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
  if (distance >= 200 || distance <= 0) {
    Serial.println("Out of range");
    return -1;
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
  for (int i = 0; i < 4; i++) {
    digitalWrite(llights[i], HIGH);
  }
  delay(10);
  for (int i = 0; i < 4; i++) {
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
  //uncomment when ultrasound ready
  /*pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);*/
  pinMode(IR_INPUT, INPUT);
  randomSeed(analogRead(IR_INPUT));
}


void loop() {
  while (digitalRead(START_SWITCH)) {
    get_line_pos();
    delay(100);
    float ultra = ultrasound();
    if (ultra==-1){
      confirmatory_flash();
    }
    Serial.println(ultra);
  }
  follow_line(0, 125 / SLOWDOWN);
  for (int i=0; i<5; i++){
    confirmatory_flash();
    straight(0.2);
    spin(90);
    bool scan_success=spin_scan(180, 0.5)!=180;
    if (scan_success&&i!=4){
      prev_distance = 0;
      approach_victim(2);
      //grab
      spin(180);
      straight(prev_distance);
    }else{
      if (find_line()) {
        follow_line(0, 100 / SLOWDOWN);
        straight(0.35);
      }
      break;
    }
    if (find_line()) {
      follow_line(-FOLLOW_TURN / 2, 100 / SLOWDOWN);
      //drop
      straight(-0.2);
      spin(-90);
      spin_until(lsense_pins[1], 180);
      follow_line(0, 80 / SLOWDOWN);
    }
  }
}
