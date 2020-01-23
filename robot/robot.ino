#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
const int MOTOR_SPEED = 200;
const float mps = 0.153; // meters per second motor time
const float dps = 70; // degrees per second motor time
int led_phase = 0;
const int FOLLOW_TURN = 40;
const int lsense_pins[4] = {A0,A1,A2,A3};
const int llights[4] = {2,3,4,5};
const int START_SWITCH = 12;
int last_result=0;
int sum(int arr[], int l) {
  //add numbers in an array of length l
  int s = 0;
  for (int x = 0; x < l; x++) {
    s += arr[x];
  }
  return s;
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
  //go forward for m meters
  motor(MOTOR_SPEED, MOTOR_SPEED, m / mps * 10);
}
void spin(float degrees) {
  //turn on the spot
  int sign = degrees < 0 ? -1 : 1;
  motor(MOTOR_SPEED * -sign, MOTOR_SPEED * sign, degrees / dps * 10);
}
int get_line_pos() {
  //get line position, returns -1 to 1 for single sensor, 2 for multiple sensors and -2 for no sensors
  int results[3];
  for (int l = 0; l < 3; l++) {
    results[l] = digitalRead(lsense_pins[l]);
    //debug leds
    digitalWrite(llights[l],results[l]);
  }
  if (sum(results,3) > 1) {
    return 2;
  } else if (results[0]) {
    return -1;
  } else if (results[1]) {
    return 0;
  }else if (results[2]){
    return 1;
  }
  return -2;
}
void line_follow(int bias, int follow_time) {
  //follow line with turning bias when multiple sensors for follow_timex100ms
  for (int t = 0; t < follow_time; t++) {
    int result = get_line_pos();
    if (result == 2) {
      turn(bias, 1);
      last_result=bias;
    }else if (result==-2){
      turn(last_result,1);
    } else {
      turn(FOLLOW_TURN * result, 1);
      last_result=FOLLOW_TURN*result;
    }
  }
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
}

void loop() {
  while (digitalRead(START_SWITCH)) {
    get_line_pos();
  }
  //Serial.println("GOING!");
  line_follow(0,100);
}
