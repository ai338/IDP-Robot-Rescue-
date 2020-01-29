
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>             //Servo library
Servo servo_test;        //initialize a servo object for the connected servo  
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
const float SLOWDOWN = 0.5; //slowdown for testing
const int MOTOR_SPEED = 200 * SLOWDOWN; // default motor speed, don't change PLEASE
const float mps = 0.151 * SLOWDOWN; // meters per second motor time
const float dps = 62 * SLOWDOWN; // degrees per second motor time
int led_phase = 0; //keep track of LED state
const int FOLLOW_TURN = MOTOR_SPEED; //default turning power subtracted from inside wheel
const int lsense_pins[4] = {A1, A2, A3, A0}; //line sensor pins
const int llights[4] = {3, 4, 5, 2}; //debug LEDs for line sensor
const int START_SWITCH = 12; // switch to start robot
int last_result = 0; //keep track of last turn to return robot to line
const int trigPin = 12; // ultrasound stuff
const int echoPin = 13; // yeah
float prev_distance = 0;


long ultrasound() {
  long duration, distance;
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
  motor(MOTOR_SPEED * sign, MOTOR_SPEED * sign, fabs(m) / mps * 10);
  prev_distance += m;
}
void spin(float deg) {
  //turn on the spot
  int sign = deg < 0 ? -1 : 1;
  motor(MOTOR_SPEED * -sign, MOTOR_SPEED * sign, deg / dps * 10);
}
float spin_until(int pin, int deg_max) {
  //spin until we get a signal from pin or reach deg_max
  float dpds = dps / 10;
  float total = 0;
  for (int i = 0; i < deg_max / dpds; i++) {
    if (digitalRead(pin)) {
      confirmatory_flash();
      return total;
    }
    spin(dpds);
    total += dpds;
  }
  return deg_max;
}
float move_until(int pin, float dist_min, float dist_max) {
  //straight until we get a signal from pin or reach dist_max
  float mpds = mps / 10.0;
  float total = 0;
  for (int i = 0; i < (dist_max / mpds); i++) {
    if (digitalRead(pin) && (total > dist_min)) {
      confirmatory_flash();
      return total;
    }
    motor(MOTOR_SPEED,MOTOR_SPEED,1);
    total += mpds;
  }
  return dist_max;
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
}
int get_line_pos() {
  //get line position, returns -1 to 1 for single sensor, 2 for multiple sensors and -2 for no sensors
  int results[3];
  for (int l = 0; l < 4; l++) {
    int r = digitalRead(lsense_pins[l]);
    if (l < 3) {
      results[l] = r;
    }
    //debug leds
    digitalWrite(llights[l], r);
  }
  if (sum(results, 3) > 1) {
    return 2;
  } else if (results[0]) {
    return -1;
  } else if (results[1]) {
    return 0;
  } else if (results[2]) {
    return 1;
  }
  return -2;
}
void follow_line(int bias, int follow_time) {
  //follow line with turning bias when multiple sensors for follow_timex100ms
  for (int t = 0; t < follow_time; t++) {
    int result = get_line_pos();
    if (result == 2) { // multiple sensors
      turn(bias, 1);
      last_result = bias;
    } else if (result == -2) { // no sensors
      turn(last_result, 1);
    } else { // single sensor turned on 
      turn(FOLLOW_TURN * -result, 1); 
      last_result = FOLLOW_TURN * -result;
    }
  }
}

bool find_line()
{
  if (spin_until(lsense_pins[3], 360) != 360) {
    spin(180);
    if (move_until(lsense_pins[1], 0.12, 0.36) != 0.36) {
      return true;
    }
  }
  return false;
}

bool return_back(float distance, int deg, int bias, int follow_time)
{
  spin(180);
  straight(prev_distance);
  spin(deg);

  if (find_line()){
    follow_line(bias, follow_time);
    return true;
  }
  return false;
}

void ungrip(int starting_angle)
{
  for (int angle = starting_angle; angle >= 1; angle-=5) //command to move from starting_angle to 0
  {
    servo_test.write(angle); // command to rotate the servo to the specified angle
    delay(5); 
  }

  delay(1000); 
}

void grip(int finishing_angle)
{
  for (int angle = 0; angle <= finishing_angle; angle += 1) //command to move from 0 degrees to finishing_degrees
  {
    servo_test.write(angle);  // command to rotate the servo to the specified angle
    delay(15); 
  }

  delay(1000); 
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
  }
  //Serial.println("GOING!");
  follow_line(0,300);
  int rturn = random(-90,90);
  spin(rturn);
  prev_distance=0;
  straight(random(1,10)*0.1);
  spin(180);
  straight(prev_distance);
  if (find_line()) {
    follow_line(0, 300);
  }
}
