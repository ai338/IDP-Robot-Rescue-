
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);
Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
const float SLOWDOWN = 0.6; //slowdown for testing
const int MOTOR_SPEED = 200 * SLOWDOWN; // default motor speed, don't change PLEASE
const float mps = 0.151 * SLOWDOWN; // meters per second motor time
const float dps = 62 * SLOWDOWN; // degrees per second motor time
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
  if (fabs(deg)<dps/10){
    float slowdown=fabs(deg)/(dps/10);
    motor(MOTOR_SPEED * -sign * slowdown, MOTOR_SPEED * sign * slowdown, 1);
  }else{
    motor(MOTOR_SPEED * -sign, MOTOR_SPEED * sign, fabs(deg) / dps * 10);
  }
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
    motor(MOTOR_SPEED, -MOTOR_SPEED, 1);
    total += dpds;
  }
  return deg_max;
}
float spin_scan(int deg_max, float slowdown) {
  //spin slower until we get reading from a victim
  //spin until we get a signal from pin or reach deg_max
  float dpds = dps / 10 * slowdown;
  float total = 0;
  for (int i = 0; i < deg_max / dpds; i++) {
    if (victim_detect()) {
      confirmatory_flash();
      spin(-FOV_CORRECTION);
      return total;
    }
    motor(MOTOR_SPEED * slowdown, -MOTOR_SPEED * slowdown, 1);
    total += dpds;
  }
  return deg_max;
}
float move_until(int pin, float dist_min, float dist_max) {
  //straight until we get a signal from any of the pin or reach dist_max
  //if pin -1 wait for ANY front line sensor
  float mpds = mps / 10.0;
  float total = 0;
  for (int i = 0; i < (dist_max / mpds); i++) {
    if ((pin == -1 ? get_line_pos() != -2 : digitalRead(pin)) && (total > dist_min)) {
      confirmatory_flash();
      return total;
    }
    motor(MOTOR_SPEED, MOTOR_SPEED, 1);
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
  //get line position, returns -1 to 1 for single sensor, sensor count for multiple sensors and -2 for no sensors
  int results[3];
  for (int l = 0; l < 4; l++) {
    int r = digitalRead(lsense_pins[l]);
    if (l < 3) {
      results[l] = r;
    }
    //debug leds
    digitalWrite(llights[l], r);
  }
  if (sum(results, 3) == 3) {
    return 3;
  } else if (sum(results, 2) == 2) {
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
  //follow line with turning bias when multiple sensors for follow_timex100ms, and then stops following at T or at 3*follow_time
  for (int t = 0; t < follow_time * 3; t++) {
    int result = get_line_pos();
    if (result > 1) {
      if (result == 3 && t > follow_time) {
        break;
      }
      turn(bias, 1);
      last_result = bias;
    } else if (result == -2) {
      turn(last_result, 1);
    } else {
      turn(FOLLOW_TURN * -result, 1);
      last_result = FOLLOW_TURN * -result;
    }
  }
}

bool find_line()
{
  if (spin_until(lsense_pins[3], 360) != 360) {
    spin(180);
    if (move_until(-1, 0.12, 0.36) != 0.36) {
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

  if (find_line()) {
    follow_line(bias, follow_time);
    return true;
  }
  return false;
}
void approach_victim(float max_d){
  for (int i=0;i<max_d/(mps/10);i++){
    straight(mps/10);
    if (ultrasound()<5){
      break;
    }
    if (!victim_detect()){
      spin(10);
      if (spin_scan(20,0.5)==20){
        break;
      }
    }
  }
}

void ungrip(int starting_angle)
{
  for (int angle = starting_angle; angle >= 1; angle-=5) //command to move from starting_angle to 0
  {
    myservo_grab.write(angle); // command to rotate the servo to the specified angle
    delay(5); 
  }

  delay(1000); 
}

void grip(int finishing_angle)
{
  for (int angle = 0; angle <= finishing_angle; angle += 1) //command to move from 0 degrees to finishing_degrees
  {
    myservo_grab.write(angle);  // command to rotate the servo to the specified angle
    delay(15); 
  }

  delay(1000); 
}

void lift_up(int lift_angle)
{
  for (int angle = 0; angle <= lift_angle; angle++) // command to move from 0 degrees to lift_angle
  {
    myservo_lift.write(angle); 
    delay(15); 
  }

  delay(1000); 
}

void lift_down(int lift_angle)
{
  for (int angle = lift_angle; angle >= 0; angle--)
  {
    myservo_lift.write(angle); 
    delay(15); 
  }

  delay(1000); 
}

void pick_robot()
{
  grip(90); 
  lift_up(LIFT_ANGLE); 
}

void drop_robot()
{
  lift_down(LIFT_ANGLE); 
  ungrip(90);
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
    Serial.println(ultrasound());
  }
  if(spin_scan(180,0.5)!=180){
    Serial.println("AYAAAAAAAAAAAAAA");
    approach_victim(1);
  }
  
//  follow_line(0, 100 / SLOWDOWN);
//  confirmatory_flash();
//  straight(0.2);
//  spin(90);
//  spin_scan(180, 0.5);
//  prev_distance = 0;
//  straight(random(2, 5) * 0.1);
//  spin(180);
//  straight(prev_distance);
//  if (find_line()) {
//    follow_line(-FOLLOW_TURN / 2, 100 / SLOWDOWN);
//    straight(-0.2);
//    spin(-90);
//    spin_until(lsense_pins[1], 180);
//    follow_line(0, 80 / SLOWDOWN);
//    straight(0.2);
//    spin(180);
//    if (find_line()) {
//      follow_line(0, 100 / SLOWDOWN);
//      straight(0.4);
//    }
//
//  }
}
