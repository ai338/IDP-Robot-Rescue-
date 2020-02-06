void motor(int l, int r, int t) {
  //set motors running for time t while flashing LED
  left_motor->setSpeed(abs(l));
  right_motor->setSpeed(abs(r));
  left_motor->run(l < 0 ? BACKWARD : FORWARD);
  right_motor->run(r < 0 ? BACKWARD : FORWARD);
  for (int ds = 0; ds < t; ds++) {
    led_phase++;
    led_phase %= 10;
    digitalWrite(llights[3], led_phase < 5 ? LOW : HIGH);
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
