void motor(int l, int r, float t) {
  //set motors running for time t (in seconds) while flashing LED
  if (t < 0.05) {
    //for small t change motor speed
    float speedchange = t / 0.05;
    l *= speedchange;
    r *= speedchange;
    t = 0.05;
  }
  left_motor->setSpeed(abs(l));
  right_motor->setSpeed(abs(r));
  left_motor->run(l < 0 ? BACKWARD : FORWARD);
  right_motor->run(r < 0 ? BACKWARD : FORWARD);
  for (int hds = 0; hds < t * 20; hds++) {
    //update LED
    led_phase++;
    led_phase %= 20;
    digitalWrite(llights[3], led_phase % 10 < 5 ? LOW : HIGH);
    delay(50);
  }
  left_motor->setSpeed(0);
  right_motor->setSpeed(0);
}
void turn(int bias, float t) {
  //turn robot with one wheel turning at speed MOTOR_SPEED - |bias|
  if (bias < 0) {
    motor(MOTOR_SPEED + bias, MOTOR_SPEED, t);
  } else {
    motor(MOTOR_SPEED, MOTOR_SPEED - bias, t);
  }
}
void straight(float m) {
  //go forward/backward for m meters
  int sign = m < 0 ? -1 : 1;
  motor(MOTOR_SPEED * sign, MOTOR_SPEED * sign, fabs(m) / mps);
  prev_distance += m;
}
void spin(float deg) {
  //turn on the spot
  int sign = deg < 0 ? -1 : 1;
  motor(MOTOR_SPEED * -sign, MOTOR_SPEED * sign, fabs(deg) / dps);
}
float spin_until(int pin, int deg_max) {
  //spin until we get a signal from pin or reach deg_max
  float dpds = dps / 20;
  float total = 0;
  for (int i = 0; i < deg_max / dpds; i++) {
    if (digitalRead(pin)) {
      confirmatory_flash();
      return total;
    }
    motor(MOTOR_SPEED, -MOTOR_SPEED, 0.05);
    total += dpds;
  }
  return deg_max;
}
float spin_scan(int deg_max, float slowdown, bool use_ultra) {
  //spin slower until we get reading from a victim
  float dpds = dps / 20 * slowdown;
  float total = 0;
  for (int i = 0; i < deg_max / dpds; i++) {
    if (use_ultra ? ultrasound() < ULTRA_SCAN : victim_detect()) {
      confirmatory_flash();
      spin(use_ultra ? -ULTRA_FOV : -FOV_CORRECTION);
      return total;
    }
    motor(MOTOR_SPEED * slowdown, -MOTOR_SPEED * slowdown, 0.05);
    total += dpds;
  }
  return deg_max;
}
float move_until(int pin, float dist_min, float dist_max) {
  //straight until we get a signal from any of the pin or reach dist_max
  //if pin -1 wait for ANY front line sensor
  float mpds = mps / 20;
  float total = 0;
  for (int i = 0; i < (dist_max / mpds); i++) {
    if ((pin == -1 ? get_line_pos() != -2 : digitalRead(pin)) && (total > dist_min)) {
      confirmatory_flash();
      return total;
    }
    motor(MOTOR_SPEED, MOTOR_SPEED, 0.05);
    total += mpds;
  }
  return dist_max;
}
