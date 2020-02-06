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
void approach_victim(float max_d, int ir_target){
  for (int i=0;i<max_d/(mps/10);i++){
    straight(mps/10);
    if (ir_target==0? ultrasound()<=10:analogRead(IR_DISTANCE)>ir_target){
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
