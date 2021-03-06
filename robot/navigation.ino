int get_line_pos() {
  //get line position, returns -1 to 1 for single sensor, sensor count for multiple sensors and -2 for no sensors
  int results[3];
  for (int l = 0; l < 4; l++) {
    int r = digitalRead(lsense_pins[l]);
    if (l < 3) {
      results[l] = r;
    }
    //set debug leds
    if (l < 3) {
      digitalWrite(llights[l], r);
    }
  }
  if (sum(results, 3) == 3) {
    return 3;
  } else if (sum(results, 3) == 2) {
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
  //follow line with turning bias when multiple sensors for follow_time seconds, and then stops following at T-junction after follow_time seconds
  for (int t = 0; true; t++) {
    int result = get_line_pos();
    if (result > 1) {
      if (result == 3 && t > follow_time * 2) {
        break;
      }
      turn(bias, 0.05);
      last_result = bias;
    } else if (result == -2) {
      turn(last_result, 0.05);
    } else {
      turn(FOLLOW_TURN * -result, 0.05);
      last_result = FOLLOW_TURN * -result;
    }
  }
}

bool find_line()
{
  //find line from cave entrance, hopefully ignoring the T
  if (spin_until(lsense_pins[3], 360) != 360) {
    spin(180);
    if (move_until(-1, 0.12, 0.36) != 0.36) {
      return true;
    }
  }
  return false;
}
bool approach_victim(float max_d, bool use_ultra) {
  //approach a target, returns true if successful
  float last_u = ULTRA_SCAN;
  for (int i = 0; i < max_d / (mps / 10); i++) {
    straight(mps / 10);
    float u = ultrasound();
    if (u <= 10) {
      //in grab range
      return true;
    }
    if (i % 5 == 3 && (use_ultra ? u > last_u : !victim_detect())) {
      //victim signal lost, readjust
      spin(12);
      if (spin_scan(30, 0.25, use_ultra) == 30 || (use_ultra && (last_u + 5 < ultrasound()))) {
        //failed :(
        spin(-12);
        return false;
      }
    }
    if (use_ultra && victim_detect()) {
      //use IR if we get IR
      use_ultra = false;
    } else if (use_ultra) {
      float u = ultrasound();
      last_u = u != 999 ? u : last_u;
      //last_u used to (hopefully) avoid trying to grab walls
    }
  }
  return false;
}
