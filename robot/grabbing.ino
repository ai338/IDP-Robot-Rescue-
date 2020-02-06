void actuate_servo(Servo servo, int target_angle){
  int start_angle = servo.read();
  int sign=target_angle>start_angle?1:-1;
  for (int a = start_angle; a!=target_angle; a+=sign){
    servo.write(a);
    delay(5);
  }
}
void pick_robot()
{
  actuate_servo(grabber, 95);
  actuate_servo(lifter, 90);
}

void drop_robot()
{
  actuate_servo(grabber, 0);
  actuate_servo(lifter, 150);
}
