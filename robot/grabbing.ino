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
