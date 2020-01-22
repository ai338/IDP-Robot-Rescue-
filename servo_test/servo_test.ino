#include <Servo.h>             //Servo library
 
Servo servo_test;        //initialize a servo object for the connected servo  
                
 
 
void setup() 
{ 
  servo_test.attach(9);      // attach the signal pin of servo to pin9 of arduino
} 
  
void loop() 
{ 
  int angle_rotation = 180; 
  grip(angle_rotation); 
  ungrip(angle_rotation); 
  
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
