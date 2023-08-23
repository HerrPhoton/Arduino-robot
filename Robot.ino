#include "Robot.h"

void setup() 
{
  delay(1000);
  Serial.begin(9600);

  Robot robot;

  robot.set_angles(90, 135, 90, 0);
}

void loop() 
{

}
