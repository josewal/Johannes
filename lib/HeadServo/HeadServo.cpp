#include <HeadServo.h>

HeadServo::HeadServo() {}
HeadServo::HeadServo(int servo_pin, int min = 0, int max = 180)
{
  SERVO_PIN = servo_pin;
  MIN_ANGLE = min;
  MAX_ANGLE = max;
}

void HeadServo::begin(int _stept = 50, int _restt = 100)
{
  servo.attach(SERVO_PIN);
  servo.write(angle);
  last_step_time = millis();
  min_step_time = _stept;
  min_rest_time = _restt;
}

void HeadServo::servoStep(int step_angle = 1)
{
  if (angle < angleToReach)
  {
    angle = min(180, angle + step_angle);
  }
  else
  {
    angle = max(0, angle - step_angle);
  }

  last_step_time = millis();
  servo.write(angle);
}

void HeadServo::update()
{
  if (angle == angleToReach)
  {
    movement_done = true;
  }
  else
  {
    movement_done = false;
    settled = false;
    int step_diff = abs(angleToReach -angle);
    if (millis() - last_step_time > min_step_time)
    {
      if (step_diff > 5){
        servoStep(step_diff/2);
      }else
      {
        servoStep(1);
      }
      
      
    }
  }

  if (movement_done && !settled)
  {
    settled = millis() - last_step_time > min_rest_time;
  }
}

void HeadServo::setTarget(int _angle)
{
  angleToReach = constrain(_angle, MIN_ANGLE, MAX_ANGLE);
}

boolean HeadServo::isSettled()
{
  return settled;
}