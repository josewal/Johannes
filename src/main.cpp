#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#include <Motor.h>
#include <Encoder.h>
#include <MotorController.h>
#include <Sonar.h>
#include <HeadServo.h>
#include <Organ.h>

Motor motorLeft(6, 30, 28, 30);
Motor motorRight(7, 26, 24, 30);
const float WHEEL_SPREAD = 17;
float WHEEL_SPREAD_CIRC;

Encoder *Encoder::instances[2] = {NULL, NULL};
Encoder encoderLeft(3, 5, PINE, 5, 3, PINE);
Encoder encoderRight(2, 4, PINE, 4, 5, PING);

MotorController controllerLeft(motorLeft, encoderLeft);
MotorController controllerRight(motorRight, encoderRight);

HeadServo servo(10, 0, 180);

Sonar *Sonar::instance = NULL;
Sonar sonar(21, 34, 50);

Organ head(servo, sonar);

int angle = 0;
int prev_anle = 0;

boolean movedL = false;
boolean movedR = false;

void setup()
{
  head.servo.begin(50, 0);
  head.sonar.begin();

  Serial.begin(9600);

  controllerLeft.setup(1);
  controllerRight.setup(0);
  WHEEL_SPREAD_CIRC = WHEEL_SPREAD * PI;

  controllerLeft.rpm_PID_setup(6, 1, 0, 255);
  controllerRight.rpm_PID_setup(6, 1, 0, 255);

  controllerLeft.steps_PID_setup(3, 2, 0, 70);
  controllerRight.steps_PID_setup(3, 2, 0, 70);

  head.setScan(10, 0, 180);
}

void drive()
{

  if (head.distance < 50)
  {
    if (head.angle_pinged_at <= 90)
    {
      controllerLeft.moveRPM(40);
      controllerRight.moveRPM(0);
    }
    else
    {
      controllerLeft.moveRPM(0);
      controllerRight.moveRPM(40);
    }
  }
  else
  {
    if (head.distance > 200)
    {
      if (head.angle_pinged_at <= 90)
      {
        controllerLeft.moveRPM(0);
        controllerRight.moveRPM(40);
      }
      else
      {
        controllerLeft.moveRPM(40);
        controllerRight.moveRPM(0);
      }
    }
    else
    {
      controllerLeft.moveRPM(30);
      controllerRight.moveRPM(30);
    }
  }
}

void rotate(float _angle)
{
  float dist = _angle / 360 * WHEEL_SPREAD_CIRC;

  controllerLeft.moveCM(dist);
  controllerLeft.rotated = false;
  controllerRight.moveCM(-dist);
  controllerRight.rotated = true;
}

void loop()
{
  controllerLeft.update();
  controllerRight.update();

  head.update();

  if (head.state == -1)
  {
    if (head.max_dist_angle != -1)
    {
      angle = angle + head.max_dist_angle - 90;
      rotate(head.max_dist_angle - 90);
      head.max_dist_angle = -1;
    }
    else
    {
      if (controllerLeft.rotated && controllerLeft.rotated)
      {
        if (movedL && movedR)
        {
          if (controllerLeft.isSettled() && controllerRight.isSettled())
          {
            head.setScan(10, 0, 180);
            movedL = false;
            movedR = false;
          }
        }
        else
        {
          controllerLeft.moveCM(min(100,head.max_dist - 20));
          controllerRight.moveCM(min(100,head.max_dist - 20));
          movedL = true;
          movedR = true;
        }
      }
    }
  }
}
