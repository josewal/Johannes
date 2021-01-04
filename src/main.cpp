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

Motor motorLeft(6, 30, 28, 30);
Motor motorRight(7, 26, 24, 30);

Encoder *Encoder::instances[2] = {NULL, NULL};
Encoder encoderLeft(3, 5, PINE, 5, 3, PINE);
Encoder encoderRight(2, 4, PINE, 4, 5, PING);

MotorController controllerLeft(motorLeft, encoderLeft);
MotorController controllerRight(motorRight, encoderRight);

Sonar *Sonar::instance = NULL;
Sonar sonar(21, 34, 50);

Servo sonar_servo;
int angle = 90;
int angleToReach = 100;

float distance = 0;
float min_dist = 999;
int min_dist_angle = 0;
int obstacle_angle = 0;

void setup()
{
  Serial.begin(9600);

  controllerLeft.setup(1);
  controllerRight.setup(0);

  controllerLeft.rpm_PID_setup(6, 1, 0, 255);
  controllerRight.rpm_PID_setup(6, 1, 0, 255);

  sonar.begin();

  sonar_servo.attach(10);
}

void sweep(int step = 1, int range = 20)
{
  if (angle < angleToReach)
  {
    angle = min(180, angle + step);
    sonar_servo.write(angle);
    if (angleToReach - angle < step)
    {
      int diffAngle = angleToReach - min_dist_angle;
      if (diffAngle < range / 2)
      {
        angleToReach = min(180, min_dist_angle + range / 2);
        min_dist = 999;
      }
      else
      {
        angleToReach = max(0, min_dist_angle - range / 2);
        obstacle_angle = min_dist_angle;
      }
    }
  }
  else if (angle >= angleToReach)
  {
    angle = max(0, angle - step);
    sonar_servo.write(angle);
    if (angle - angleToReach < step)
    {
      int diffAngle = min_dist_angle - angleToReach;
      if (diffAngle < range / 2)
      {
        angleToReach = max(range / 2, min_dist_angle - range / 2);
        obstacle_angle = min_dist_angle;
      }
      else
      {
        angleToReach = min(180 - (range / 2), min_dist_angle + range / 2);
        min_dist = 999;
      }
    }
  }
}

void loop()
{
  controllerLeft.update();
  controllerRight.update();
  sonar.update();

  distance = sonar.getDistance();
  if (distance < min_dist)
  {
    min_dist = distance;
    min_dist_angle = angle;
  }
  Serial.println(min_dist_angle);
  sweep(2, 180);

  delay(100);
}
