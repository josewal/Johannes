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

Encoder *Encoder::instances[2] = {NULL, NULL};
Encoder encoderLeft(3, 5, PINE, 5, 3, PINE);
Encoder encoderRight(2, 4, PINE, 4, 5, PING);

MotorController controllerLeft(motorLeft, encoderLeft);
MotorController controllerRight(motorRight, encoderRight);

HeadServo servo(10, 0, 180);

Sonar *Sonar::instance = NULL;
Sonar sonar(21, 34, 50);

Organ head(servo, sonar);

void setup()
{
  head.servo.begin(50, 100);
  head.sonar.begin();

  Serial.begin(9600);

  controllerLeft.setup(1);
  controllerRight.setup(0);

  controllerLeft.rpm_PID_setup(6, 1, 0, 255);
  controllerRight.rpm_PID_setup(6, 1, 0, 255);

  head.scan(10, 20, 160);
}

void loop()
{
  controllerLeft.update();
  controllerRight.update();
  head.update();

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

  if (head.scan_done)
  {
    Serial.println();
    Serial.print("Min dist: ");
    Serial.print(head.min_dist);
    Serial.print(" at angle: ");
    Serial.println(head.min_dist_angle);
    Serial.println();
    head.scan(head.scan_step, head.scan_end, head.scan_start);
  }
}
