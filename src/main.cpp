#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <Wire.h>

#include <Motor.h>
#include <Encoder.h>
#include <MotorController.h>
#include <Sonar.h>
#include <HeadServo.h>
#include <Organ.h>
#include <DriveUnit.h>

#define ANSWERSIZE 5
#define SLAVE_ADDR 8

void recieveEvent(int _howMany);
void requestEvent();

int rpm_l = 0;
int rpm_r = 0;

Motor motorLeft(6, 30, 28, 20);
Motor motorRight(7, 24, 26, 20);
float WHEEL_SPREAD_CIRC;

Encoder *Encoder::instances[2] = {NULL, NULL};
Encoder encoderLeft(3, 5, PINE, 5, 3, PINE);
Encoder encoderRight(2, 4, PINE, 4, 5, PING);

MotorController leftController(motorLeft, encoderLeft);
MotorController rightController(motorRight, encoderRight);

DriveUnit driver(leftController, rightController);

HeadServo servo(10, 0, 180);

Sonar *Sonar::instance = NULL;
Sonar sonar(18, 34, 200);

Organ head(servo, sonar);

int drive_seqeunce = -1;

void setup()
{
  head.servo.begin(50, 0);
  head.sonar.begin();

  Serial.begin(115200);

  leftController.setup(1);
  rightController.setup(0);

  leftController.rpm_PID_setup(20, 5, 0, 255);
  rightController.rpm_PID_setup(20, 5, 0, 255);

  leftController.steps_PID_setup(4, 2, 0, 70);
  rightController.steps_PID_setup(4, 2, 0, 70);

  head.servo.setTarget(90);
  drive_seqeunce = 1;
}

void resolveCommunication(){

  String json = Serial.readStringUntil('*');
  Serial.flush();
  digitalWrite(LED_BUILTIN, LOW);
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json);

  head.servo.setTarget(doc["servo"]);
  rpm_l = doc["rpm_l"];
  rpm_r = doc["rpm_r"];
}

void loop()
{
  if (Serial.available())
  {
    resolveCommunication();
  }
  
  driver.update();
  head.update();
  driver.driveRPM(rpm_l, rpm_r);
<<<<<<< HEAD
}
=======

  // switch (drive_seqeunce)
  // {
  // case 1:
  //   if (head.state == -1)
  //   {
  //     head.servo.setTarget(90);
  //     driver.rotateBy(head.max_dist_angle - 90);
  //     drive_seqeunce = 2;
  //   }
  //   break;

  // case 2:
  //   if (driver.rotationDone)
  //   {
  //     int dist = min(150, head.max_dist - 30);
  //     driver.driveCM(dist);
  //     drive_seqeunce = 3;
  //   }
  //   break;

  // case 3:
  //   if (driver.arrived)
  //   {
  //     head.setScan(10, 0, 180);
  //     drive_seqeunce = 1;
  //   }
  //   break;

  // default:
  //   break;
  // }
}
>>>>>>> parent of 4f040d4 (adding dist sensor removing sonar)
