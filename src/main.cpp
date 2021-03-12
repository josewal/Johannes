#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SharpIR.h>

#include <Motor.h>
#include <Encoder.h>
#include <MotorController.h>
#include <HeadServo.h>
#include <Organ.h>
#include <DriveUnit.h>


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
SharpIR sensor( SharpIR::GP2Y0A21YK0F, A0 );


Organ head(servo, sensor);

int drive_seqeunce = -1;

void setup()
{

  Serial.begin(115200);

  leftController.setup(1);
  rightController.setup(0);

  leftController.rpm_PID_setup(20, 5, 0, 255);
  rightController.rpm_PID_setup(20, 5, 0, 255);

  leftController.steps_PID_setup(4, 2, 0, 70);
  rightController.steps_PID_setup(4, 2, 0, 70);

  head.state = 0;
  head.servo.setTarget(90);
}

void resolveCommunication(){

  String json = Serial.readStringUntil('*');
  Serial.flush();
  digitalWrite(LED_BUILTIN, LOW);
  StaticJsonDocument<200> doc;
  deserializeJson(doc, json);

  if(doc["type"] == "servo"){
    int angle = doc["angle"];
    head.servo.setTarget(angle);
  }else if(doc["type"] == "rpm"){
    rpm_l = doc["rpm_l"];
    rpm_r = doc["rpm_r"];
  }  
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
}