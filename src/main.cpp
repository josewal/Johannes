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

Motor motorLeft(6, 30, 28, 30);
Motor motorRight(7, 26, 24, 30);
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

  Serial.begin(9600);
  Wire.begin(SLAVE_ADDR);

  Wire.onRequest(requestEvent);
  Wire.onReceive(recieveEvent);

  leftController.setup(1);
  rightController.setup(0);

  leftController.rpm_PID_setup(6, 1, 0, 255);
  rightController.rpm_PID_setup(6, 1, 0, 255);

  leftController.steps_PID_setup(4, 2, 0, 70);
  rightController.steps_PID_setup(4, 2, 0, 70);

<<<<<<< HEAD
  head.state = 0;
  head.servo.setTarget(90);
=======
  head.setScan(10, 0, 180);
  drive_seqeunce = 1;
>>>>>>> parent of 352ced0... Matlab files added
}

void recieveEvent(int _howMany)
{
  Serial.println("Event recieved:");
  int json_buffer = 30;
  char json[json_buffer];
  int i = 0;
  while (Wire.available())
  {
    json[i] = Wire.read();
    if (json[i] == 125)
    {
      Wire.flush();
      break;
    }
    i++;
  }
  StaticJsonDocument<200> doc;
<<<<<<< HEAD
  deserializeJson(doc, json);

  if(doc["type"] == "servo"){
    int angle = doc["angle"];
    head.servo.setTarget(angle);
  }else if(doc["type"] == "rpm"){
    rpm_l = doc["rpm_l"];
    rpm_r = doc["rpm_r"];
  }  
=======
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  rpm_l = doc["rpm_l"];
  rpm_r = doc["rpm_r"];

  Serial.print(rpm_l);
  Serial.print("\t");
  Serial.println(rpm_r);
}

void requestEvent()
{
  StaticJsonDocument<200> doc;
  doc["rpm_l"] = rpm_l;
  doc["rpm_r"] = rpm_r;
  serializeJson(doc, Wire);
  Serial.println("Event requested:");
>>>>>>> parent of 352ced0... Matlab files added
}

void loop()
{

  driver.update();
  head.update();
<<<<<<< HEAD
  driver.driveRPM(rpm_l, rpm_r);
=======
  if (rpm_l != 0)
  {
    driver.driveCM(rpm_l);
    rpm_l = 0;
  }

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
>>>>>>> parent of 352ced0... Matlab files added
}
