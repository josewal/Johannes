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
#include <DriveUnit.h>


Motor motorLeft(6, 30, 28, 30);
Motor motorRight(7, 26, 24, 30);
float WHEEL_SPREAD_CIRC;

Encoder *Encoder::instances[2] = {NULL, NULL};
Encoder encoderLeft(3, 5, PINE, 5, 3, PINE);
Encoder encoderRight(2, 4, PINE, 4, 5, PING);

MotorController L_Controller(motorLeft, encoderLeft);
MotorController R_Controller(motorRight, encoderRight);

DriveUnit driver;

HeadServo servo(10, 0, 180);

Sonar *Sonar::instance = NULL;
Sonar sonar(21, 34, 50);

Organ head(servo, sonar);

void setup()
{
  head.servo.begin(50, 0);
  head.sonar.begin();

  Serial.begin(9600);


  L_Controller.setup(1);
  R_Controller.setup(0);
  
  L_Controller.rpm_PID_setup(6, 1, 0, 255);
  R_Controller.rpm_PID_setup(6, 1, 0, 255);

  L_Controller.steps_PID_setup(3, 2, 0, 70);
  R_Controller.steps_PID_setup(3, 2, 0, 70);

  driver.setup(L_Controller, R_Controller);
  driver.driveCM(100);
}


void loop()
{
  
  driver.update();
  head.update();

  Serial.println(driver.leftController.encoder.protected_step_count);
  
}

// if (head.state == -1)
//   {
//     if (head.max_dist_angle != -1)
//     {
//       angle = angle + head.max_dist_angle - 90;
//       rotate(head.max_dist_angle - 90);
//       head.max_dist_angle = -1;
//     }
//     else
//     {
//       if (leftController.rotated && leftController.rotated)
//       {
//         if (movedL && movedR)
//         {
//           if (leftController.isSettled() && rightController.isSettled())
//           {
//             head.setScan(10, 0, 180);
//             movedL = false;
//             movedR = false;
//           }
//         }
//         else
//         {
//           leftController.moveCM(min(100,head.max_dist - 20));
//           rightController.moveCM(min(100,head.max_dist - 20));
//           movedL = true;
//           movedR = true;
//         }
//       }
//     }
//   }