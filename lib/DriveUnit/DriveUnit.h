#ifndef DRIVEUNIT_H
#define DRIVEUNIT_H

#include <Arduino.h>
#include <MotorController.h>

class DriveUnit
{
private:
    const float WHEEL_SPREAD = 17;
    float WHEEL_SPREAD_CIRC = WHEEL_SPREAD * PI;

public:
    boolean rotationDone = true;
    int angle_to_reach = 0;
    int angle = 0;

    boolean arrived = true;
    int dist_to_reach = 0;
    int dist = 0;

    MotorController &leftController;
    MotorController &rightController;

    DriveUnit::DriveUnit(MotorController& , MotorController&);

    void update();
    boolean hasRotated();
    boolean hasArrived();
    void stop();
    void driveCM(int);
    void driveRPM(int, int = 999);
    void rotateBy(float);
    void rotateTo(float);
};

#endif