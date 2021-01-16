#include <DriveUnit.h>

DriveUnit::DriveUnit()
{
}

void DriveUnit::setup(MotorController _left, MotorController _right)
{
    leftController = _left;
    rightController = _right;
}

void DriveUnit::update()
{
    leftController.update();
    rightController.update();
    hasArrived();
    hasRotated();
}

boolean DriveUnit::hasArrived()
{
    if (!rotationDone && leftController.isSettled() && rightController.isSettled())
    {
        dist = dist_to_reach;
        arrived = true;
    }
    else
    {
        arrived = false;
    }
    return arrived;
}

boolean DriveUnit::hasRotated()
{
    if (!rotationDone && leftController.isSettled() && rightController.isSettled())
    {
        angle = angle_to_reach;
        rotationDone = true;
    }
    else
    {
        rotationDone = false;
    }
    return rotationDone;
}

void DriveUnit::driveCM(int _cm)
{
    arrived = false;
    dist_to_reach = _cm;

    leftController.moveCM(dist_to_reach);
    rightController.moveCM(dist_to_reach);
}

void DriveUnit::rotateTo(int _angle)
{
    rotationDone = false;
    angle_to_reach = _angle;
    float dist = _angle / 360 * WHEEL_SPREAD_CIRC;

    leftController.moveCM(dist);
    rightController.moveCM(-dist);
}