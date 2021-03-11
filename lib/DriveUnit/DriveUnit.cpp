#include <DriveUnit.h>

DriveUnit::DriveUnit(MotorController &leftController, MotorController &rightController) : leftController(leftController), rightController(rightController)
{
}

void DriveUnit::update()
{
    leftController.update();
    rightController.update();
    
    switch (state)
    {
    case 1:
        break;
    case 2:
        hasArrived();
        hasRotated();
        break;
    default:
        break;
    }
}

void DriveUnit::stop()
{
    leftController.stop();
    rightController.stop();
}

boolean DriveUnit::hasArrived()
{

    if (!arrived)
    {
        if (leftController.settled && leftController.settled)
        {
            dist = dist_to_reach;
            arrived = true;
            stop();
        }
        else
        {
            arrived = false;
        }
    }
    else
    {
        arrived = true;
    }

    return arrived;
}

boolean DriveUnit::hasRotated()
{

    if (!rotationDone)
    {
        if (leftController.settled && rightController.settled)
        {
            rotationDone = true;
            stop();
        }
        else
        {
            rotationDone = false;
        }
    }

    return rotationDone;
}

void DriveUnit::driveCM(int _cm)
{
    state = 2;
    arrived = false;
    dist_to_reach = dist + _cm;

    leftController.moveCM(_cm);
    rightController.moveCM(_cm);
}

void DriveUnit::driveRPM(int _left, int _right = 999)
{
    state = 1;
    arrived = false;
    leftController.moveRPM(_left);
    if (_right == 999)
    {
        rightController.moveRPM(_left);
    }
    else
    {
        rightController.moveRPM(_right);
    }
}

void DriveUnit::rotateTo(float _angle)
{
    rotationDone = false;
    float _dist = (_angle / 360) * WHEEL_SPREAD_CIRC;
    leftController.moveCM(_dist);
    rightController.moveCM(-_dist);
}

void DriveUnit::rotateBy(float _angle)
{
    rotateTo(_angle);
}