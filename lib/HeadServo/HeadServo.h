#ifndef HEADSERVO_H
#define HEADSERVO_H
#include "Arduino.h"
#include <Servo.h>

class HeadServo
{
private:
    Servo servo;
    int SERVO_PIN;

    boolean movement_done = true;
    boolean settled = true;

    unsigned long last_step_time;
    unsigned int min_step_time = 50;
    unsigned int min_rest_time = 100;

public:
    int MIN_ANGLE;
    int MAX_ANGLE;
    int angleToReach = 0;
    int angle = 0;

public:
    HeadServo();
    HeadServo(int servo_pin, int min = 0, int max = 180);
    void begin(int _stept = 50, int _restt = 100);
    void servoStep(int step_angle = 1);
    void update();
    void setTarget(int _angle);
    boolean isSettled();
};

#endif