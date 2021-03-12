#ifndef ORGAN_H
#define ORGAN_H

#include <Arduino.h>
#include <HeadServo.h>
#include <SharpIR.h>


class Organ
{
private:
public:

    unsigned int dists[180];
    unsigned int lastPingAt = 0;
    unsigned long last_measurment = 0;
    unsigned int scan_start;
    unsigned int scan_end;
    unsigned int scan_step;

    int state = -1;

    SharpIR sensor;
    HeadServo servo;

    Organ(HeadServo _servo, SharpIR _sensor);
    void begin();
    void update();

    void pingAt(int angle, int n_pings = -1, int interval = -1);
    void setScan(int step, int from, int to = 0);
    void scan();
};

#endif