#ifndef ORGAN_H
#define ORGAN_H

#include <Arduino.h>
#include <HeadServo.h>
#include <Sonar.h>

class Organ
{
private:
public:
    float min_dist = 999;
    float max_dist = 0;
    int min_dist_angle = 0;
    int max_dist_angle = 0;
    int obstacle_angle = 0;

    int angle_pinged_at;
    float distance = 0;

    unsigned int scan_start;
    unsigned int scan_end;
    unsigned int scan_step;

    int pingCycle = -1;
    int state = -1;
    /* states:
    -1 off
    1 pinging at
    2 scanning
    */

    HeadServo servo;
    Sonar sonar;

    boolean scan_done = true;
    boolean ping_needed = false;
    boolean ping_expected = false;

    Organ(HeadServo _servo, Sonar _sonar);
    void begin();
    void update();

    void pingAt(int angle, int n_pings = -1, int interval = -1);
    void setScan(int step, int from, int to);
    void scan();
};

#endif