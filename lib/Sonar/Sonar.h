#ifndef SONAR_H
#define SONAR_H

#include <Arduino.h>

class Sonar
{
    static Sonar *instance;
    static void InterruptHandler();

private:
    int ECHO_PIN;
    int TRIG_PIN;

    unsigned int SAMPLING_RATE;

    volatile unsigned long pullUp_time;
    volatile unsigned long pullLow_time;

    unsigned long last_ping_time;

    double SP_OF_SOUND = 0.0343;
    unsigned long duration = 0;
    float distance;

    boolean enabled;


public:
    volatile boolean listen;
    boolean last_ping_resolved;

private:
    void ISR_ROUTINE(void);

public:
    Sonar();
    Sonar(int echo_pin, int trig_pin, unsigned int sampling_rate = 100);
    void begin(boolean enable = true);
    void enable();
    void disable();
    void setSampleRate(unsigned int interval);
    void update();
    boolean readyToPing();
    void reset();
    void ping();
    void calculateDist();
    float getDistance();
    void discardPing();
};

#endif