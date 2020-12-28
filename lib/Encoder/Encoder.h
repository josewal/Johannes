#ifndef ENCODER_H
#define ENCODER_H
#include "Arduino.h"

class Encoder
{
    static Encoder *instances[2];
    static void InterruptHandler0();
    static void InterruptHandler1();

    //VARIABLES DECLARATION
private:
    int A_PIN;
    int A_PORT;
    int A_REG;
    int B_PIN;
    int B_PORT;
    int B_REG;

    volatile int dir;

    volatile int step_count;
    volatile long step_time;
    volatile unsigned long prev_time;

public:
    int protected_dir;
    int protected_step_count;
    long protected_step_time;
    int RPM;

    //METHODS DECLARATION
private:
public:
    Encoder(int a_pin, int a_port, int a_reg, int b_pin, int b_port, int b_reg);
    void begin();
    void getData();
    void ISR_ROUTINE0(void);
    void ISR_ROUTINE1(void);
};

#endif