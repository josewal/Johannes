#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
#include <Arduino.h>

class MotorController
{
public:
    int A_PIN;
    int A_PORT;
    int B_PIN;
    int B_PORT;

    MotorController(int a_pin, int a_port, int b_pin, int b_port);
    void begin();
};

#endif