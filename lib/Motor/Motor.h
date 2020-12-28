#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

class Motor
{
public:
    int PWM_PIN;
    int DIR_A_PIN;
    int DIR_B_PIN;

    int MIN_PWM;
    int MAX_PWM;

    Motor(int pwm_pin, int dir_a_pin, int dir_b_pin, int min_pwm = 0, int max_pwm = 255);
    void begin();
    void drive(int velocity);

};

#endif
