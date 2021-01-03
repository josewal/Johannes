#include <Motor.h>

Motor::Motor() {}
Motor::Motor(int pwm_pin, int dir_a_pin, int dir_b_pin, int min_pwm = 0, int max_pwm = 255)
{
    PWM_PIN = pwm_pin;
    DIR_A_PIN = dir_a_pin;
    DIR_B_PIN = dir_b_pin;

    MIN_PWM = min_pwm;
    MAX_PWM = max_pwm;
}

void Motor::begin()
{
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_A_PIN, OUTPUT);
    pinMode(DIR_B_PIN, OUTPUT);
}

void Motor::drive(int velocity)
{
    velocity = constrain(velocity, -MAX_PWM, MAX_PWM);

    if (velocity > MIN_PWM)
    {
        analogWrite(PWM_PIN, velocity);
        digitalWrite(DIR_A_PIN, HIGH);
        digitalWrite(DIR_B_PIN, LOW);
    }
    else if (velocity < -MIN_PWM)
    {
        velocity = -velocity;
        analogWrite(PWM_PIN, velocity);
        digitalWrite(DIR_A_PIN, LOW);
        digitalWrite(DIR_B_PIN, HIGH);
    }
    else
    {
        analogWrite(PWM_PIN, 0);
        digitalWrite(DIR_A_PIN, LOW);
        digitalWrite(DIR_B_PIN, LOW);
    }
}

;