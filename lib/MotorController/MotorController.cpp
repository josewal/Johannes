#include <MotorController.h>

MotorController::MotorController(int a_pin, int a_port, int b_pin, int b_port)
{
    A_PIN = a_pin;
    A_PORT = a_port;
    B_PIN = b_pin;
    B_PORT = b_port;
}

void MotorController::begin()
{
}
