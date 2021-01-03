#include <MotorController.h>
#include <PID_v1.h>

MotorController::MotorController(Motor mtr, Encoder enc)
{
    PID temp_rpm_PID(&rpm_input, &rpm_PID_output, &rpm_setpoint, 0, 0, 0, DIRECT);
    rpm_PID = temp_rpm_PID;

    PID temp_steps_PID(&steps_input, &steps_PID_output, &steps_setpoint, 0, 0, 0, DIRECT);
    steps_PID = temp_steps_PID;

    motor = mtr;
    encoder = enc;
}

void MotorController::setup(int dir = 1)
{
    encoder.setDirection(dir);
    encoder.begin();
    motor.begin();
}

void MotorController::rpm_PID_setup(double Kp, double Ki, double Kd, int limits = 255)
{
    rpm_input = 0;
    rpm_setpoint = 0;
    rpm_PID.SetTunings(Kp, Ki, Kd);

    rpm_PID.SetOutputLimits(-limits, limits);
    rpm_PID.SetSampleTime(50);
    rpm_PID.SetMode(AUTOMATIC);
}

void MotorController::steps_PID_setup(double Kp, double Ki, double Kd, int limits = 255)
{
    steps_input = 0;
    steps_setpoint = 0;
    steps_PID.SetTunings(Kp, Ki, Kd);

    steps_PID.SetOutputLimits(-limits, limits);
    steps_PID.SetSampleTime(50);
    steps_PID.SetMode(AUTOMATIC);
}

void MotorController::update()
{
    encoder.update();
    rpm_input = encoder.rpm;
    steps_input = encoder.protected_step_count;

    steps_PID.Compute();
    rpm_PID.Compute();

    motor.drive(rpm_PID_output);
}

void MotorController::rpm_Control(int desired_rpm)
{
    rpm_setpoint = desired_rpm;
    rpm_PID.Compute();
}

void MotorController::step_Control(int desired_steps)
{
    steps_setpoint = desired_steps;
    steps_PID.Compute();
}

void MotorController::moveRPM(int rpm, int t = 0, int acl = 0, int dcl = 0)
{
    rpm_setpoint = rpm;
}
