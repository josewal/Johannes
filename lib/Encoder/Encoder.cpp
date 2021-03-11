#include <Encoder.h>
 
void Encoder::InterruptHandler0()
{
    if (Encoder::instances[0] != NULL)
        Encoder::instances[0]->ISR_ROUTINE0();
} // end of myClass::switchPressedExt0

void Encoder::InterruptHandler1()
{
    if (Encoder::instances[1] != NULL)
        Encoder::instances[1]->ISR_ROUTINE1();
} // end of myClass::switchPressedExt1


Encoder::Encoder(int a_pin, int a_port, int a_reg, int b_pin, int b_port, int b_reg)
{
    A_PIN = a_pin;
    A_PORT = a_port;
    A_REG = a_reg;
    B_PIN = b_pin;
    B_PORT = 3;
    B_REG = 0x0C;
}

void Encoder::begin()
{
    pinMode(A_PIN, INPUT_PULLUP);
    pinMode(B_PIN, INPUT_PULLUP);

    switch (A_PIN)
    {
    case 3:
        attachInterrupt(digitalPinToInterrupt(A_PIN), InterruptHandler0, RISING);
        instances[0] = this;
        break;

    case 2:
        attachInterrupt(digitalPinToInterrupt(A_PIN), InterruptHandler1, RISING);
        instances[1] = this;
        break;
    }
}

void Encoder::getData()
{
    noInterrupts();

    if ((micros() - prev_time) >= 100000)
    {
        step_time = 100000;
        prev_time = micros();
    }

    protected_step_count = step_count;
    protected_step_time = constrain(step_time, -100000, 100000);

    interrupts();
}

void Encoder::calculateRPM()
{
    if (protected_step_time < 100000)
    {
        rpm = -60000000 / (75 * protected_step_time);
        rpm = rpm/3;
    }
    else
    {
        rpm = 0;
    }
}

void Encoder::update(){
    Encoder::getData();
    Encoder::calculateRPM();
}

void Encoder::setDirection(int dir)
{
    DIR_PARAMETER = dir;
}

void Encoder::ISR_ROUTINE0()
{
    if (bitRead(PINE, 3) == DIR_PARAMETER)
    {
        step_count++;
        step_time = (micros() - prev_time);
    }

    else
    {
        step_count--;
        step_time = -(micros() - prev_time);
    }

    prev_time = micros();
};

void Encoder::ISR_ROUTINE1()
{
    if (bitRead(PING, 5) == DIR_PARAMETER)
    {
        step_count++;
        step_time = (micros() - prev_time);
    }

    else
    {
        step_count--;
        step_time = -(micros() - prev_time);
    }

    prev_time = micros();
};