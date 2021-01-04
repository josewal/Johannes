#include <Sonar.h>

void Sonar::InterruptHandler()
{
    Sonar::instance->ISR_ROUTINE();
}

Sonar::Sonar(int echo_pin, int trig_pin, unsigned int sampling_rate = 100)
{
    ECHO_PIN = echo_pin;
    TRIG_PIN = trig_pin;
    SAMPLING_RATE = sampling_rate;
    instance = this;
}

void Sonar::begin(boolean enable = true)
{
    pinMode(ECHO_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), InterruptHandler, CHANGE);
    enabled = false;
    if (enable)
    {
        Sonar::enable();
    }

}

void Sonar::enable()
{
    if (enabled)
    {
        return;
    }

    pullUp_time = 0;
    pullLow_time = 0;

    last_ping_resolved = true;
    last_ping_time = millis();
    distance = 999;

    listen = true;
    enabled = true;
}

void Sonar::disable()
{
    listen = false;
    enabled = false;
}

void Sonar::setSampleRate(unsigned int interval)
{
    SAMPLING_RATE = interval;
}

float Sonar::getDistance()
{
    return distance;
}

void Sonar::update()
{
    if (!enabled)
    {
        Serial.print("Not enabled");
        return;
    }

    if (!last_ping_resolved)
    {
        Sonar::calculateDist();
        last_ping_resolved = true;
        
    }

    if (Sonar::readyToPing())
    {
        Sonar::ping();
    }
}

boolean Sonar::readyToPing(){
    boolean time_elapsed = millis() - last_ping_time > SAMPLING_RATE;
    return last_ping_resolved && time_elapsed;
}

void Sonar::calculateDist()
{
    duration = pullLow_time - pullUp_time;
    pullUp_time = 0;
    pullLow_time = 0;

    if ((duration < 38000) && (duration != 0))
    {
        distance = (duration / 2) * SP_OF_SOUND;
    }
    else
    {
        distance = 999;
    }
}

void Sonar::ping()
{
    listen = true;
    last_ping_time = millis();

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
}

void Sonar::ISR_ROUTINE(void)
{
    if (!listen)
    {
        return;
    }

    if (pullUp_time == 0)
    {
        pullUp_time = micros();
    }
    else
    {
        pullLow_time = micros();
        last_ping_resolved = false;
        listen = false;
    }
}
