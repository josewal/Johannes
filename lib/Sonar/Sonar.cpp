#include <Sonar.h>

void Sonar::InterruptHandler()
{
    if(Sonar::instance!=NULL){
        Sonar::instance->ISR_ROUTINE();
    }
}

Sonar::Sonar() {
}
Sonar::Sonar(int echo_pin, int trig_pin, unsigned int sampling_rate = 100)
{

    ECHO_PIN = echo_pin;
    TRIG_PIN = trig_pin;
    SAMPLING_RATE = sampling_rate;
}

void Sonar::begin(boolean enable = true)
{
    pinMode(ECHO_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), InterruptHandler, CHANGE);
    instance = this;
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

    listen = false;
    enabled = true;
}

void Sonar::disable()
{
    listen = false;
    enabled = false;
}

void Sonar::reset(){
    disable();
    enable();
}

void Sonar::setSampleRate(unsigned int interval)
{
    SAMPLING_RATE = interval;
}

float Sonar::getDistance()
{
    return distance;
}

boolean Sonar::readyToPing()
{
    // Serial.print("Am I Ready: ");
    // Serial.print("\t");

    boolean time_elapsed = millis() - last_ping_time > SAMPLING_RATE;
    // Serial.println(last_ping_resolved && time_elapsed);
    return last_ping_resolved && time_elapsed;

}

void Sonar::calculateDist()
{
    // Serial.println("Calculated dist ");
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
    last_ping_resolved = true;
}

void Sonar::ping()
{
    // Serial.println("Pinged ");
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
