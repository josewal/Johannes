#include <Organ.h>

Organ::Organ(HeadServo _servo, SharpIR _sensor)
{
    servo = _servo;
    sensor = _sensor;
}

void Organ::begin()
{
}

void Organ::update()
{
    servo.update();
    
    switch (state)
    {
    case 0:
        //STATE COMMANDER
        break;
        
    case 1:
        //SENDING PING
        if (servo.isSettled())
        {
            dists[constrain(servo.angle,0,179)] = sensor.getDistance();
            Serial.println(dists[constrain(servo.angle,0,179)]);
            lastPingAt = servo.angle;
        }
        

    case 2:
            if (lastPingAt != scan_end)
                {
                    state++;
                }
                else
                {
                    state = -1;
                }
        break;

    case 3:
        //MOVING
        int angle = servo.angle;
        if (angle > scan_end)
        {
            servo.setTarget(max(scan_end, angle - scan_step));
        }
        else if (angle < scan_end)
        {
            servo.setTarget(min(scan_end, angle + scan_step));
        }
        state = 1;
        break;

    default:
        break;
    }
}

void Organ::setScan(int step, int from, int to = 0)
{
    state = 1;

    if (step == 0)
    {
        step = 0;
        scan_end = constrain(from, servo.MIN_ANGLE, servo.MAX_ANGLE);
        scan_start = from;
    }
    else
    {
        scan_step = step;
        scan_start = max(servo.MIN_ANGLE, from);
        scan_end = min(servo.MAX_ANGLE, to);
    }
    servo.setTarget(scan_start);
    lastPingAt = scan_start;
    last_measurment = millis();
}