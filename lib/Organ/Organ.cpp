#include <Organ.h>

Organ::Organ(HeadServo _servo, Sonar _sonar)
{
    sonar = _sonar;
    servo = _servo;
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
        if (sonar.readyToPing() && servo.isSettled())
        {
            sonar.ping();

            angle_pinged_at = servo.angle;

            state++;
        }
        break;

    case 2:
        //EXPECTING PING ECHO
        if (sonar.got_echo)
        {
            sonar.calculateDist();
            distance = sonar.getDistance();

            if (distance < min_dist)
            {
                min_dist = distance;
                min_dist_angle = angle_pinged_at;
            }
            if (distance > max_dist)
            {
                max_dist = distance;
                max_dist_angle = angle_pinged_at;
            }

            if (angle_pinged_at != scan_end)
            {
                state++;
            }
            else
            {
                state = -1;
            }
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


    sonar.reset();
    angle_pinged_at = -1;

    min_dist = 999;
    min_dist_angle = 0;
    max_dist = 0;
    max_dist_angle = 0;
    distance = 999;
}