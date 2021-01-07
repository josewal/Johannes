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
    case -1:
        break;

    case 1:
        scan();
        break;

    case 2:
        if (servo.isSettled())
        {
            if (sonar.readyToPing() && ping_needed)
            {
                sonar.ping();
                angle_pinged_at = servo.angle;

                ping_needed = false;
                ping_expected = true;
            }
        }

        if (!sonar.last_ping_resolved)
        {
            if (ping_expected)
            {
                sonar.calculateDist();
                distance = sonar.getDistance();
                ping_expected = false;
                state = -1;
            }
            else
            {
                // sonar.discardPing();
            }
        }
        break;

    default:
        break;
    }
}

void Organ::scan()
{
    if (angle_pinged_at == scan_end && sonar.last_ping_resolved && !sonar.listen)
    {
        // Serial.print("Scan done at: ");
        // Serial.print(servo.angle);
        // Serial.print(", last ping distance: ");
        // Serial.print(distance);
        // Serial.print(", at angle ");
        // Serial.println(angle_pinged_at);
        scan_done = true;
        state = -1;
    }

    if (!sonar.last_ping_resolved)
    {
        if (!ping_expected)
        {
            sonar.discardPing();
        }
        else
        {
            sonar.calculateDist();
            distance = sonar.getDistance();
            // Serial.print("Resolved ping at: ");
            // Serial.print(servo.angle);
            // Serial.print(" for ping angle of: ");
            // Serial.println(angle_pinged_at);
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
        }
    }

    if (servo.isSettled())
    {
        if (angle_pinged_at != servo.angle)
        {
            ping_needed = true;
        }

        if (sonar.readyToPing() && ping_needed)
        {
            sonar.ping();
            angle_pinged_at = servo.angle;

            // Serial.print("Pinged at angle: ");
            // Serial.print(angle_pinged_at);
            // Serial.println("!!!");

            ping_needed = false;
            ping_expected = true;
        }

        if (!sonar.listen && !ping_needed)
        {
            int angle = servo.angle;
            if (angle > scan_end)
            {
                servo.setTarget(max(scan_end, angle - scan_step));
            }
            else if (angle < scan_end)
            {
                servo.setTarget(min(scan_end, angle + scan_step));
            }
        }
    }
}

void Organ::setScan(int step, int from, int to)
{
    state = 1;

    scan_start = max(servo.MIN_ANGLE, from);
    servo.setTarget(scan_start);

    sonar.reset();

    scan_end = min(servo.MAX_ANGLE, to);

    scan_step = step;
    scan_done = false;
    angle_pinged_at = -1;

    min_dist = 999;
    min_dist_angle = 0;
    distance = 999;
    ping_needed = true;
}

void Organ::pingAt(int _angle, int _n_pings = -1, int _interval = -1)
{
    state = 2;

    int angle = constrain(_angle, servo.MIN_ANGLE, servo.MAX_ANGLE);
    servo.setTarget(angle);

    sonar.reset();

    scan_end = -1;

    scan_step = -1;
    scan_done = true;
    angle_pinged_at = -1;

    min_dist = 999;
    min_dist_angle = 0;
    distance = 999;
    ping_needed = true;
}