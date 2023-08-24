#pragma once
#define _USE_MATH_DEFINES

#include <math.h>
#include "Robot.h"

Robot::Robot()
{
    // Connecting servos      
    for (int i = 0; i < 12; i++)
    {
        servo[i].attach(i + 2);
        delay(100);
    }

    // Setting all servos to the initial position
    for (int i = 0; i < 12; i++)
    {
        servo[i].write(zeroPositions[i]);
        delay(100);
    }
}

void Robot::set_coordinates(float x, float y, float z, int leg)
{
    MicroTuple<int, int, int> tmp = coords2angles(x, y, z);
    set_angles(tmp.get<0>(), tmp.get<1>(), tmp.get<2>(), leg);
}

void Robot::set_angles(int hipDegrees, int kneeDegrees, int ankleDegrees, int leg, int step = 10)
{
    // Determining the step size for each servo.
    int hipStep = (servo[3 * leg].read() - hipDegrees) / step;
    int kneeStep = (servo[3 * leg + 1].read() - kneeDegrees) / step;
    int ankleStep = (servo[3 * leg + 2].read() - ankleDegrees) / step;

    // Moving the servos to the required angle.
    for (int i = 0; i < step; i++)
    {
        write_servo(3 * leg, servo[3 * leg].read() + hipStep);
        write_servo(3 * leg + 1, servo[3 * leg + 1].read() + kneeStep);
        write_servo(3 * leg + 2, servo[3 * leg + 2].read() + ankleStep);
    }

    // Moving the servos to the required angle in case of error.
    if (servo[3 * leg].read() - hipDegrees != 0)
        write_servo(3 * leg, hipDegrees);

    if (servo[3 * leg + 1].read() - kneeDegrees != 0)
        write_servo(3 * leg + 1, kneeDegrees);

    if (servo[3 * leg + 2].read() - ankleDegrees != 0)
        write_servo(3 * leg + 2, ankleDegrees);
}

MicroTuple<int, int, int> Robot::coords2angles(float x, float y, float z)
{
    float h1 = atan(x / z);
    float z2 = z / cos(h1);
    float h2 = acos((pow(len1, 2) + pow(z2, 2) - pow(len2, 2)) / 2);
    float k0 = acos((pow(len1, 2) + pow(len2, 2) - pow(z2, 2)) / 2);

    int hipDegrees = 90;
    int kneeDegrees = to_degrees(h1 + h2);
    int ankleDegrees = 180 - to_degrees(k0);

    return { hipDegrees, kneeDegrees, ankleDegrees };
}

void Robot::write_servo(int num_servo, int degrees)
{
    //servo[num_servo].write(zeroPositions[num_servo] + directions[num_servo] * degrees);
    servo[num_servo].write(directions[num_servo] * degrees);
}

inline double Robot::to_degrees(float radians)
{
    return radians * (180.0 / M_PI);
}