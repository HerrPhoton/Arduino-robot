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
        curServosAngles[i] = zeroPositions[i];

        delay(100);
    }
}

void Robot::set_coordinates(float x, float y, float z, int leg)
{
    MicroTuple<float, float, float> tmp = coords2angles(x, y, z);
    set_angles(tmp.get<0>(), tmp.get<1>(), tmp.get<2>(), leg);
}

void Robot::set_angles(float hipDegrees, float kneeDegrees, float ankleDegrees, int leg)
{
    write_servo(3 * leg, hipDegrees);
    write_servo(3 * leg + 1, kneeDegrees);
    write_servo(3 * leg + 2, ankleDegrees);
}

MicroTuple<float, float, float> Robot::coords2angles(float x, float y, float z)
{
    float h1 = atan(x / z);
    float z2 = z / cos(h1);
    float h2 = acos((pow(len1, 2) + pow(z2, 2) - pow(len2, 2)) / 2);
    float k0 = acos((pow(len1, 2) + pow(len2, 2) - pow(z2, 2)) / 2);

    float hipDegrees = to_degrees(h1 + h2);
    float kneeDegrees = 180 - to_degrees(k0);
    float ankleDegrees;

    return { hipDegrees, kneeDegrees, ankleDegrees };
}

void Robot::write_servo(int num_servo, float degrees)
{
    //servo[num_servo].write(zeroPositions[num_servo] + directions[num_servo] * degrees);
    float angle = directions[num_servo] * degrees;

    servo[num_servo].write(angle);
    curServosAngles[num_servo] = angle;
}

inline double Robot::to_degrees(float radians)
{
    return radians * (180.0 / M_PI);
}