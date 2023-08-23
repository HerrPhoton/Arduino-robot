#pragma once

#include <microTuple.h>
#include <Servo.h>

class Robot
{
public:
    ////////////////////////////////////////////////////////////////////////////////
    // Initialization of all servos
    ////////////////////////////////////////////////////////////////////////////////
    Robot();

    ////////////////////////////////////////////////////////////////////////////////
    // Translation of the end of the specified leg to the specified coordinate.
    // The 0th leg is the left front, the 1st leg is the left rear,
    // 2nd leg - right front, 3rd leg - right back
    ////////////////////////////////////////////////////////////////////////////////
    void set_coordinates(float x, float y, float z, int leg);

    ////////////////////////////////////////////////////////////////////////////////
    // Translation of the foot servos to the specified angle.
    // The 0th leg is the left front, the 1st leg is the left rear,
    // 2nd leg - right front, 3rd leg - right back
    ////////////////////////////////////////////////////////////////////////////////
    void set_angles(float hipDegrees, float kneeDegrees, float ankle, int leg);

    ////////////////////////////////////////////////////////////////////////////////
    // Calculates the angles of the servos to set the end of the leg 
    // to the specified coordinates.
    ////////////////////////////////////////////////////////////////////////////////
    MicroTuple<float, float, float> coords2angles(float x, float y, float z);

private:
    ////////////////////////////////////////////////////////////////////////////////
    // Translation of the servo to the specified angle.
    // The 0th leg is the left front, the 1st leg is the left rear,
    // 2nd leg - right front, 3rd leg - right back
    ////////////////////////////////////////////////////////////////////////////////
    void write_servo(int num_servo, float degrees);

    ////////////////////////////////////////////////////////////////////////////////
    // Conversion from radians to degrees.
    ////////////////////////////////////////////////////////////////////////////////
    inline double to_degrees(float radians);

private:
    const float len1; // Distance from the servo shaft of the knee to the ankle shaft (cm)
    const float len2; // Distance from the ankle servo shaft to the end of the leg (cm)

private:
    Servo servo[12]; // Array with all servos

    int zeroPositions[12]; // Initial positions of servos
    int directions[12]; // Direction of rotation of servos (1 - counterclockwise, -1 - clockwise)

    int curServosAngles[12]; // The angles at which the servos are currently rotated
    MicroTuple<float, float, float> curLegsCoords[4]; // Coordinates on which the legs are set
};