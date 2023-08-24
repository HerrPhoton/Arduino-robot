#pragma once

#include <microTuple.h>
#include <Servo.h>

class Robot
{
public:
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Initialization of all servos.
    /////////////////////////////////////////////////////////////////////////////////////////////
    Robot();

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Translation of the end of the specified leg to the specified coordinate.
    // The 0th leg is the left front, the 1st leg is the left rear,
    // 2nd leg - right front, 3rd leg - right back.
    /////////////////////////////////////////////////////////////////////////////////////////////
    void set_coordinates(float x, float y, float z, int leg);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Discretely sets the angles of the foot servos to a given angle.
    // step - The number of steps for which all servos will move to the specified position.
    // The 0th leg is the left front, the 1st leg is the left rear,
    // 2nd leg - right front, 3rd leg - right back.
    /////////////////////////////////////////////////////////////////////////////////////////////
    void set_angles(int hipDegrees, int kneeDegrees, int ankleDegrees, int leg, int step = 10);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Calculates the angles of the servos to set the end of the leg 
    // to the specified coordinates.
    /////////////////////////////////////////////////////////////////////////////////////////////
    MicroTuple<int, int, int> coords2angles(float x, float y, float z);

private:
    /////////////////////////////////////////////////////////////////////////////////////////////
    // Translation of the servo to the specified angle.
    // The 0th leg is the left front, the 1st leg is the left rear,
    // 2nd leg - right front, 3rd leg - right back.
    /////////////////////////////////////////////////////////////////////////////////////////////
    void write_servo(int num_servo, int degrees);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // Conversion from radians to degrees.
    /////////////////////////////////////////////////////////////////////////////////////////////
    inline double to_degrees(float radians);

private:
    const float len1 = 5.0; // Distance from the servo shaft of the knee to the ankle shaft (cm).
    const float len2 = 4.8; // Distance from the ankle servo shaft to the end of the leg (cm).

private:
    Servo servo[12]; // Array with all servos.

    int zeroPositions[12] = { 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90 }; // Initial positions of servos.
    int directions[12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }; // Direction of rotation of servos (1 - counterclockwise, -1 - clockwise).

    MicroTuple<int, int, int> curLegsCoords[4]; // Coordinates on which the legs are set.
};