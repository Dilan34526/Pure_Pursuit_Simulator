#include "odom.h"

float convertToVelocity(float velocity) {
    return (60 * velocity) / (2 * M_PI * 2);
}

float convertToVoltage(float velocity) {
    float wheel_velocity = (60 * velocity) / (2 * M_PI * 2);
    return (wheel_velocity / 200) * 127;
}

vector3D odom(vector3D pos, float leftWheelVelocity, float rightWheelVelocity) {
    float deltaLeftDistance = 0.01 * 0.1047 * leftWheelVelocity * 8;
    float deltaRightDistance = 0.01 * 0.1047 * rightWheelVelocity * 8;

    float deltaAverageDistance = (deltaLeftDistance + deltaRightDistance) / 2;
    float deltaHeading = (deltaRightDistance - deltaLeftDistance) / 12;

    float deltaX = deltaAverageDistance * cos(pos.z + (deltaHeading / 2));
    float deltaY = deltaAverageDistance * sin(pos.z + (deltaHeading / 2));

    pos.x = pos.x + deltaX;
    pos.y = pos.y + deltaY;
    pos.z = pos.z + deltaHeading;
    return pos;
}