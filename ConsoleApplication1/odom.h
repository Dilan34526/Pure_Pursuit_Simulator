#pragma once

#include <vector>
#include "vectors.h"

extern float convertToVelocity(float velocity);

extern float convertToVoltage(float velocity);

extern vector3D odom(vector3D pos, float leftWheelVelocity, float rightWheelVelocity);