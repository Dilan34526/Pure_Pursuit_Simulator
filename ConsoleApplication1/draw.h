#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "vectors.h"

using namespace cv;

extern float angleChange(float angle);

extern Mat drawPath(Mat& img, std::vector<float> x, std::vector<float> y);

extern void drawRobot(Mat img, vector3D pos, vector2D lookaheadPt, float leftWheelVel, float rightWheelVel);