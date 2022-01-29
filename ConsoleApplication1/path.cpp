#include "path.h"
#include "math.h"
#include <iostream>

float roundTo(float num) {

    return std::ceil(num * 1000.0) / 1000.0;

}

std::vector<hermite> path::getRobotPath() { return robotPath; }

hermite path::getStartPoint() { if (robotPath.size() > 0) { return robotPath.front(); } }
hermite path::getEndPoint() { return robotPath.at(robotPath.size() - 1); }

/**
    * Initializes the path
    *
    * @param maxVel   maximum robot velocity
    * @param maxAccel maximum robot acceleration
    * @param maxVelk  maximum turning velocity (between 1-5)
    */
void path::initializePath(float maxVel, float maxAccel, float maxVelk) {
    setCurvatures();
    setDistances();
    setTargetVelocities(maxVel, maxAccel, maxVelk);
}

void path::addSegment(vector3D start, vector3D end) {
    std::vector<hermite> injectTemp;
    injectTemp = injectPoints(start, end);
    robotPath.insert(robotPath.end(), injectTemp.begin(), injectTemp.end());
}

std::vector<hermite> path::injectPoints(vector3D startPt, vector3D endPt) {

    std::vector<hermite> temp;

    float dist = startPt.dist(endPt);

    float mx0 = roundTo(cosf(startPt.z * (M_PI / 180.0)) * dist * 2);
    float my0 = roundTo(sinf(startPt.z * (M_PI / 180.0)) * dist * 2);
    float mx1 = roundTo(cosf(endPt.z * (M_PI / 180.0)) * dist * 2);
    float my1 = roundTo(sinf(endPt.z * (M_PI / 180.0)) * dist * 2);

    float ax = (2 * startPt.x) - (2 * endPt.x) + mx1 + mx0;
    float bx = (-2 * mx0) - (mx1)-(3 * startPt.x) + (3 * endPt.x);
    float cx = mx0;
    float dx = startPt.x;

    float ay = (2 * startPt.y) - (2 * endPt.y) + my1 + my0;
    float by = (-2 * my0) - (my1)-(3 * startPt.y) + (3 * endPt.y);
    float cy = my0;
    float dy = startPt.y;

    for (float t = 0; t < 1.01; t += 0.01) {

        float xt = (ax * pow(t, 3)) + (bx * pow(t, 2)) + (cx * t) + dx;
        float yt = (ay * pow(t, 3)) + (by * pow(t, 2)) + (cy * t) + dy;

        vector2D vector = vector2D(xt, yt);
        hermite herm = hermite(vector);
        temp.push_back(herm);
    }
    return temp;
}

float path::calculatePathCurvature(std::vector<hermite> path, int pointIndex) {
    vector2D point = path.at(pointIndex).point;
    vector2D prevPoint = path.at(pointIndex - 1).point;
    vector2D nextPoint = path.at(pointIndex + 1).point;

    float distanceOne = point.dist(prevPoint);
    float distanceTwo = point.dist(nextPoint);
    float distanceThree = nextPoint.dist(prevPoint);

    float productOfSides = distanceOne * distanceTwo * distanceThree;
    float semiPerimeter = (distanceOne + distanceTwo + distanceThree) / 2;

    float changeInDistanceOne = roundTo(semiPerimeter - distanceOne);
    float changeInDistanceTwo = roundTo(semiPerimeter - distanceTwo);
    float changeInDistanceThree = roundTo(semiPerimeter - distanceThree);


    float triangleArea = sqrt(semiPerimeter * (changeInDistanceOne) * (changeInDistanceTwo) * (changeInDistanceThree));

    float radius = (productOfSides) / (4 * triangleArea);

    float curvature = 1000000000000000000;

    if (radius != 0) {
        curvature = 1 / radius;
    }

    return curvature;
}

float path::calculateMaxVelocity(std::vector<hermite> path, int point, float pathMaxVel, float k) {
    if (point > 0) {
        float curvature = calculatePathCurvature(path, point);
        return std::min(pathMaxVel, k / curvature); //k is a constant (generally between 1-5 based on how quickly you want to make the turn)
    }
    return pathMaxVel;
}


void path::setCurvatures() {
    getStartPoint().setCurvature(0);
    getEndPoint().setCurvature(0);
    for (int i = 1; i < robotPath.size() - 1; i++) {
        robotPath.at(i).setCurvature(calculatePathCurvature(robotPath, i));

    }
}

void path::setDistances() {
    float distance = 0;
    getStartPoint().setDistance(0);
    std::vector<hermite> robotPathCopy = robotPath;
    for (int i = 1; i < robotPathCopy.size(); i++) {
        distance += robotPathCopy.at(i).point.dist(robotPathCopy.at(i - 1).point);
        robotPath.at(i).setDistance(distance);
    }
}

void path::setTargetVelocities(float maxVel, float maxAccel, float k) {
    robotPath.at(robotPath.size() - 1).setVelocity(0);
    for (int i = robotPath.size() - 2; i >= 0; i--) {
        float distance = robotPath.at(i + 1).point.dist(robotPath.at(i).point);
        float maxReachableVel = sqrt(pow(robotPath.at(i + 1).getVelocity(), 2) + (2 * maxAccel * distance));
        robotPath.at(i).setVelocity(std::min(calculateMaxVelocity(robotPath, i, maxVel, k), maxReachableVel));
    }
}

