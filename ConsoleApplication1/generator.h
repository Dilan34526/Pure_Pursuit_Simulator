#pragma once
#include <vector>
#include "path.h"
#include "vectors.h"

extern class generator {

    std::vector<vector3D> points;
    float maxVel, maxAccel, maxVelk;
    bool forward;

public:
    generator() {
        maxVel = 0;
        maxAccel = 0;
        maxVelk = 0;
        forward = true;
    }

    generator(bool fwd) {
        maxVel = 0;
        maxAccel = 0;
        maxVelk = 0;
        forward = fwd;
    }

    void setVelocities(float maxVel1, float maxAccel1, float maxVelk1) {
        maxVel = maxVel1;
        maxAccel = maxAccel1;
        maxVelk = maxVelk1;
    }

    void addPoint(vector3D point) {
        points.push_back(point);
    }

    void addPoints(std::vector<vector3D> points1) {
        points.insert(points.end(), points1.begin(), points.end());
    }

    path generatePath() {
        path path1 = path(forward);
        for (int i = 0; i < points.size() - 1; ++i) {
            path1.addSegment(points.at(i), points.at(i + 1));
        }
        path1.initializePath(maxVel, maxAccel, maxVelk);
        /*for (int i = 0; i < path1.getRobotPath().size(); i++) {
            std::cout << path1.getRobotPath().at(i).point.x << "\n";
        }*/
        return path1;
    }
};