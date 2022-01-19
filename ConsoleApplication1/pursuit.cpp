#include "pursuit.h"
#include "path.h"
#define BASE_WIDTH 12
#define BAD -1000000000000

int lastClosestPoint = 0;
float lookaheadDistance = 6;
bool isForward = true;
bool onLastSegment = false;

std::vector<hermite> pursuitPath;

vector3D pos;

float pursuit::calcIntersectionTVal(vector2D startPoint, vector2D endPoint, vector2D currPos, float lookaheadDistance) {

    vector2D d = endPoint.sub(startPoint);
    vector2D f = startPoint.sub(currPos);
    float a = d.dot(d);
    float b = 2 * f.dot(d);
    float c = f.dot(f) - pow(lookaheadDistance, 2);
    float discriminant = (pow(b, 2)) - (4 * a * c);
    if (discriminant < 0) {
        return BAD;
    } else {
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);
        if (t1 >= 0 && t1 <= 1) {
            return t1;
        }
        if (t2 >= 0 && t2 <= 1) {
            return t2;
        }
        return BAD;
    }
}

vector2D pursuit::calculateLookAheadPoint(vector2D startPoint, vector2D endPoint, vector2D currPos, float lookaheadDistance, bool onLastSegment) {
    float tIntersect = pursuit::calcIntersectionTVal(startPoint, endPoint, currPos, lookaheadDistance);
   // std::cout << "HEY THIS IS: " << tIntersect;
    if (tIntersect == BAD && onLastSegment) {
        return pursuitPath.at(pursuitPath.size() - 1).point;
    } else if (tIntersect == BAD) {
        vector2D p;
        p = vector2D();
        return p;
    }
    else {
        vector2D intersectPoint = endPoint.sub(startPoint);
        vector2D vectorSegment = vector2D().mult(intersectPoint, tIntersect);
        vector2D point = vectorSegment.add(startPoint);
        return point;
    }
}

//heading must be in radians
float pursuit::calculateCurvatureLookAheadArc(vector2D currPos, float heading, vector2D lookahead, float lookaheadDistance) {
    float a = -tan(heading);
    float b = 1;
    float c = (tan(heading) * currPos.x) - currPos.y;
    float x = fabs(a * lookahead.x + b * lookahead.y + c) / sqrt(pow(a, 2) + pow(b, 2));
    float cross = (sin(heading) * (lookahead.x - currPos.x)) - (cos(heading) * (lookahead.y - currPos.y));
    float side = cross > 0 ? 1 : -1;
    float curvature = (2 * x) / (pow(lookaheadDistance, 2));
    return curvature * side;
}

int pursuit::getClosestPointIndex(vector2D currPos) {
    float shortestDistance = WINT_MAX;
    int closestPoint = 0;
    std::vector<hermite> robotPath = pursuitPath;
    for (int i = lastClosestPoint; i < robotPath.size(); i++) {
        if (robotPath.at(i).point.dist(currPos) < shortestDistance) {
            closestPoint = i;
            shortestDistance = robotPath.at(i).point.dist(currPos);
        }
        //lcd::print(6, "shortest distance: %f", shortestDistance);
    }
    lastClosestPoint = closestPoint;
    return closestPoint;
}

bool pursuit::isDone() {
    return pursuit::getClosestPointIndex(pos.to2D()) == pursuitPath.size() - 1;
}

float pursuit::changeLkhdDist(float x, float y, float lookaheadDistance) {
    vector2D point1 = vector2D(x, y);
    float error = pursuitPath.at(pursuitPath.size() - 1).point.dist(point1);
    float lDist = lookaheadDistance;
    if (lookaheadDistance < error) {
        lDist = error;
    }
    return lDist;
}

//After you calculate the left and right wheen speed in in/s, I need to convert to voltage
/**
     * Calculates the left target velocity given target overall velocity
     *
     * @param targetRobotVelocity target overall robot velocity
     * @param curvature           curvature of path at current point
     * @return left target velocity
     */
float pursuit::calculateLeftTargetVelocity(float targetRobotVelocity, float curvature) {
    return targetRobotVelocity * ((2 + (BASE_WIDTH * curvature))) / 2;
}

/**
   * Calculates the right target velocity given target overall velocity
   * @param targetRobotVelocity target overall robot velocity
   * @param curvature curvature of path at current point
   * @return right target velocity
   */
float pursuit::calculateRightTargetVelocity(float targetRobotVelocity, float curvature) {
    return targetRobotVelocity * ((2 - (BASE_WIDTH * curvature))) / 2;
}