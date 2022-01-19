#pragma once
#include <vector>
#include "vectors.h"
#include <string>
#include <functional>
#include <iostream>
#include <optional>

class pursuit {


public:
	//this is for path pursuit action
	float calcIntersectionTVal(vector2D startPoint, vector2D endPoint, vector2D currPos, float lookaheadDistance);

	vector2D calculateLookAheadPoint(vector2D startPoint, vector2D endPoint, vector2D currPos, float lookaheadDistance, bool onLastSegment);

	//heading must be in radians
	float calculateCurvatureLookAheadArc(vector2D currPos, float heading, vector2D lookahead, float lookaheadDistance);

	int getClosestPointIndex(vector2D currPos);

	float changeLkhdDist(float x, float y, float lookaheadDistance);

	bool isDone();

	//After you calculate the left and right wheen speed in in/s, I need to convert to voltage
	/**
		 * Calculates the left target velocity given target overall velocity
		 *
		 * @param targetRobotVelocity target overall robot velocity
		 * @param curvature           curvature of path at current point
		 * @return left target velocity
		 */
	float calculateLeftTargetVelocity(float targetRobotVelocity, float curvature);

	/**
	   * Calculates the right target velocity given target overall velocity
	   * @param targetRobotVelocity target overall robot velocity
	   * @param curvature curvature of path at current point
	   * @return right target velocity
	   */
	float calculateRightTargetVelocity(float targetRobotVelocity, float curvature);

};

extern std::vector<hermite> pursuitPath;



