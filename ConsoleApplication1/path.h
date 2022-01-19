#pragma once
#include <vector>
#include "vectors.h"
#include "pursuit.h"

class path {
private:
	std::vector<hermite> robotPath;
	bool forward;

public:


	path(bool fwd) { forward = fwd; }

	std::vector<hermite> getRobotPath();

	hermite getStartPoint();
	hermite getEndPoint();

	void initializePath(float maxVel, float maxAccel, float maxVelk);

	void addSegment(vector3D start, vector3D end);
	std::vector<hermite> injectPoints(vector3D startPt, vector3D endPoint);

	float calculatePathCurvature(std::vector<hermite> path, int pointIndex);
	float calculateMaxVelocity(std::vector<hermite> path, int point, float pathMaxVel, float k);

	void setCurvatures();
	void setDistances();
	void setTargetVelocities(float maxVel, float maxAccel, float maxVelk);

	friend class pursuit;

};