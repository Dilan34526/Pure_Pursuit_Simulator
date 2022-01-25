#include "pid.h"
#include <cmath>


void pidInit(PID& pid, float kP, float kI, float kD, float epsilonInner, float epsilonOuter) {
	pid.kP = kP;
	pid.kI = kI;
	pid.kD = kD;
	pid.epsilonInner = epsilonInner; //bounds of integration
	pid.epsilonOuter = epsilonOuter;
	pid.sigma = 0;
	pid.error = 0;
	pid.gradient = 0;
	pid.lastVal = 0;
	pid.lastTime = 0;
}


float pidCalculate(PID& pid, float setPoint, float processVariable) {

	//  Calculate elpased time per loop
	float deltaTime = (float)(10 - pid.lastTime) / 1000.0;
	pid.lastTime = 10;

	//  Calculate error
	pid.error = setPoint - processVariable;

	//  Rate of change (slope) in error per loop
	if (deltaTime > 0) {
		pid.gradient = (processVariable - pid.lastVal) / deltaTime;
	}
	pid.lastVal = processVariable;

	//  Sum error within bounds of integration
	if (fabs(pid.error) > pid.epsilonInner && fabs(pid.error) < pid.epsilonOuter) {
		pid.sigma += pid.error * deltaTime;
	}

	//  Reset integral outside of its bounds
	if (fabs(pid.error) > pid.epsilonOuter) {
		pid.sigma = 0;
	}

	float output = 0;
	output = (pid.error * pid.kP) + (pid.sigma * pid.kI) - (pid.gradient * pid.kD);
	return output;
}

