#pragma once

/*
* @struct: PID Calculation Data
*   @ float kP: proportional gain
*   @ float kI: integral gain
*   @ float kD: derivative gain
*   @ float epsilonInner: inner bounds of integration
*   @ float epsilonOuter: outer bounds of integration
*   @ float sigma: integral of error
*   @ float lastValue: previously measured sensor value
*   @ unsigned long lastTime: previously measured system time
*/
extern struct PID {
	float kP;
	float kI;
	float kD;
	float epsilonInner;
	float epsilonOuter;
	float sigma;
	float lastVal;
	float error;
	float gradient;
	unsigned long lastTime;
};

extern void pidInit(PID& pid, float kP, float kI, float kD, float epsilonInner, float epsilonOuter);

extern float pidCalculate(PID& pid, float setPoint, float processVariable);