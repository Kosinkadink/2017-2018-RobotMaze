#include "PID.h"

PID::PID() {

}


PID::PID(float p, float i, float d) {
	pConst = p;
	iConst = i;
	dConst = d;
}


void PID::reset() {
	lastTimestamp = micros();
	integral = 0;
}


float PID::calculate(float measured) {
	float error, derivative, integralValue;
	// get time delta
	unsigned long timestamp = micros();
	unsigned long timeDelta = timestamp - lastTimestamp;
	// if no discernable time has passed, return last value
	if (!timeDelta) {
		return lastValue;
	}
	else {
		// calculate P, I, D
		error = measured-goal;
		integralValue = (error*(timeDelta/1000000));
		if (!resetIntegral) {
			integral += integralValue;
		}
		else {
			if (lastError != 0 && (error == 0 || -1*abs(error)/error == abs(lastError)/lastError)) {
				integral = integralValue;
			}
			else {
				integral += integralValue;
			}
		}
		derivative = (error-lastError)/(timeDelta/1000000);
		// update lastError
		lastError = error;
		// calculate new value and set it
		lastValue = (pConst*error) + (iConst*integral) + (dConst*derivative);
		return lastValue;
	}
}
