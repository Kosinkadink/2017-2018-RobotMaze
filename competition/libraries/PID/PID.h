#ifndef PID_H
#define PID_H
#include "Arduino.h"

class PID {
	private:
		// PID constants
		float pConst = 0;
		float iConst = 0;
		float dConst = 0;
		// timestamp for last run
		unsigned long lastTimestamp = micros();
		float lastError = 0;
		float integral = 0;
		float lastIntegralValue = 0;
		// others
		float goal = 0;
		float lastValue = 0;
		bool resetIntegral = true;
	public:
		PID();
		PID(float p=0, float i=0, float d=0);
		void reset();
		void setGoal(long value) { goal = value; };
		void setResetIntegral(bool value) { resetIntegral = value; };
		float getValue() { return lastValue; };
		float calculate(float measured);

};

#endif
