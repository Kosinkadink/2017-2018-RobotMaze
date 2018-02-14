#ifndef PID_H
#define PID_H
#include "Arduino.h"

class PID {
	private:
		// PID constants
		float pConstant;
		float iConstant;
		float dConstant;
		// timestamp for last run
		unsigned long lastTimestamp;
		float previousError;
		float integral;
		// others
		long goal;
	public:
		PID();
		PID(float p=0, float i=0, float d=0);
		void setGoal(long value) { goal = value; };
		long getValue(long measured);

};

