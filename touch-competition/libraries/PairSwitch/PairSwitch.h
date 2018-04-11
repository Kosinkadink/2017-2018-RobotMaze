#ifndef PAIRSWITCH_H
#define PAIRSWITCH_H
#include "Arduino.h"

class PairSwitch {
	private:
		int leftPin;
		int rightPin;
		// last reading
		int leftReading = 0;
		int rightReading = 0;
		void initializePins();
	public:
		PairSwitch() {};
		PairSwitch(int pinL, int pinR);
		// get raw data
		void read();
		
		int getLeftReading() { return leftReading; };
		int getRightReading() { return rightReading; };

		int getDiff();
		
		int getDist();
		int getDistMin() { return min(leftReading,rightReading); };
		int getDistMax() { return max(leftReading,rightReading); };
};

#endif
