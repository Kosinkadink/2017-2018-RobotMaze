#ifndef PAIRIR_H
#define PAIRIR_H
#include "Arduino.h"

class PairIR {
	private:
		int leftIR;
		int rightIR;
		int diffGoal = 0;
		int diffThreshold = 5;
		int distGoal = 400;
		int distThreshold = 10;
		// last readings
		int leftReading = distGoal;
		int rightReading = distGoal;
	public:
		PairIR() {};
		PairIR(int pinL, int pinR);
		// set values
		void setDiffGoal(int goal) { diffGoal = goal; };
		void setDiffThreshold(int thresh) { diffThreshold = thresh; };
		void setDistGoal(int goal) { distGoal = goal; };
		void setDistThreshold(int thresh) { distThreshold = thresh; };
		// get raw data
		void read(); // perform sensor readings
		int getDiff();
		int getAverage();
		// get data after considering thresholds; full means include thresholded distance
		int getDiffCorrection(bool full = true);
		int getAverageCorrection(bool full = true);
};

#endif