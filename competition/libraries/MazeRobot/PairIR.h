#ifndef PAIRIR_H
#define PAIRIR_H
#include "Arduino.h"

class PairIR {
	private:
		int leftIR;
		int rightIR;
		int diffGoal = 0;
		int diffTolerance = 5;
		int distGoal = 400;
		int distTolerance = 10;
		// last readings
		int leftReading = distGoal;
		int rightReading = distGoal;
		int leftReadingMean = distGoal;
		int rightReadingMean = distGoal;
		// moving average variables
		float newValueWeight = 0.70;
		// FUNCTIONS
		void getNewMovingMean(int newValue, int& mean);
	public:
		PairIR() {};
		PairIR(int pinL, int pinR);
		// set values
		void setDiffGoal(int goal) { diffGoal = goal; };
		void setDiffTolerance(int thresh) { diffTolerance = thresh; };
		void setDistGoal(int goal) { distGoal = goal; };
		void setDistTolerance(int thresh) { distTolerance = thresh; };
		void setNewValueWeight(float value) { newValueWeight = value; };
		// get raw data
		void read(); // perform sensor readings
		
		int getLeftReading() { return leftReading; };
		int getLeftReadingMean() { return leftReadingMean; };
		int getRightReading() { return rightReading; };
		int getRightReadingMean() { return rightReadingMean; };

		int getDiff();
		int getDist();
		// get data after considering thresholds; full means include thresholded distance
		int getDiffCorrection();
		int getDistCorrection();
};

#endif