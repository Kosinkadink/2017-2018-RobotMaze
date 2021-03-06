#include "PairIR.h"

PairIR::PairIR(int pinL, int pinR) {
	leftIR = pinL;
	rightIR = pinR;
}

void PairIR::read() {
	leftReading = analogRead(leftIR);
	rightReading = analogRead(rightIR);
	getNewMovingMean(leftReading,leftReadingMean);
	getNewMovingMean(rightReading,rightReadingMean);
}

int PairIR::getDiff() {
	return rightReading - leftReading;
}

int PairIR::getDist() {
	return (leftReading+rightReading)/2;
}

int PairIR::getDistMin() {
	return min(leftReading,rightReading);
}

int PairIR::getDistMax() {
	return max(leftReading,rightReading);
}

void PairIR::getNewMovingMean(int newValue, int& mean) {
	mean = newValue*newValueWeight + mean*(1-newValueWeight);
}

int PairIR::getDiffCorrection() {
	int diff = getDiff();
	if (diff < diffGoal - diffTolerance) {
		return diff;
	}
	else if (diff > diffGoal + diffTolerance) {
		return diff;
	}
	else {
		return 0;
	}
}

int PairIR::getDistCorrection(int valueToUse) {
	int dist;
	

	if (valueToUse < 0) {
		dist = getDistMin();
	}
	else if (valueToUse > 0) {
		dist = getDistMax();
	}
	else {
		dist = getDist();
	}

	/*switch(valueToUse) {
		case -1: dist = getDistMin(); break; // get min
		case 0: dist = getDist(); break; // get average
		case 1: dist = getDistMax(); break; // get max
		default: dist = getDist(); //get average otherwise
	}*/

	// if should check tolerance, stay around the goal
	if (checkTolerance) {
		if (dist < distGoal - distTolerance) {
			return distGoal - dist;
		}
		else if (dist > distGoal + distTolerance) {
			return distGoal - dist;
		}
		else {
			return 0;
		}
	}
	// otherwise stay with min/max bounds
	else {
		if (dist < distMin) {
			return distMin - dist;
		}
		else if (dist > distMax) {
			return distMax - dist;
		}
		else {
			return 0;
		}
	}
}
