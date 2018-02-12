#include "PairIR.h"

PairIR::PairIR() {

}

PairIR::PairIR(int pinL, int pinR) {
	leftIR = pinL;
	rightIR = pinR;
}

void PairIR::read() {
	leftReading = analogRead(leftIR);
	rightReading = analogRead(rightIR);
}

int PairIR::getDiff() {
	return rightReading - leftReading;
}

int PairIR::getAverage() {
	return (leftReading+rightReading)/2;
}
