#include "PairSwitch.h"

PairSwitch::PairSwitch(int pinL, int pinR) {
	leftPin = pinL;
	rightPin = pinR;
	initializePins();
}

void PairSwitch::initializePins() {
	pinMode(leftPin,INPUT);
	pinMode(rightPin,INPUT);
}

void PairSwitch::read() {
	if (digitalRead(leftPin) == HIGH) {
		leftReading = 1;
	}
	else {
		leftReading = 0;
	}
	if (digitalRead(rightPin) == HIGH) {
		rightReading = 1;
	}
	else {
		rightReading = 0;
	}
}

int PairSwitch::getDist() {
	return (leftReading + rightReading)/2;
}

int PairSwitch::getDiff() {
	return rightReading - leftReading;
}

