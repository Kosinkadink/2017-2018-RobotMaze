#include "PairIR.h"

#define DIST_PIN_BACK_L A8
#define DIST_PIN_BACK_R A9
#define DIST_PIN_LEFT_L A10
#define DIST_PIN_LEFT_R A11
#define DIST_PIN_FRONT_L A12
#define DIST_PIN_FRONT_R A13
#define DIST_PIN_RIGHT_L A14
#define DIST_PIN_RIGHT_R A15


// create IR
PairIR backPair = PairIR(DIST_PIN_BACK_L, DIST_PIN_BACK_R);
PairIR leftPair = PairIR(DIST_PIN_LEFT_L, DIST_PIN_LEFT_R);
PairIR frontPair = PairIR(DIST_PIN_FRONT_L, DIST_PIN_FRONT_R);
PairIR rightPair = PairIR(DIST_PIN_RIGHT_L, DIST_PIN_RIGHT_R);


void setup() {
	Serial.begin(9600);
}

void loop() {
	delay(50);
	// read sensor pairs
	backPair.read();
	leftPair.read();
	frontPair.read();
	rightPair.read();
	// for back pair:
	Serial.print(backPair.getDiff());
	Serial.print("\t");
	Serial.println(backPair.getDist());
	/*
	// for left pair:
	Serial.print(leftPair.getDiff());
	Serial.print("\t");
	Serial.println(leftPair.getDist());
	// for front pair:
	Serial.print(frontPair.getDiff());
	Serial.print("\t");
	Serial.println(frontPair.getDist());
	// for right pair:
	Serial.print(rightPair.getDiff());
	Serial.print("\t");
	Serial.println(rightPair.getDist());
	*/
}
