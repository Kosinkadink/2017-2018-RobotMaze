#include "MazeRobot.h"

MazeRobot::MazeRobot(MechanumController& controller) {
	attachController(controller);
}



void MazeRobot::performMovement() {
	// if no corrections necessary, do goals
	if (!correctionX && !correctionY && !correctionRotate) {
		currentX = goalX;
		currentY = goalY;
		currentRotate = goalRotate;
	}
	// otherwise, throttle speeds to allow for correction
	else {
		// got total absolute sum of corrections
		long correctAbsSum = abs(correctionX) + abs(correctionY) + abs(correctionRotate);
		// calculate X
		if (goalX > 0) {
			currentX = max(goalX-correctAbsSum,0) + correctionX;
		}
		else if (goalX < 0) {
			currentX = min(goalX+correctAbsSum,0) + correctionX;
		}
		else {
			currentX = correctionX;
		}
		// calculate Y
		if (goalY > 0) {
			currentY = max(goalY-correctAbsSum,0) + correctionY;
		}
		else if (goalY < 0) {
			currentY = min(goalY+correctAbsSum,0) + correctionY;
		}
		else {
			currentY = correctionY;
		}
		// calculate Rotate
		if (goalRotate > 0) {
			currentRotate = max(goalRotate-correctAbsSum,0) + correctionRotate;
		}
		else if (goalRotate < 0) {
			currentRotate = min(goalRotate+correctAbsSum,0) + correctionRotate;
		}
		else {
			currentRotate = correctionRotate;
		}
	}

	// set mech movement
	mech->setTranslateX(currentX);
	mech->setTranslateY(currentY);
	mech->setRotate(currentRotate);
	// finally, perform mech's movement
	mech->performMovement();
	/*Serial.print(currentX);
	Serial.print(" ");
	Serial.print(currentY);
	Serial.print(" ");
	Serial.println(currentRotate);*/
}


void MazeRobot::attachController(MechanumController& controller) {
	mech = &controller;
}

