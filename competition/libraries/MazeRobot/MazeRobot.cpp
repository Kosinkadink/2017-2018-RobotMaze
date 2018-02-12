#include "MazeRobot.h"

MazeRobot::MazeRobot(MechanumController& controller) {
	attachController(controller);
}



void MazeRobot::performMovement() {
	// if no corrections necessary, do goals
	if (!correctX && !correctY && !correctRotate) {
		currentX = goalX;
		currentY = goalY;
		currentRotate = goalRotate;
	}
	// otherwise, throttle speeds to allow for correction
	else {
		// got total absolute sum of corrections
		long correctAbsSum = abs(correctX) + abs(correctY) + abs(correctRotate);
		// calculate X
		if (goalX > 0) {
			currentX = max(goalX-correctAbsSum,0) + correctX;
		}
		else if (goalX < 0) {
			currentX = min(goalX+correctAbsSum,0) + correctX;
		}
		else {
			currentX = correctX;
		}
		// calculate Y
		if (goalY > 0) {
			currentY = max(goalY-correctAbsSum,0) + correctY;
		}
		else if (goalY < 0) {
			currentY = min(goalY+correctAbsSum,0) + correctY;
		}
		else {
			currentY = correctY;
		}
		// calculate Rotate
		if (goalRotate > 0) {
			currentRotate = maRotate(goalRotate-correctAbsSum,0) + correctRotate;
		}
		else if (goalRotate < 0) {
			currentRotate = min(goalRotate+correctAbsSum,0) + correctRotate;
		}
		else {
			currentRotate = correctRotate;
		}
	}

	// set mech movement
	mech.setTranslateX(currentX);
	mech.setTranslateY(currentY);
	mech.setRotate(currentRotate);
	// finally, perform mech's movement
	mech.performMovement();
}


void MazeRobot::attachController(MechanumController& controller) {
	mech = &controller;
}

