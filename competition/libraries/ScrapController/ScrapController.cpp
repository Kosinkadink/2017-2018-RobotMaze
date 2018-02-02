#include "ScrapController.h"

//ScrapController::

ScrapController::ScrapController() {
	
}

ScrapController::ScrapController(ScrapMotorControl& motorControl) {
	speedControl = &motorControl;
	stop();
}

ScrapController::ScrapController(ScrapMotor& mot, ScrapEncoder& enc) {
	speedControl = new ScrapMotorControl(mot,enc);
	stop();
}

ScrapController::ScrapController(ScrapMotor& mot, ScrapEncoder& enc, ScrapSwitch& swi) {
	attachSwitch(swi);
	speedControl = new ScrapMotorControl(mot,enc);
	stop();
}

// move back until switches are activated
bool ScrapController::performReset() {
	speedControl->stop();
	// check if switch is pressed
	if (resetswitch->getIfPressed()) {
		speedControl->reset();
		return true;
	}
	else {
		speedControl->setControl(-100);
		speedControl->performMovement();
		return false;
	}
}

bool ScrapController::set(long g) {
	goal = g;
	return checkIfDone();
}

void ScrapController::stop() {
	speedControl->stop();
	speedControl->performMovement();
}

bool ScrapController::performMovement() {
	if (checkIfDone()) { 
		stop();
		return true;
	}
	//else, gotta do stuff
	else {
		if (speedControl->getCount() < goal) {
			speedControl->setControl(calcSpeed());
		}
		else {
			speedControl->setControl(-calcSpeed());
		}
	}
	speedControl->performMovement();
	return false;
}

// calculate speed to give motor
float ScrapController::calcSpeed() {
	long diff = getDiff();
	return speedControl->mapFloat(diff,1,slowdownThresh,minEncSpeed,maxEncSpeed);
}

long ScrapController::getDiff() {
	return abs(speedControl->getCount() - goal);
}

// increment or decrement speed
void ScrapController::incrementSpeed(float speedEncDiff) {
	speedControl->incrementSpeed(speedEncDiff);
}

void ScrapController::decrementSpeed(float speedEncDiff) {
	speedControl->decrementSpeed(speedEncDiff);
}


// check if encoder count is within tolerance of goal
bool ScrapController::checkIfDone() {
	return (speedControl->getCount() >= goal - encTolerance ) && (speedControl->getCount() <= goal + encTolerance );
}
