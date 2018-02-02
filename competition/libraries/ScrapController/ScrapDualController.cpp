#include "ScrapController.h"

//ScrapDualController::

ScrapDualController::ScrapDualController() {

}

ScrapDualController::ScrapDualController(ScrapMotorControl& motorControl1, ScrapMotorControl& motorControl2) {
	speedControl1 = &motorControl1;
	speedControl2 = &motorControl2;
	stop();
}

ScrapDualController::ScrapDualController(ScrapMotor& mot1, ScrapMotor& mot2, ScrapEncoder& enc1, ScrapEncoder& enc2) {
	speedControl1 = new ScrapMotorControl(mot1,enc1);
	speedControl2 = new ScrapMotorControl(mot2,enc2);
	stop();
}

ScrapDualController::ScrapDualController(ScrapMotor& mot1, ScrapMotor& mot2, ScrapEncoder& enc1, ScrapEncoder& enc2, ScrapSwitch& swi1, ScrapSwitch& swi2) {
	speedControl1 = new ScrapMotorControl(mot1,enc1);
	speedControl2 = new ScrapMotorControl(mot2,enc2);
	attachSwitch1(swi1);
	attachSwitch2(swi2);
	stop();
}

// move back until switches are activated
bool ScrapDualController::performReset() {
	speedControl1->stop();
	speedControl2->stop();
	// check if both switches are pressed
	if (switch1->getIfPressed() && switch2->getIfPressed()) {
		speedControl1->reset();
		speedControl2->reset();
		return true;
	}
	else {
		if (switch1->getIfPressed()) {
			speedControl1->setSpeed(0);
		}
		else {
			speedControl1->setControl(-100);
		}
		if (switch2->getIfPressed()) {
			speedControl2->setSpeed(0);
		}
		else {
			speedControl2->setControl(-100);
		}
		speedControl1->performMovement();
		speedControl2->performMovement();
		return false;
	}
}

bool ScrapDualController::set(long g1, long g2) {
	goal1 = g1;
	goal2 = g2;
	return checkIfDone();
}

bool ScrapDualController::set(long goal) {
	return set(goal,goal);
}

void ScrapDualController::shiftCount() {
	speedControl1->setCount(speedControl1->getCount() - goal1);
	speedControl2->setCount(speedControl2->getCount() - goal2);
	set(0,0);
}

bool ScrapDualController::performMovement() {
	//check if already done moving
	if (checkIfDone()) { 
		stop();
		return true;
	}
	//else, gotta do stuff
	if (checkIfDone1()) {
		speedControl1->stop();
	}
	else {
		if (speedControl1->getCount() < goal1) {
			speedControl1->setControl(calcSpeed1());
		}
		else {
			speedControl1->setControl(-calcSpeed1());
		}
	}
	if (checkIfDone2()) {
		speedControl2->stop();
	}
	else {
		if (speedControl2->getCount() < goal2) {
			speedControl2->setControl(calcSpeed2());
		}
		else {
			speedControl2->setControl(-calcSpeed2());
		}
	}
	// based on separation, balance speed of each motor
	balanceSpeed();
	// now execute movement
	speedControl1->performMovement();
	speedControl2->performMovement();
	
	return false;

}

bool ScrapDualController::checkIfNoSpeed() {
	return (speedControl1->getSpeed() == 0 && speedControl2->getSpeed() == 0);
}

// calculate speed to give motor
float ScrapDualController::calcSpeed1() {
	long diff = getDiff1();
	return speedControl1->mapFloat(diff,1,slowdownThresh1,minEncSpeed,maxEncSpeed);
}

// calculate speed to give motor
float ScrapDualController::calcSpeed2() {
	long diff = getDiff2();
	return speedControl2->mapFloat(diff,1,slowdownThresh2,minEncSpeed,maxEncSpeed);
}

long ScrapDualController::getDiff1() {
	return abs(speedControl1->getCount() - goal1);
}

long ScrapDualController::getDiff2() {
	return abs(speedControl2->getCount() - goal2);
}

void ScrapDualController::stop() {
	speedControl1->stop();
	speedControl2->stop();
}

// increment or decrement target speed
void ScrapDualController::incrementSpeed(float speedDiff) {
	speedControl1->incrementSpeed(speedDiff);
	speedControl2->incrementSpeed(speedDiff);
}

void ScrapDualController::decrementSpeed(float speedDiff) {
	speedControl1->decrementSpeed(speedDiff);
	speedControl2->decrementSpeed(speedDiff);
}

void ScrapDualController::balanceSpeed() {
	float common_speed;
	// if 1 too far ahead, balance power towards 2
	if (abs((speedControl1->getCount() - speedControl2->getCount())*speedControl1->getDirection()) >= diffTolerance) {
		moveSpeedToward2(speedBalance);
	}
	// if 2 too far ahead, balance power towards 1
	else if (abs((speedControl1->getCount() - speedControl2->getCount())*speedControl1->getDirection()) <= -diffTolerance) {
		moveSpeedToward1(speedBalance);
	}
	// otherwise, make speeds match if possible
	else {
		if (speedControl1->getSpeedGoal() != 0 && speedControl1->getSpeedGoal() != 0) {
			common_speed = (speedControl1->getSpeedGoal() + speedControl2->getSpeedGoal())/2.0;
			speedControl1->setSpeed(common_speed);
			speedControl2->setSpeed(common_speed);
		}
	}
}

// balance speed
void ScrapDualController::moveSpeedToward1(float speedDiff) {
	speedControl1->incrementSpeed(speedDiff);
	speedControl2->decrementSpeed(speedDiff);
}

void ScrapDualController::moveSpeedToward2(float speedDiff) {
	speedControl1->decrementSpeed(speedDiff);
	speedControl2->incrementSpeed(speedDiff);
}

// check if encoder count is within tolerance of goal
bool ScrapDualController::checkIfDone() {
	return (checkIfDone1() && checkIfDone2());
}

bool ScrapDualController::checkIfDone1() {
	return (speedControl1->getCount() >= goal1 - encTolerance ) && (speedControl1->getCount() <= goal1 + encTolerance );
}

bool ScrapDualController::checkIfDone2() {
	return (speedControl2->getCount() >= goal2 - encTolerance ) && (speedControl2->getCount() <= goal2 + encTolerance );
}
