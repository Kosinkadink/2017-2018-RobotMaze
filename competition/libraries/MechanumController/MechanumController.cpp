#include "MechanumController.h"

MechanumController::MechanumController() {

}

MechanumController::MechanumController(ScrapMotorControl& speed_FL, ScrapMotorControl& speed_FR, ScrapMotorControl& speed_BL, ScrapMotorControl& speed_BR) {
	speedFL = &speed_FL;
	speedFR = &speed_FR;
	speedBL = &speed_BL;
	speedBR = &speed_BR;
	setMinimumPower(minPower);
	setMinimumSpeed(minEncSpeed);
	setMaximumSpeed(maxEncSpeed);
}


void MechanumController::performMovement() {
	inputControlValues();
	speedFL->performMovement();
	speedFR->performMovement();
	speedBL->performMovement();
	speedBR->performMovement();
}

void MechanumController::inputControlValues() {
	// initialize raw values to consider
	int maxRawValue;
	int rawValueFL, rawValueFR, rawValueBL, rawValueBR;
	int encValueFL, encValueFR, encValueBL, encValueBR;
	long norm_maximum_val = maximum_val-deadzone;
	long proc_maximum_val = 1024;
	// check if the movements are zero
	if (!translate_x && !translate_y && !rotate) {
		speedFL->stop();
		speedFR->stop();
		speedBL->stop();
		speedBR->stop();
		return;
	}
	int proc_translate_x = (abs(translate_x)/translate_x)*map(abs(translate_x), -norm_maximum_val, norm_maximum_val, -proc_maximum_val, proc_maximum_val);
	int proc_translate_y = (abs(translate_y)/translate_y)*map(abs(translate_y), -norm_maximum_val, norm_maximum_val, -proc_maximum_val, proc_maximum_val);
	int proc_rotate = (abs(rotate)/rotate)*map(abs(rotate), -norm_maximum_val, norm_maximum_val, -proc_maximum_val, proc_maximum_val);
	rawValueFL = rawValueFR = rawValueBL = rawValueBR = 0;
	// translation and rotation consideration
	rawValueFL += proc_translate_y + proc_translate_x + proc_rotate;
	rawValueFR += proc_translate_y - proc_translate_x - proc_rotate;
	rawValueBL += proc_translate_y - proc_translate_x + proc_rotate;
	rawValueBR += proc_translate_y + proc_translate_x - proc_rotate;
	// figure out max raw value; this will be used to scale the speed of the motors if max is greater than a motor can output
	maxRawValue = max(max(abs(rawValueFL),abs(rawValueFR)),max(abs(rawValueBL),abs(rawValueBR)));
	// if max value does not exceed the normalized allowed maximum value, use the normalized max as max raw value
	if (maxRawValue < proc_maximum_val) {
		maxRawValue = proc_maximum_val;
	}
	// calculate enc speeds and assign to speed controllers if the raw values were nonzero
	// if the raw values were negative, account for the mapping error
	// FRONT LEFT
	encValueFL = (abs(rawValueFL)/rawValueFL)*map(abs(rawValueFL),0,maxRawValue,minEncSpeed,maxEncSpeed);
	speedFL->setControl(encValueFL);
	// FRONT RIGHT
	encValueFR = (abs(rawValueFR)/rawValueFR)*map(abs(rawValueFR),0,maxRawValue,minEncSpeed,maxEncSpeed);
	speedFR->setControl(encValueFR);
	// BACK LEFT
	encValueBL = (abs(rawValueBL)/rawValueBL)*map(abs(rawValueBL),0,maxRawValue,minEncSpeed,maxEncSpeed);
	speedBL->setControl(encValueBL);
	// BACK RIGHT
	encValueBR = (abs(rawValueBR)/rawValueBR)*map(abs(rawValueBR),0,maxRawValue,minEncSpeed,maxEncSpeed);
	speedBR->setControl(encValueBR);
	// print values for debug purposes
	/*Serial.print(encValueFL);
	Serial.print(" ");
	Serial.print(encValueFR);
	Serial.print(" ");
	Serial.print(encValueBL);
	Serial.print(" ");
	Serial.println(encValueBR);*/
}

void MechanumController::setTranslateX(long val) {
	if (val < 0) {
		val += deadzone;
	}
	else if (val > 0) {
		val -= deadzone;
	}
	translate_x = val;
}

void MechanumController::setTranslateY(long val) {
	if (val < 0) {
		val += deadzone;
	}
	else if (val > 0) {
		val -= deadzone;
	}
	translate_y = val;
}

void MechanumController::setRotate(long val) {
	if (val < 0) {
		val += deadzone;
	}
	else if (val > 0) {
		val -= deadzone;
	}
	rotate = val;
}

void MechanumController::setMinimumPower(int power) {
	minPower = power;
	speedFL->setMinPower(minPower);
	speedFR->setMinPower(minPower);
	speedBL->setMinPower(minPower);
	speedBR->setMinPower(minPower);
}

void MechanumController::setMinimumSpeed(float speed) {
	minEncSpeed = speed;
	speedFL->setMinSpeed(minEncSpeed);
	speedFR->setMinSpeed(minEncSpeed);
	speedBL->setMinSpeed(minEncSpeed);
	speedBR->setMinSpeed(minEncSpeed);
}

void MechanumController::setMaximumSpeed(float speed) {
	maxEncSpeed = speed;
	speedFL->setMaxSpeed(maxEncSpeed);
	speedFR->setMaxSpeed(maxEncSpeed);
	speedBL->setMaxSpeed(maxEncSpeed);
	speedBR->setMaxSpeed(maxEncSpeed);
}

long MechanumController::getAverageCount() {
	return (speedFL->getCount()+speedFR->getCount()+speedBL->getCount()+speedBR->getCount())/4;
}

