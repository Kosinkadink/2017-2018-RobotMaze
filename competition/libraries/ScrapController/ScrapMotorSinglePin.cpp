#include "ScrapController.h"


ScrapMotorSinglePin::ScrapMotorSinglePin(int PinD, int PinPWM, int dirMultip) {
	PIN_D = PinD;
	PIN_PWM = PinPWM;
	setDirectionMultiplier(dirMultip);
	initMotor();
}

// initMotor: initialize pins and reset direction/power
void ScrapMotorSinglePin::initMotor() {
	pinMode(PIN_D,OUTPUT);
	pinMode(PIN_PWM,OUTPUT);
	setDirection(0);
	setPower(0);
}

// setMotor: set direction + power based on value given
void ScrapMotorSinglePin::setMotor(int pwm) {
	setDirection(pwm);
	setPower(abs(pwm));
}

// setDirectionMultiplier: set multiplier to flip default direction
void ScrapMotorSinglePin::setDirectionMultiplier(int multi) {
	if (multi > 0) {
		powerMultiplier = 1; 
	}
	else if (multi < 0) {
		powerMultiplier = -1;
	}
	// do nothing if zero
}

// setDirection: set direction based on sign of number
void ScrapMotorSinglePin::setDirection(int pwm) {
	pwm *= powerMultiplier;
	if (pwm > 0) {
		digitalWrite(PIN_D,HIGH);
		currDir = 1;
	}
	else if (pwm < 0) {
		digitalWrite(PIN_D,LOW);
		currDir = -1;
	}
	else {
		analogWrite(PIN_PWM,pwm);
		digitalWrite(PIN_D,HIGH);
		currDir = 0;
	}
}

// getDirection: return direction: 1, 0, or -1
int ScrapMotorSinglePin::getDirection() {
	return currDir;
}

// setPower: set power (PWM) - make sure is in valid range
void ScrapMotorSinglePin::setPower(int pwm) {
	pwm = max(0,min(255,pwm));
	analogWrite(PIN_PWM,pwm);
	currPower = pwm;
}

// getPower: return current power (PWM)
int ScrapMotorSinglePin::getPower() {
	return currPower;
}

// stop: stop the motor
void ScrapMotorSinglePin::stop() {
	setMotor(0);
}
