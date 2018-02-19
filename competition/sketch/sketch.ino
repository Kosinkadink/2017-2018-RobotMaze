#include "ScrapController.h"
#include "MechanumController.h"
#include "MazeRobot.h"
#include "PairIR.h"
#include "PID.h"


#define DIST_PIN_BACK_L A8
#define DIST_PIN_BACK_R A9
#define DIST_PIN_LEFT_L A10
#define DIST_PIN_LEFT_R A11
#define DIST_PIN_FRONT_L A12
#define DIST_PIN_FRONT_R A13
#define DIST_PIN_RIGHT_L A14
#define DIST_PIN_RIGHT_R A15


#define CHECK_ZONE 5000
const int deadzone = 0;
const long max_stick_value = 100;

#define FRONT_LEFT_PIN_INTERRUPT 2
#define FRONT_LEFT_PIN_CHECKER 14
#define FRONT_RIGHT_PIN_INTERRUPT 3
#define FRONT_RIGHT_PIN_CHECKER 15
#define BACK_LEFT_PIN_INTERRUPT 18
#define BACK_LEFT_PIN_CHECKER 16
#define BACK_RIGHT_PIN_INTERRUPT 19
#define BACK_RIGHT_PIN_CHECKER 17

#define FRONT_LEFT_MOTOR_PWM 6
#define FRONT_LEFT_MOTOR_PIN1 32
#define FRONT_LEFT_MOTOR_PIN2 33
#define FRONT_RIGHT_MOTOR_PWM 12
#define FRONT_RIGHT_MOTOR_PIN1 34
#define FRONT_RIGHT_MOTOR_PIN2 35
#define BACK_LEFT_MOTOR_PWM 5
#define BACK_LEFT_MOTOR_PIN1 22
#define BACK_LEFT_MOTOR_PIN2 23
#define BACK_RIGHT_MOTOR_PWM 4
#define BACK_RIGHT_MOTOR_PIN1 24
#define BACK_RIGHT_MOTOR_PIN2 25


int diffGoal = 0;
int diffTolerance = 7;

int distGoal = 500;
int distTolerance = 30;


// create robot parts
ScrapEncoder encoderFL = ScrapEncoder(FRONT_LEFT_PIN_INTERRUPT, FRONT_LEFT_PIN_CHECKER);
ScrapEncoder encoderFR = ScrapEncoder(FRONT_RIGHT_PIN_INTERRUPT, FRONT_RIGHT_PIN_CHECKER);
ScrapEncoder encoderBL = ScrapEncoder(BACK_LEFT_PIN_INTERRUPT, BACK_LEFT_PIN_CHECKER);
ScrapEncoder encoderBR = ScrapEncoder(BACK_RIGHT_PIN_INTERRUPT, BACK_RIGHT_PIN_CHECKER);

ScrapMotor motorFL = ScrapMotor(FRONT_LEFT_MOTOR_PIN1, FRONT_LEFT_MOTOR_PIN2, FRONT_LEFT_MOTOR_PWM);
ScrapMotor motorFR = ScrapMotor(FRONT_RIGHT_MOTOR_PIN1, FRONT_RIGHT_MOTOR_PIN2, FRONT_RIGHT_MOTOR_PWM);
ScrapMotor motorBL = ScrapMotor(BACK_LEFT_MOTOR_PIN1, BACK_LEFT_MOTOR_PIN2, BACK_LEFT_MOTOR_PWM, -1);
ScrapMotor motorBR = ScrapMotor(BACK_RIGHT_MOTOR_PIN1, BACK_RIGHT_MOTOR_PIN2, BACK_RIGHT_MOTOR_PWM, -1);

ScrapMotorControl speedFL = ScrapMotorControl(motorFL, encoderFL);
ScrapMotorControl speedFR = ScrapMotorControl(motorFR, encoderFR);
ScrapMotorControl speedBL = ScrapMotorControl(motorBL, encoderBL);
ScrapMotorControl speedBR = ScrapMotorControl(motorBR, encoderBR);

MechanumController mechControl = MechanumController(speedFL,speedFR,speedBL,speedBR);

MazeRobot mazeRobot = MazeRobot(mechControl);

// create IR
PairIR backPair = PairIR(DIST_PIN_BACK_L, DIST_PIN_BACK_R);
PairIR leftPair = PairIR(DIST_PIN_LEFT_L, DIST_PIN_LEFT_R);
PairIR frontPair = PairIR(DIST_PIN_FRONT_L, DIST_PIN_FRONT_R);
PairIR rightPair = PairIR(DIST_PIN_RIGHT_L, DIST_PIN_RIGHT_R);

float generalSpeed = 5;

PID rightDistPID = PID(0.05,0.00001,0.00001);
PID rightDiffPID = PID(0.05,0.00001,0);

unsigned long startTime = micros();
unsigned long startTimeIR = micros();
unsigned long serialTime = micros();


void setup() {
	initEncoders();
	// set IR stuff
	rightPair.setDiffGoal(diffGoal);
	rightPair.setDiffTolerance(diffTolerance);
	rightPair.setDistGoal(distGoal);
	rightPair.setDistTolerance(distTolerance);
	// set PID stuff
	rightDistPID.setGoal(0);
	rightDiffPID.setGoal(0);

	// set mech stuff
	mechControl.setMaximumValue(max_stick_value);
	mechControl.setDeadzone(deadzone);
	Serial.begin(9600);
	//mechControl.setTranslateY(3);
	//mazeRobot.setTranslateY(0);
	//delay(2000);
	rightDistPID.reset();
	rightDiffPID.reset();
	delay(2000);
	mazeRobot.setTranslateY(generalSpeed);
}

void loop() {
	unsigned long currentTime = micros();
	if (currentTime - startTimeIR > 5000) {
		//int readValueL = analogRead(DIST_PIN_L);
		//int readValueR = analogRead(DIST_PIN_R);
		//getNewMovingMean(readValueL,movingMeanL);
		//getNewMovingMean(readValueR,movingMeanR);
		//Serial.print(movingMean1);
		//Serial.print("\t");
		//Serial.println(movingMean2);
		rightPair.read();
		rightDistPID.calculate(rightPair.getDistCorrection());
		rightDiffPID.calculate(rightPair.getDiffCorrection());
		//Serial.print(rightPair.getDiff());
		//Serial.print("\t");
		//Serial.println(rightPair.getDist());
		startTimeIR = currentTime;

		//delay(50);
	}
	if (currentTime - startTime > 2000) {
		// set rotation
		/*if (rightPair.getDiffCorrection() < 0) {
			//mechControl.setRotate(-3);
			mazeRobot.correctRotate(-3);
		}
		else if (rightPair.getDiffCorrection() > 0) {
			//mechControl.setRotate(3);
			mazeRobot.correctRotate(3);
		}
		else {
			//mechControl.setRotate(0);
			mazeRobot.correctRotate(0);
		}
		// set translation
		if (rightPair.getDistCorrection() > 0) {
			//mechControl.setTranslateX(5);
			mazeRobot.correctTranslateX(5);
		}
		else if (rightPair.getDistCorrection() < 0) {
			//mechControl.setTranslateX(-5);
			mazeRobot.correctTranslateX(-5);
		}
		else {
			//mechControl.setTranslateX(0);
			mazeRobot.correctTranslateX(0);
		}*/
		mazeRobot.correctRotate(rightDiffPID.getValue());
		mazeRobot.correctTranslateX(rightDistPID.getValue());

		//mechControl.performMovement();
		mazeRobot.performMovement();
		startTime = currentTime;
	}
	if (currentTime - serialTime > 100000) {
		Serial.print(rightDistPID.getValue());
		Serial.print("\t");
		Serial.println(rightDiffPID.getValue());
		serialTime = currentTime;
	}

}


void initEncoders() {
	attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_PIN_INTERRUPT),checkEncoderFL,CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_PIN_INTERRUPT),checkEncoderFR,CHANGE);
	attachInterrupt(digitalPinToInterrupt(BACK_LEFT_PIN_INTERRUPT),checkEncoderBL,CHANGE);
	attachInterrupt(digitalPinToInterrupt(BACK_RIGHT_PIN_INTERRUPT),checkEncoderBR,CHANGE);
}

void checkEncoderFL() {
	encoderFL.checkEncoderFlipped();
}

void checkEncoderFR() {
	encoderFR.checkEncoder();
}

void checkEncoderBL() {
	encoderBL.checkEncoderFlipped();
}

void checkEncoderBR() {
	encoderBR.checkEncoder();
}
