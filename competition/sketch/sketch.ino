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
// done making robot parts


// create IR
PairIR backPair = PairIR(DIST_PIN_BACK_L, DIST_PIN_BACK_R);
PairIR leftPair = PairIR(DIST_PIN_LEFT_L, DIST_PIN_LEFT_R);
PairIR frontPair = PairIR(DIST_PIN_FRONT_L, DIST_PIN_FRONT_R);
PairIR rightPair = PairIR(DIST_PIN_RIGHT_L, DIST_PIN_RIGHT_R);
PairIR* criticalIR;
// done creating IR


float generalSpeed = 40;

int diffGoal = 0;
int diffTolerance = 10;

int distGoal = 485;
int distTolerance = 20;

int distSlowdown = 230;
int distTransition = 385;

// PIDs
PID distPID = PID(0.12,0.000025,0.00001);
PID diffPID = PID(0.12,0.000025,0);
// done with PIDs

// timers
unsigned long startTime = micros();
unsigned long startTimeIR = micros();
unsigned long serialTime = micros();
// done with timers

void allRead() {
	backPair.read();
	leftPair.read();
	frontPair.read();
	rightPair.read();
}

float getCorrectionMultiplier() {
	return max(mazeRobot.getMaxGoal()/20,1);
}


void setup() {
	initEncoders();
	// set IR stuff
	backPair.setDiffGoal(-10);
	backPair.setDiffTolerance(diffTolerance);
	backPair.setDistGoal(465);
	backPair.setDistTolerance(distTolerance);
	leftPair.setDiffGoal(-7);
	leftPair.setDiffTolerance(diffTolerance);
	leftPair.setDistGoal(465);
	leftPair.setDistTolerance(distTolerance);
	frontPair.setDiffGoal(-3);
	frontPair.setDiffTolerance(diffTolerance);
	frontPair.setDistGoal(465);
	frontPair.setDistTolerance(distTolerance);
	rightPair.setDiffGoal(10);
	rightPair.setDiffTolerance(diffTolerance);
	rightPair.setDistGoal(455);
	rightPair.setDistTolerance(distTolerance);
	// set PID stuff
	distPID.setGoal(0);
	diffPID.setGoal(0);

	// set mech stuff
	mechControl.setMaximumValue(max_stick_value);
	mechControl.setDeadzone(deadzone);
	Serial.begin(9600);
	//mechControl.setTranslateY(3);
	//mazeRobot.setTranslateY(0);
	//delay(2000);
	delay(2000);
	// perform segments
	
	// NAVIGATE THROUGH BEGINNING BLOCK
	seventhSegment();
	eighthSegment();
	// NAVIGATE THROUGH MIDDLE TWO BLOCKS
	/*
	firstSegment();
	secondSegment();
	thirdSegment();
	fourthSegment();
	fifthSegment();
	sixthSegment();
	thirdSegment();
	fourthSegment();
	fifthSegment();
	seventhSegment();
	*/

}

void loop() {
	mazeRobot.reset();
	unsigned long currentTime = micros();
	if (currentTime - startTime > 2000) {

		//mechControl.performMovement();
		mazeRobot.performMovement();
		startTime = currentTime;
	}
}

void loopOLD() {
	unsigned long currentTime = micros();
	if (currentTime - startTimeIR > 5000) {
		
		allRead();
		distPID.calculate(rightPair.getDistCorrection()*getCorrectionMultiplier());
		diffPID.calculate(rightPair.getDiffCorrection()*getCorrectionMultiplier());
		
		startTimeIR = currentTime;
		if (frontPair.getDist() >= 400) {
			mazeRobot.setTranslateY(0);
		}
		else if (frontPair.getDist() < 400 && frontPair.getDist() > 250) {
			mazeRobot.setTranslateY(10);	
		}

		//delay(50);
	}
	if (currentTime - startTime > 2000) {
		
		mazeRobot.correctRotate(diffPID.getValue());
		mazeRobot.correctTranslateX(distPID.getValue());

		//mechControl.performMovement();
		mazeRobot.performMovement();
		startTime = currentTime;
	}
	/*if (currentTime - serialTime > 100000) {
		Serial.print(distPID.getValue());
		Serial.print("\t");
		Serial.println(diffPID.getValue());
		serialTime = currentTime;
	}*/

}


void firstSegment() {
	// FORWARD following RIGHT wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateY(generalSpeed);
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			distPID.calculate(rightPair.getDistCorrection()*getCorrectionMultiplier());
			diffPID.calculate(rightPair.getDiffCorrection()*getCorrectionMultiplier());
			
			startTimeIR = currentTime;
			if (frontPair.getDist() >= distTransition) {
				break;
			}
			else if (frontPair.getDist() < distTransition && frontPair.getDist() > distSlowdown) {
				mazeRobot.setTranslateY(10);	
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateX(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
}


void secondSegment() {
	// LEFTWARD following FRONT wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateX(-generalSpeed);
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			distPID.calculate(frontPair.getDistCorrection()*getCorrectionMultiplier());
			diffPID.calculate(frontPair.getDiffCorrection()*getCorrectionMultiplier());
			
			startTimeIR = currentTime;
			if (leftPair.getDist() >= distTransition) {
				break;
			}
			else if (leftPair.getDist() < distTransition && leftPair.getDist() > distSlowdown) {
				mazeRobot.setTranslateX(-10);	
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateY(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
}


void thirdSegment() {
	// BACKWARD following LEFT wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateY(-generalSpeed);
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			distPID.calculate(-leftPair.getDistCorrection()*getCorrectionMultiplier());
			diffPID.calculate(leftPair.getDiffCorrection()*getCorrectionMultiplier());
			
			startTimeIR = currentTime;
			if (backPair.getDist() >= distTransition) {
				break;
			}
			else if (backPair.getDist() < distTransition && backPair.getDist() > distSlowdown) {
				mazeRobot.setTranslateY(-10);	
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateX(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
}

void fourthSegment() {
	// RIGHTWARD going between BOTTOM then TOP wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateX(generalSpeed);
	bool huggingBottom = true;
	bool huggingFront = false;
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			
			if (abs(backPair.getDiff()) > 100) {
				huggingBottom = false;
			}
			if (!huggingBottom && frontPair.getDist() > 300) {
				huggingFront = true;
			}
			// while next to bottom wall, hug it
			if (huggingBottom) {
				distPID.calculate(-backPair.getDistCorrection()*getCorrectionMultiplier());
				diffPID.calculate(backPair.getDiffCorrection()*getCorrectionMultiplier());
			}
			// otherwise, strage and then hug top wall
			/*else {
				if (huggingFront) {
					distPID.calculate(frontPair.getDistCorrection()*getCorrectionMultiplier());
					diffPID.calculate(frontPair.getDiffCorrection()*getCorrectionMultiplier());
				}
				else {
					diffPID.reset();
				}
			}*/


			startTimeIR = currentTime;
			if (rightPair.getDist() >= distTransition) {
				break;
			}
			else if (rightPair.getDist() < distTransition && rightPair.getDist() > distSlowdown) {
				mazeRobot.setTranslateX(10);
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateY(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
}


void fifthSegment() {
	// BACKWARD following RIGHT wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateY(-generalSpeed/2);
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			distPID.calculate(rightPair.getDistCorrection()*getCorrectionMultiplier());
			diffPID.calculate(rightPair.getDiffCorrection()*getCorrectionMultiplier());
			
			startTimeIR = currentTime;
			if (backPair.getDist() >= distTransition) {
				break;
			}
			else if (backPair.getDist() < distTransition && backPair.getDist() > distSlowdown) {
				mazeRobot.setTranslateY(-10);	
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateX(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
}


void sixthSegment() {
	// LEFTWARD following BOTTOM wall, then TOP wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateX(-generalSpeed);
	bool huggingBottom = true;
	bool huggingFront = false;
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			
			if (abs(backPair.getDiff()) > 100) {
				huggingBottom = false;
			}
			if (!huggingBottom && frontPair.getDist() > 300) {
				huggingFront = true;
			}
			// while next to bottom wall, hug it
			if (huggingBottom) {
				distPID.calculate(-backPair.getDistCorrection()*getCorrectionMultiplier());
				diffPID.calculate(backPair.getDiffCorrection()*getCorrectionMultiplier());
			}
			// otherwise, strage and then hug top wall
			else {
				//distPID.calculate(frontPair.getDistCorrection()*getCorrectionMultiplier());
				distPID.reset();
				diffPID.calculate(frontPair.getDiffCorrection()*getCorrectionMultiplier());
				
			}


			startTimeIR = currentTime;
			if (leftPair.getDist() >= distTransition) {
				break;
			}
			else if (leftPair.getDist() < distTransition && leftPair.getDist() > distSlowdown) {
				mazeRobot.setTranslateX(-10);
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateY(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
}

void seventhSegment() {
	// LEFTWARD following BOTTOM wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateX(-generalSpeed);
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			
			distPID.calculate(-backPair.getDistCorrection()*getCorrectionMultiplier());
			diffPID.calculate(backPair.getDiffCorrection()*getCorrectionMultiplier());

			startTimeIR = currentTime;
			if (leftPair.getDist() >= distTransition) {
				break;
			}
			else if (leftPair.getDist() < distTransition && leftPair.getDist() > distSlowdown) {
				mazeRobot.setTranslateX(-10);
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateY(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
}

void eighthSegment() {
	// FORWARD following LEFT wall
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateY(generalSpeed);
	// encoder counts
	long startEncoderCount = mazeRobot.getAverageCount();
	long currEncoderCount = startEncoderCount;
	long countDifference;
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			distPID.calculate(-leftPair.getDistCorrection()*getCorrectionMultiplier());
			diffPID.calculate(leftPair.getDiffCorrection()*getCorrectionMultiplier());
			// encoder count stuff
			currEncoderCount = mazeRobot.getAverageCount();
			countDifference = currEncoderCount - startEncoderCount;
			// end of encoder count stuff
			startTimeIR = currentTime;
			//if (backPair.getDistMin() <= 130 || countDifference > 2000) {
			if (countDifference > 1850) {
				break;
			}
			//else if (backPair.getDist() > 130 && backPair.getDist() < (130+20)) {
			else if (backPair.getDist() < (130+35)) {
				mazeRobot.setTranslateY(10);	
			}
		}
		if (currentTime - startTime > 2000) {
			
			mazeRobot.correctRotate(diffPID.getValue());
			mazeRobot.correctTranslateX(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			startTime = currentTime;
		}
	}
	mazeRobot.reset();
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
