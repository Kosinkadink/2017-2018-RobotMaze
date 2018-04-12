#include "ScrapController.h"
#include "MechanumController.h"
#include "MazeRobot.h"
#include "PairSwitch.h"
#include "PID.h"


#define SWITCH_PIN_BACK_L 38
#define SWITCH_PIN_BACK_R 40
#define SWITCH_PIN_LEFT_L 42
#define SWITCH_PIN_LEFT_R 44
#define SWITCH_PIN_FRONT_L 46
#define SWITCH_PIN_FRONT_R 48
#define SWITCH_PIN_RIGHT_L 50
#define SWITCH_PIN_RIGHT_R 52


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

ScrapMotor motorFL = ScrapMotor(FRONT_LEFT_MOTOR_PIN1, FRONT_LEFT_MOTOR_PIN2, FRONT_LEFT_MOTOR_PWM,-1);
ScrapMotor motorFR = ScrapMotor(FRONT_RIGHT_MOTOR_PIN1, FRONT_RIGHT_MOTOR_PIN2, FRONT_RIGHT_MOTOR_PWM,-1);
ScrapMotor motorBL = ScrapMotor(BACK_LEFT_MOTOR_PIN1, BACK_LEFT_MOTOR_PIN2, BACK_LEFT_MOTOR_PWM);
ScrapMotor motorBR = ScrapMotor(BACK_RIGHT_MOTOR_PIN1, BACK_RIGHT_MOTOR_PIN2, BACK_RIGHT_MOTOR_PWM);

ScrapMotorControl speedFL = ScrapMotorControl(motorFL, encoderFL);
ScrapMotorControl speedFR = ScrapMotorControl(motorFR, encoderFR);
ScrapMotorControl speedBL = ScrapMotorControl(motorBL, encoderBL);
ScrapMotorControl speedBR = ScrapMotorControl(motorBR, encoderBR);

MechanumController mechControl = MechanumController(speedFL,speedFR,speedBL,speedBR);

MazeRobot mazeRobot = MazeRobot(mechControl);
// done making robot parts


// create Switches
PairSwitch backPair = PairSwitch(SWITCH_PIN_BACK_L, SWITCH_PIN_BACK_R);
PairSwitch leftPair = PairSwitch(SWITCH_PIN_LEFT_L, SWITCH_PIN_LEFT_R);
PairSwitch frontPair = PairSwitch(SWITCH_PIN_FRONT_L, SWITCH_PIN_FRONT_R);
PairSwitch rightPair = PairSwitch(SWITCH_PIN_RIGHT_L, SWITCH_PIN_RIGHT_R);
// done creating Switches

float generalSpeed = 10;

// PIDs
//PID distPID = PID(0.12,0.000025,0.00001);
PID distPID = PID(4,0.001,0);
//PID diffPID = PID(0.12,0.000025,0);
PID diffPID = PID(2,0.005,0);
// done with PIDs

// timers
unsigned long startTime = micros();
unsigned long startTimeIR = micros();
unsigned long serialTime = micros();
// done with timers


int distTransition = 1;


void allRead() {
	backPair.read();
	leftPair.read();
	frontPair.read();
	rightPair.read();
}

void setup() {
	initEncoders();
	//mechControl.setMinimumPower(0);
	//mechControl.setMinimumSpeed(2);
	//mechControl.setMaximumSpeed(2000);
	// set mech stuff
	mechControl.setMaximumValue(max_stick_value);
	mechControl.setDeadzone(deadzone);
	distPID.setGoal(1);
	Serial.begin(9600);

	firstSegment();
	thirdSegment();
	mazeRobot.reset();
	mazeRobot.performMovement();
	mechControl.setMinimumPower(255);
	suicideRunRight();//testSegment();
	suicideRunUp();
	suicideRunLeft();
	suicideRunUp();
	suicideRunLeft();
	suicideRunDown();
	suicideRunRight();
	suicideRunDown();
	suicideRunLeft();
	suicideRunDown();
	suicideRunRight();
	suicideRunDown();
	suicideRunLeft();
	suicideRunUp();
	suicideRunLeft();
	suicideRunUp();
	suicideRunRight();
	suicideRunUp();
	suicideRunLeft();
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

void loopTEST() {
	delay(50);
	allRead();
	Serial.print(backPair.getLeftReading());
	Serial.print(" ");
	Serial.print(backPair.getRightReading());
	Serial.print(" ");
	Serial.print(leftPair.getLeftReading());
	Serial.print(" ");
	Serial.print(leftPair.getRightReading());
	Serial.print(" ");
	Serial.print(frontPair.getLeftReading());
	Serial.print(" ");
	Serial.print(frontPair.getRightReading());
	Serial.print(" ");
	Serial.print(rightPair.getLeftReading());
	Serial.print(" ");
	Serial.println(rightPair.getRightReading());
}


void suicideRunLeft() {
	mazeRobot.setTranslateX(-generalSpeed);
	while (leftPair.getDist() != distTransition) {
		allRead();
		mazeRobot.performMovement();
		delay(2);
	}
	mazeRobot.reset();
	mazeRobot.performMovement();
}

void suicideRunRight() {
	mazeRobot.setTranslateX(generalSpeed);
	while (rightPair.getDist() != distTransition) {
		allRead();
		mazeRobot.performMovement();
		delay(2);
	}
	mazeRobot.reset();
	mazeRobot.performMovement();
}

void suicideRunUp() {
	mazeRobot.setTranslateY(generalSpeed);
	while (frontPair.getDist() != distTransition) {
		allRead();
		mazeRobot.performMovement();
		delay(2);
	}
	mazeRobot.reset();
	mazeRobot.performMovement();
}

void suicideRunDown() {
	mazeRobot.setTranslateY(-generalSpeed);
	while (backPair.getDist() != distTransition) {
		allRead();
		mazeRobot.performMovement();
		delay(2);
	}
	mazeRobot.reset();
	mazeRobot.performMovement();
}


void testSegment() {
	//distPID.reset();
	//diffPID.reset();
	mazeRobot.setTranslateY(20);
	mazeRobot.setTranslateX(-10);
	//mazeRobot.correctTranslateY(0);
	//mazeRobot.correctTranslateX(0);
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			//distPID.calculate(backPair.getDist());
			//diffPID.calculate(backPair.getDiff());
			
			startTimeIR = currentTime;
			if (leftPair.getDist() >= distTransition) {
				break;
			}
			//Serial.print(mazeRobot.getGoalX());
			//Serial.print(" ");
			//Serial.println(mazeRobot.getGoalY());
		}
		if (currentTime - startTime > 2000) {
			
			//mazeRobot.correctRotate(diffPID.getValue());
			//mazeRobot.correctTranslateY(distPID.getValue());
			// perform designated movement
			mazeRobot.performMovement();
			/*Serial.print(mazeRobot.getGoalX());
			Serial.print(" ");
			Serial.println(mazeRobot.getGoalY());*/
			startTime = currentTime;
		}	
	}
	mazeRobot.reset();
}


void firstSegment() {
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateX(-generalSpeed);
	unsigned long movementTime = micros();
	bool goTowardsWall = true;
	unsigned long waitTime = 1000000;
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			//distPID.calculate(backPair.getDist());
			//diffPID.calculate(backPair.getDiff());
			
			startTimeIR = currentTime;
			if (leftPair.getDist() >= distTransition) {
				break;
			}
			//Serial.print(mazeRobot.getGoalX());
			//Serial.print(" ");
			//Serial.println(mazeRobot.getGoalY());
		}
		if (currentTime - startTime > 2000) {
			
			//mazeRobot.correctRotate(diffPID.getValue());
			//mazeRobot.correctTranslateY(distPID.getValue());
			// perform designated movement
			if (backPair.getLeftReading() == 1 || backPair.getRightReading() == 1) {
				mazeRobot.setTranslateY(0);
				mazeRobot.setTranslateX(-generalSpeed);
			}
			mazeRobot.performMovement();
			startTime = currentTime;
		}
		if (currentTime - movementTime > waitTime) {
			movementTime = currentTime;
			if (goTowardsWall) {
				mazeRobot.setTranslateY(0);
				mazeRobot.setTranslateX(-generalSpeed);
				waitTime = 50000;
			}
			else {
				mazeRobot.setTranslateX(0);
				mazeRobot.setTranslateY(-generalSpeed);
				waitTime = 1000000;
			}
			goTowardsWall = !goTowardsWall;
		}	
	}
	mazeRobot.reset();
}


void thirdSegment() {
	// go backward first
	mazeRobot.reset();
	mazeRobot.setTranslateY(-generalSpeed);
	while (backPair.getDist() != distTransition) {
		allRead();
		mazeRobot.performMovement();
		delay(2);
	}
	mazeRobot.reset();
	long startEncoderCount = mazeRobot.getAverageCount();
	long currEncoderCount = startEncoderCount;
	long countDifference = 0;
	unsigned long currentTime = micros();
	mazeRobot.setTranslateY(generalSpeed);
	while (true) {
		currentTime = micros();
		if (currentTime - startTime > 2000) {
			mazeRobot.performMovement();
			startTime = currentTime;
		}
		if (currentTime - startTimeIR > 5000) {
			currEncoderCount = mazeRobot.getAverageCount();
			countDifference = currEncoderCount - startEncoderCount;
			if (abs(countDifference) > 480) {
				break;
			}
			startTimeIR = currentTime;
		}
	}
	mazeRobot.reset();
}


void secondSegment() {
	distPID.reset();
	diffPID.reset();
	mazeRobot.setTranslateY(generalSpeed);
	while (true) {
		unsigned long currentTime = micros();
		if (currentTime - startTimeIR > 5000) {
			
			allRead();
			distPID.calculate(backPair.getDist());
			diffPID.calculate(backPair.getDiff());
			
			startTimeIR = currentTime;
			if (frontPair.getDist() >= distTransition) {
				break;
			}
			//Serial.print(mazeRobot.getGoalX());
			//Serial.print(" ");
			//Serial.println(mazeRobot.getGoalY());
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


void initEncoders() {
	attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_PIN_INTERRUPT),checkEncoderFL,CHANGE);
	attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_PIN_INTERRUPT),checkEncoderFR,CHANGE);
	attachInterrupt(digitalPinToInterrupt(BACK_LEFT_PIN_INTERRUPT),checkEncoderBL,CHANGE);
	attachInterrupt(digitalPinToInterrupt(BACK_RIGHT_PIN_INTERRUPT),checkEncoderBR,CHANGE);
}

void checkEncoderFL() {
	encoderFL.checkEncoder();
}

void checkEncoderFR() {
	encoderFR.checkEncoderFlipped();
}

void checkEncoderBL() {
	encoderBL.checkEncoder();
}

void checkEncoderBR() {
	encoderBR.checkEncoderFlipped();
}
