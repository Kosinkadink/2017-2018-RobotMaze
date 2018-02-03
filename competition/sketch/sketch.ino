#include "ScrapController.h"
#include "MechanumController.h"


#define DIST_PIN_L A14
#define DIST_PIN_R A15


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


// IR stuff
float newValueWeight = 0.40;
int movingMeanL = 0;
int movingMeanR = 0;

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


unsigned long startTime = micros();
unsigned long startTimeIR = micros();


void setup() {
	initEncoders();
	mechControl.setMaximumValue(max_stick_value);
	mechControl.setDeadzone(deadzone);
	pinMode(DIST_PIN_L,INPUT);
	pinMode(DIST_PIN_R,INPUT);
	Serial.begin(9600);
	mechControl.setTranslateY(5);
}

void getNewMovingMean(int newValue, int& mean) {
	mean = newValue*newValueWeight + mean*(1-newValueWeight);
}

void loop() {
	unsigned long currentTime = micros();
	if (currentTime - startTime > 2000) {
		// set rotation
		if (movingMeanR-movingMeanL < diffGoal - diffTolerance) {
			mechControl.setRotate(-3);
		}
		else if (movingMeanR-movingMeanL > diffGoal + diffTolerance) {
			mechControl.setRotate(3);
		}
		else {
			mechControl.setRotate(0);
		}
		// set translation
		if ((movingMeanR+movingMeanL)/2 < distGoal - distTolerance) {
			mechControl.setTranslateX(5);
		}
		else if ((movingMeanR+movingMeanL)/2 > distGoal + distTolerance) {
			mechControl.setTranslateX(-5);	
		}
		else {
			mechControl.setTranslateX(0);
		}

		mechControl.performMovement();
		startTime = currentTime;
	}
	if (currentTime - startTimeIR > 2000) {
		int readValueL = analogRead(DIST_PIN_L);
		int readValueR = analogRead(DIST_PIN_R);
		getNewMovingMean(readValueL,movingMeanL);
		getNewMovingMean(readValueR,movingMeanR);
		//Serial.print(movingMean1);
		//Serial.print("\t");
		//Serial.println(movingMean2);
		Serial.print(movingMeanR-movingMeanL);
		Serial.print("\t");
		Serial.println((movingMeanR+movingMeanL)/2);

		startTimeIR = currentTime;
		//delay(50);
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
