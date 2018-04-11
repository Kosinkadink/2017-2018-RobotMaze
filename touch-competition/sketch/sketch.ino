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


// create Switches
PairSwitch backPair = PairSwitch(SWITCH_PIN_BACK_L, SWITCH_PIN_BACK_R);
PairSwitch leftPair = PairSwitch(SWITCH_PIN_LEFT_L, SWITCH_PIN_LEFT_R);
PairSwitch frontPair = PairSwitch(SWITCH_PIN_FRONT_L, SWITCH_PIN_FRONT_R);
PairSwitch rightPair = PairSwitch(SWITCH_PIN_RIGHT_L, SWITCH_PIN_RIGHT_R);
// done creating Switches

float generalSpeed = 50;

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

void setup() {
	mechControl.setMinimumPower(5);
	mechControl.setMinimumSpeed(2);
	mechControl.setMaximumSpeed(600);
	Serial.begin(9600);
}
void loop() {
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
