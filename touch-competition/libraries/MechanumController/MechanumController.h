#ifndef MECHANUMCONTROLLER_H
#define MECHANUMCONTROLLER_H
#include "Arduino.h"
#include "ScrapController.h"


class MechanumController {
	private:
		float minEncSpeed = 0;
		float maxEncSpeed = 600;
		int minPower = 5;
		long translate_x = 0;
		long translate_y = 0;
		long rotate = 0;
		long maximum_val = 32768;
		long deadzone = 0;
		ScrapMotorControl* speedFL;
		ScrapMotorControl* speedFR;
		ScrapMotorControl* speedBL;
		ScrapMotorControl* speedBR;
	public:
		void attachSpeedFL(ScrapMotorControl& speed_control) { speedFL = &speed_control; };
		void attachSpeedFR(ScrapMotorControl& speed_control) { speedFR = &speed_control; };
		void attachSpeedBL(ScrapMotorControl& speed_control) { speedBL = &speed_control; };
		void attachSpeedBR(ScrapMotorControl& speed_control) { speedBR = &speed_control; };
		MechanumController();
		MechanumController(ScrapMotorControl& speed_FL, ScrapMotorControl& speed_FR, ScrapMotorControl& speed_BL, ScrapMotorControl& speed_BR);
		void setTranslateX(long val);
		void setTranslateY(long val);
		void setRotate(long val);
		void performMovement();
		void inputControlValues();
		void setMinimumPower(int power);
		void setMinimumSpeed(float speed);
		void setMaximumSpeed(float speed);
		void setMaximumValue(long value) { maximum_val = value; };
		void setDeadzone(long value) { deadzone = value; };
		long getAverageCount();

};


#endif