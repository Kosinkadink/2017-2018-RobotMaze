#ifndef MAZEROBOT_H
#define MAZEROBOT_H
#include "Arduino.h"
#include "MechanumController.h"

class MazeRobot {
	private:
		// requested movement
		long goalX;
		long goalY;
		long goalRotate;
		// requested corrections;
		long correctX;
		long correctY;
		long correctRotate;
		// actual values passed to controller
		long currentX;
		long currentY;
		long currentRotate;
		// controller
		MechanumController* mech;
	public:
		MazeRobot() {};
		MazeRobot(MechanumController& controller);
		// set movement goals
		void setTranslateXGoal(long goal) { goalX = goal; };
		void setTranslateYGoal(long goal) { goalY = goal; };
		void setRotateGoal(long goal) { goalRotate = goal; };
		// set requested corrections to movement
		void correctTranslateX(long corr) { correctX = corr; };
		void correctTranslateY(long corr) { correctY = corr; };
		void correctRotate(long corr) { correctRotate = corr; };
		// perform cycle of movement
		void performMovement();
		// attach a MechanumController instance
		void attachController(MechanumController& controller);

}

#endif