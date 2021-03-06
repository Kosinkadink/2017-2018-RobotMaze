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
		long correctionX;
		long correctionY;
		long correctionRotate;
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
		void setTranslateX(long goal) { goalX = goal; };
		void setTranslateY(long goal) { goalY = goal; };
		void setRotate(long goal) { goalRotate = goal; };
		// set requested corrections to movement
		void correctTranslateX(long corr) { correctionX = corr; };
		void correctTranslateY(long corr) { correctionY = corr; };
		void correctRotate(long corr) { correctionRotate = corr; };
		// get speed goals
		long getGoalX() { return goalX; };
		long getGoalY() { return goalY; };
		long getGoalRotate() { return goalRotate; };
		long getMaxGoal() { return max(goalX,max(goalY,goalRotate)); };
		// get average encoder count
		long getAverageCount() { return mech->getAverageCount(); };
		// reset values;
		void reset() { goalX = goalY = goalRotate = correctionX = correctionY = correctionRotate = 0; };
		// perform cycle of movement
		void performMovement();
		// attach a MechanumController instance
		void attachController(MechanumController& controller);

};

#endif