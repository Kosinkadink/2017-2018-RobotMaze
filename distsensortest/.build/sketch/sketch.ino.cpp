#include <Arduino.h>
#line 1 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"
#line 1 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"
#define DIST_PIN1 A0
#define DIST_PIN2 A1


float newValueWeight = 0.10;
int movingMean1 = 0;
int movingMean2 = 0;


#line 10 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"
void setup();
#line 16 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"
void getNewMovingMean(int newValue, int& mean);
#line 20 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"
void loop();
#line 10 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"
void setup() {
	pinMode(DIST_PIN1,INPUT);
	pinMode(DIST_PIN2,INPUT);
	Serial.begin(9600);
}

void getNewMovingMean(int newValue, int& mean) {
	mean = newValue*newValueWeight + mean*(1-newValueWeight);
}

void loop() {
	int readValue1 = analogRead(DIST_PIN1);
	int readValue2 = analogRead(DIST_PIN2);
	getNewMovingMean(readValue1,movingMean1);
	getNewMovingMean(readValue2,movingMean2);
	//Serial.print(movingMean1);
	//Serial.print("\t");
	//Serial.println(movingMean2);
	Serial.println(movingMean1-movingMean2);
	delay(10);
}

