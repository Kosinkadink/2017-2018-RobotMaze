# 1 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"
# 1 "C:\\Users\\Jedrek\\Documents\\ArduinoProjects\\2017-2018-RobotMaze\\distsensortest\\sketch\\sketch.ino"




float newValueWeight = 0.10;
int movingMean1 = 0;
int movingMean2 = 0;


void setup() {
 pinMode(A0,0x0);
 pinMode(A1,0x0);
 Serial.begin(9600);
}

void getNewMovingMean(int newValue, int& mean) {
 mean = newValue*newValueWeight + mean*(1-newValueWeight);
}

void loop() {
 int readValue1 = analogRead(A0);
 int readValue2 = analogRead(A1);
 getNewMovingMean(readValue1,movingMean1);
 getNewMovingMean(readValue2,movingMean2);
 //Serial.print(movingMean1);
 //Serial.print("\t");
 //Serial.println(movingMean2);
 Serial.println(movingMean1-movingMean2);
 delay(10);
}
