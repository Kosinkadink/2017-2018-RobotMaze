#define DIST_PIN1 A0
#define DIST_PIN2 A1


float newValueWeight = 0.10;
int movingMean1 = 0;
int movingMean2 = 0;


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
