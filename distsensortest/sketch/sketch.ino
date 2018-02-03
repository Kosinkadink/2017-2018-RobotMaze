#define DIST_PIN_L A14
#define DIST_PIN_R A15


float newValueWeight = 0.10;
int movingMeanL = 0;
int movingMeanR = 0;


void setup() {
	pinMode(DIST_PIN_L,INPUT);
	pinMode(DIST_PIN_R,INPUT);
	Serial.begin(9600);
}

void getNewMovingMean(int newValue, int& mean) {
	mean = newValue*newValueWeight + mean*(1-newValueWeight);
}

void loop() {
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
	delay(50);
}
