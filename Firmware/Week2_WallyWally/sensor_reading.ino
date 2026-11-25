const int trigPin = 10;
const int echoPin = 9;

void setup(){
  pinMode(trigPin, OUTPUT);

  pinMode(echoPin, INPUT);

  Serial.begin(31250);

}

void loop(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long echoTime = pulseIn(echoPin, HIGH, 30000);

  if (echoTime == 0) {
    Serial.println("No echo");

  } else{
    float distanceCm = echoTime * 0.0343 / 2.0;

    Serial.print(distanceCm, 1);

    Serial.println(" cm");

  }
  delay(250);
}