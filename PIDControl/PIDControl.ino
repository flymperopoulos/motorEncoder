int sensorPin = A1;

float data;

void setup(){
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
}

void loop(){
  data = analogRead(sensorPin);
  Serial.println(data);
  delay(100);
}
