// Arduino 1 Code

void setup() {
  Serial.begin(9600);
  //pinMode(4, OUTPUT); // Set pin 4 as OUTPUT
  pinMode(2, OUTPUT);
}

void loop() {
  //digitalWrite(4, HIGH); // Send a HIGH signal on pin 4
  digitalWrite(2, HIGH); // Send a HIGH signal on pin 4
  delay(5000); // Wait for a second
  //digitalWrite(4, LOW); // Send a LOW signal on pin 4
  digitalWrite(2, LOW);
  delay(5000); // Wait for a second

}
