const int motorLeft_A = 14;
const int motorLeft_B = 12;
const int motorRight_A = 15;
const int motorRight_B = 13;

void setup() {
  // Set motor pins as outputs
  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);
}

void loop() {
  // Forward
  analogWrite(motorLeft_A, 255);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, 255);
  analogWrite(motorRight_B, 0);
  delay(2000);

  // Backward
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 255);
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 255);
  delay(2000);

  // Stop
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 0);
  delay(1000);
}