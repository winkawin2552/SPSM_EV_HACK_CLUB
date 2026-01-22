#include <ESP32Servo.h>
#include <Wire.h>

const int motorLeft_A = 14;
const int motorLeft_B = 12;
const int motorRight_A = 15;
const int motorRight_B = 13;


Servo myservo;
const int servoPin = 16;

int speed;
const int max_speed = 255;
const int max_turn = 30;

void Forward(int speed = 512){
  speed = map(speed, 0, 512, 0, max_speed);
  analogWrite(motorLeft_A, speed);  
  analogWrite(motorLeft_B, 0);     
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}

void TurnLeft(int val){
  int angle = map(val, 0, 512, 90, 90 - max_turn);
  myservo.write(angle);
}

void TurnRight(int val){
  int angle = map(val, 0, 512, 90, 90 + max_turn);
  myservo.write(angle);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(5, 4);

  // Servo setup
  myservo.attach(servoPin);
  myservo.write(90);  // center

  // Motor pin setup
  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);

}

void loop() {
  Forward();
  TurnLeft(25);
  delay(500);
  TurnRight(25);
  delay(500);

}
