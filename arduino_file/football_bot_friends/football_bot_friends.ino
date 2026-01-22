#include <Wire.h>
#include <Ps3Controller.h>
#include <ESP32Servo.h>


// ---------------- PS3 ----------------
int player = 3;

// ---------------- Car Setup ----------------
const int motorLeft_A = 14;
const int motorLeft_B = 12;
const int motorRight_A = 15;
const int motorRight_B = 13;

const int max_speed = 255;
const int max_turn = 30;
const int DEADZONE = 20;  // ignore small joystick drift
const int center_servo = 90;

Servo myservo;
const int servoPin = 16;

// ---------------- MOTOR FUNCTIONS ----------------
void Forward(int speed = 255){
  analogWrite(motorLeft_A, speed);  
  analogWrite(motorLeft_B, 0);     
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}

void Backward(int speed = 255){
  analogWrite(motorLeft_A, 0);     
  analogWrite(motorLeft_B, speed); 
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, speed);
}

void StopMotors(){
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 0);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(5, 4);
  StopMotors();

  // PS3
  Ps3.begin("a5:9a:5a:a2:11:e1");
  Serial.println("Ready for PS3 Controller");

  // Motors
  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);

  // Servo
  myservo.attach(servoPin);
  myservo.write(center_servo);  // center
  StopMotors();
}

// ---------------- LOOP ----------------
void loop() {
  static bool wasConnected = false;
  bool connected = Ps3.isConnected();

  if (connected && !wasConnected) {
    Ps3.setPlayer(player);
    Serial.println("PS3 Connected!");
  }
  wasConnected = connected;

  if (!connected) return;

// -------- Joystick Motor --------
  int joyY = Ps3.data.analog.stick.ly;
  int joyX = Ps3.data.analog.stick.rx;
  
  // 1. Correct the speed mapping to reach full 255
  int speed = map(abs(joyY), 0, 128, 0, 255); 

  // 2. Use the 'speed' variable instead of 'joyY'
  if (joyY < -DEADZONE) {
    Forward(speed);  // Uses the 0-255 value
  } 
  else if (joyY > DEADZONE) {
    Backward(speed); // Uses the 0-255 value
  } 
  else {
    StopMotors();
  }

  // -------- Servo Control --------
  if (abs(joyX) < DEADZONE) joyX = 0;
  int angle = map(joyX, -128, 128, center_servo - max_turn, center_servo + max_turn);
  myservo.write(angle);
  Serial.printf("X: %d, Y: %d, Speed: %d, Angle: %d\n", joyX, joyY, speed, angle);

  // -------- Stop Button --------
  if (Ps3.data.button.select) {
    StopMotors();
    myservo.write(90);
  }
}
