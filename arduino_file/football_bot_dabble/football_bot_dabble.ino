#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <ESP32Servo.h>

// ---------------- Car Setup ----------------
const int motorLeft_A  = 14;
const int motorLeft_B  = 12;
const int motorRight_A = 15;
const int motorRight_B = 13;

// ---------------- Servo ----------------
Servo myservo;
const int servoPin = 16;
const int center_servo = 90;
const int max_turn = 30;

// ---------------- Speed ----------------
int robotSpeed = 120;   // default speed (0–255)

// ---------------- MOTOR FUNCTIONS (FROM CODE 1) ----------------
void Forward(int speed) {
  analogWrite(motorLeft_A, speed);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}

void Backward(int speed) {
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, speed);
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, speed);
}

void StopMotors() {
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 0);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);

  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);

  myservo.attach(servoPin);
  myservo.write(center_servo);

  StopMotors();

  Dabble.begin("MyRobot");  // Bluetooth name
  Serial.println("Dabble Ready");
}

// ---------------- LOOP ----------------
void loop() {
  Dabble.processInput();

  // -------- Speed Control --------
  if (GamePad.isTrianglePressed()) {
    robotSpeed = min(robotSpeed + 5, 255);
    delay(150);
  }
  if (GamePad.isCrossPressed()) {
    robotSpeed = max(robotSpeed - 5, 0);
    delay(150);
  }

  // -------- Movement --------
  if (GamePad.isUpPressed()) {
    Forward(robotSpeed);
  }
  else if (GamePad.isDownPressed()) {
    Backward(robotSpeed);
  }
  else {
    StopMotors();
  }

  // -------- Servo Steering (KEY PART) --------
  if (GamePad.isSquarePressed()) {
    myservo.write(center_servo - max_turn);  // turn left
  }
  else if (GamePad.isCirclePressed()) {
    myservo.write(center_servo + max_turn);  // turn right
  }
  else {
    myservo.write(center_servo);              // center
  }

  Serial.print("Speed: ");
  Serial.print(robotSpeed);
  Serial.print(" | Servo: ");
  Serial.println(myservo.read());
}
