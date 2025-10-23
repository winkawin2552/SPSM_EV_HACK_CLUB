#include <Bluepad32.h>
#include <ESP32Servo.h>

//---------------------Controller set up--------------------

ControllerPtr myController = nullptr;

// Button constants for PS4 controller
#define BUTTON_CROSS    0x0001
#define BUTTON_CIRCLE   0x0002
#define BUTTON_SQUARE   0x0004
#define BUTTON_TRIANGLE 0x0008
#define BUTTON_L1       0x0010
#define BUTTON_R1       0x0020
#define BUTTON_L2       0x0040
#define BUTTON_R2       0x0080

// D-pad directions
#define DPAD_UP        0x01
#define DPAD_RIGHT     0x04
#define DPAD_DOWN      0x02
#define DPAD_LEFT      0x08

void onConnectedController(ControllerPtr ctl) {
  myController = ctl;
  Serial.printf("Controller connected: %s\n", ctl->getModelName().c_str());
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected");
  myController = nullptr;
}

//------------------------Car set up------------------------

const int motorLeft_A = 14;
const int motorLeft_B = 12;
const int motorRight_A = 15;
const int motorRight_B = 13;

const int max_speed = 255;
const int max_turn = 35;
const int DEADZONE = 20;  // ignore small joystick drift

Servo myservo;
const int servoPin = 16;

//---------------------Driving Functions---------------------

void Forward(){
  analogWrite(motorLeft_A, max_speed);  
  analogWrite(motorLeft_B, 0);     
  analogWrite(motorRight_A, max_speed);
  analogWrite(motorRight_B, 0);
}

void Backward(){
  analogWrite(motorLeft_A, 0);     
  analogWrite(motorLeft_B, max_speed); 
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, max_speed);
}

void StopMotors(){
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, 0);
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

//---------------------Setup---------------------

void setup() {
  Serial.begin(115200);

  // Controller setup
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  Serial.println("Ready to pair your PS4 controller...");

  // Motor pin setup
  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);

  // Servo setup
  myservo.attach(servoPin);
  myservo.write(90);  // center
}

//---------------------Loop---------------------

void loop() {
  BP32.update();

  if (myController && myController->isConnected()) {
    int y = myController->axisY();   // forward/backward
    int rx = myController->axisRX(); // steering

    switch (myController->dpad()) {
      case DPAD_UP: Forward(); break;
      case DPAD_DOWN: Backward(); break;
      default: StopMotors(); break;
    }

    // Steering with deadzone
    if (abs(rx) < DEADZONE) rx = 0;
    int angle = map(rx, -512, 512, 90 - max_turn, 90 + max_turn);
    myservo.write(angle);

    // Debug info
    Serial.printf("Y=%d RX=%d Angle=%d\n", y, rx, angle);
  }
}