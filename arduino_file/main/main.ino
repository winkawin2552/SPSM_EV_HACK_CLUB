#include <Bluepad32.h>
#include <ESP32Servo.h>

//---------------------Controller set up--------------------

// Controller pointer
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

//---------------------Controller set up--------------------

//------------------------Car set up------------------------

const int motorLeft_A = 13;
const int motorLeft_B = 12;
const int motorRight_A = 25;
const int motorRight_B = 26;

const int max_speed = 255;
const int max_turn = 60;

Servo myservo;
const int servoPin = 16

//------------------------Car set up------------------------

//---------------------Driving Function---------------------

void Forward(int speed){

  analogWrite(motorLeft_A, speed);
  analogWrite(motorLeft_B, 0;
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);

}

void Backward(int speed){


}

//---------------------Driving Function---------------------

void setup() {
  Serial.begin(115200);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // optional, helps reconnect
  Serial.println("Ready to pair your PS4 controller...");
}

void loop() {
  BP32.update();  // always call update in loop

  if (myController && myController->isConnected()) {
    uint16_t btn = myController->buttons();

    // Print buttons
    if (btn & BUTTON_CROSS)    Serial.println("Cross Button");
    if (btn & BUTTON_CIRCLE)   Serial.println("Circle Button");
    if (btn & BUTTON_SQUARE)   Serial.println("Square Button");
    if (btn & BUTTON_TRIANGLE) Serial.println("Triangle Button");
    if (btn & BUTTON_L1)       Serial.println("L1");
    if (btn & BUTTON_R1)       Serial.println("R1");

    // Print triggers
    if (myController->l2() > 0) Serial.printf("L2: %d\n", myController->l2());
    if (myController->r2() > 0) Serial.printf("R2: %d\n", myController->r2());

    // Print sticks
    Serial.printf("Left Stick:  X=%d Y=%d\n", myController->axisX(), myController->axisY());
    Serial.printf("Right Stick: X=%d Y=%d\n", myController->axisRX(), myController->axisRY());

    // Print D-pad
    switch (myController->dpad()) {
      case DPAD_UP: Serial.println("D-pad Up"); break;
      case DPAD_DOWN: Serial.println("D-pad Down"); break;
      case DPAD_LEFT: Serial.println("D-pad Left"); break;
      case DPAD_RIGHT: Serial.println("D-pad Right"); break;
    }

    Serial.println(); // blank line for readability
    delay(200);       // small delay to make output readable
  }
}
