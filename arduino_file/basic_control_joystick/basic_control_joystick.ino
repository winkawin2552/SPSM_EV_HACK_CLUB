/*********
 Electric (EV) & Autonomous vehicle (AV) module kit
 By FABLab Bangkok
*********/

#include <Ps3Controller.h>
#include <ESP32Servo.h>
int player = 3;   // Variables for changinh ID player from 1 into 3
int battery = 0;  // Variable of battery
// Define channels for each motor
const int motorLeft_A = 13;   // Motor pin for controlling
const int motorLeft_B = 12;   // Motor pin for controlling
const int motorRight_A = 25;  // Motor pin for controlling
const int motorRight_B = 26;  // Motor pin for controlling
// Variables for left Analog values
int leftX = 0;  // intitial X variable of Left analog button
int leftY = 0;  // intitial Y variable of Left analog button
//min & max range of Analog values
int min_lx = -50;  // 
int max_lx = 50;   // 
int min_ly = -10;  // 
int max_ly = 10;   // 
// Variables for right Analog values
int rightX = 0;  // intitial X variable of Right analog button
int rightY = 0;  // intitial Y variable of Right analog button
//min & max range of Analog values
int min_rx = -50;  // 
int max_rx = 50;   // 
int min_ry = -10;  // 
int max_ry = 10;   //
//-----------------------------------------------------------------------------------------
Servo myservo;  // Create servo object to control a servo
int pos = 0;  // Variable to store the servo position
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin = 17;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
int servoPin = 7;
#else
int servoPin = 16; // Setup Servo Pin 
#endif
void setup() {
  Serial.begin(115200);
  Ps3.begin("d0:c6:37:a6:a6:16");  // configurate MAC address of Joystick and ESP32 (Install SixaxisPairTool for equipment mapping) https://www.filehorse.com/download-sixaxispairtool/ 
  Serial.println("Ready.");
  // Set motor controller pins as outputs
  pinMode(motorLeft_A, OUTPUT);          //  Pin motorA_IN1 as OUTPUT
  pinMode(motorLeft_B, OUTPUT);          //  Pin motorA_IN2 as OUTPUT
  pinMode(motorRight_A, OUTPUT);         //  Pin motorB_IN1 as OUTPUT
  pinMode(motorRight_B, OUTPUT);         //  Pin motorB_IN2 as OUTPUT
  myservo.setPeriodHertz(50);            //  standard 50 hz of servo
  myservo.attach(servoPin, 1000, 2000);  //  attaches the servo on pin 16 to the servo object
  myservo.write(90);                     //  setup servo motor to go to position (0-180 degree) in variable 'pos' (pos=90(Stop), 90 < Pos < 180 (turn right), Pos< 90 (Turn left))                     
}
void Forward_L(int speed) {
  analogWrite(motorLeft_A, speed);
  analogWrite(motorLeft_B, 0);
}
void Backward_L(int speed) {
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, speed);
}
void Forward_R(int speed) {
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}
void Backward_R(int speed) {
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, speed);
}
void Stop_R(int speed) {
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 0);
}
void Stop_L(int speed) {
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 0);
}
void loop() {
   static uint32_t lastSend = 0;
  static bool wasConnected = false;
  const bool connected = Ps3.isConnected();
  if (connected && !wasConnected) {
    Ps3.setPlayer(player);  // set ครั้งเดียวพอ
    Serial.println("PS3 Connected");
  }
  wasConnected = connected;
  // Delay every 50 millisecond
  if (Ps3.isConnected() && millis() - lastSend > 50) {
    battery = Ps3.data.status.battery;  // Measuring battery status
    Serial.print("The controller battery is ");
    if (battery == ps3_status_battery_charging) Serial.println("charging");
    else if (battery == ps3_status_battery_full) Serial.println("FULL");
    else if (battery == ps3_status_battery_high) Serial.println("HIGH");
    else if (battery == ps3_status_battery_low) Serial.println("LOW");
    else if (battery == ps3_status_battery_dying) Serial.println("DYING");
    else if (battery == ps3_status_battery_shutdown) Serial.println("SHUTDOWN");
    else Serial.println("UNDEFINED");
    ///////////////////////////////////Reading Analog Joy ///////////////////////////////////
    // Get Left analog value
    leftX = (Ps3.data.analog.stick.lx);  
    leftY = (Ps3.data.analog.stick.ly);  
    // Display Left analog values
    Serial.print(leftX);
    Serial.print(", ");
    Serial.print(leftY);
    Serial.print(", ");
    // Get Right analog value
    rightX = (Ps3.data.analog.stick.rx);  
    rightY = (Ps3.data.analog.stick.ry);  
    // Display Right analog values
    Serial.print(rightX);
    Serial.print(", ");
    Serial.println(rightY);
    //-----------------------------------------------------------------------------------------
    // When joystick and ESP32 board bluetoo connect already, Progran is running
    // Pad UP, DOWN, LEFT, RIGHT
    if (Ps3.data.button.up) {   // Push Pad UP, motor move forward
      Serial.println("forward");
      Forward_L(125);
      Forward_R(125);
    }
    if (Ps3.data.button.down) { // Push Pad Down, motor move backward
      Serial.println("backward");
      Backward_L(100);
      Backward_R(100);
    }
    if (Ps3.data.button.left) { // Push Pad Left, motor turn left
      myservo.write(60);        // setup servo motor moving to position in variable 'pos' and turn lef 30 degree from 90 degree axis
      Serial.println("left");
    }
    if (Ps3.data.button.right) { // Push Pad Right, motor turn right
      myservo.write(120);        // setup servo motor moving to position in variable 'pos' and turn right 30 degree from 90 degree axis
      Serial.println("right");
    }
    if (Ps3.data.button.select) { // Push Pad Select, Choosing mode 1. AI mode 2. Manual mode 
      Serial.println("AI mode");
      Forward_L(0);
      Forward_R(0);
    }
    
    if (!Ps3.data.button.right && !Ps3.data.button.left && !Ps3.data.button.down && !Ps3.data.button.up) {
      Serial.println("stop");  // No pressing, it stop movement
      myservo.write(90);       // setup servo motor stop position in variable 'pos'
      Stop_R(0);
      Stop_L(0);
    }
  }
}