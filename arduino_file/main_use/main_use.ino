#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>
#include <Wire.h>

//--------------------- Sensor set up ----------------------
Adafruit_INA219 ina219;

// screen
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

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
const int max_turn = 30;
const int DEADZONE = 20;  // ignore small joystick drift

Servo myservo;
const int servoPin = 16;

//---------------------Driving Functions---------------------

void Forward(int speed = 512){
  speed = map(speed, 0, 512, 0, max_speed);
  analogWrite(motorLeft_A, speed);  
  analogWrite(motorLeft_B, 0);     
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}

void Backward(int speed = 512){
  speed = map(speed, 0, 512, 0, max_speed);
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

void TurnLeft(int val){
  int angle = map(val, 0, 512, 90, 90 - max_turn);
  myservo.write(angle);
}

void TurnRight(int val){
  int angle = map(val, 0, 512, 90, 90 + max_turn);
  myservo.write(angle);
}

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
void start_screen(){
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20);  // Move cursor to x=0, y=20
  display.println("START");
  display.display();
}
//---------------------Setup---------------------

void setup() {
  Serial.begin(115200);
  Wire.begin(5, 4);

  // Servo setup
  myservo.attach(servoPin);
  myservo.write(90);  // center
  StopMotors();

  Serial.println("Initializing INA219...");
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip!");
    while (1) { delay(10); }
  }
  Serial.println("INA219 connection successful!");

  // Controller setup
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  Serial.println("Ready to pair your PS4 controller...");

  // Motor pin setup
  pinMode(motorLeft_A, OUTPUT);
  pinMode(motorLeft_B, OUTPUT);
  pinMode(motorRight_A, OUTPUT);
  pinMode(motorRight_B, OUTPUT);


  start_screen();
  delay(2000);
}


void show_data(float busVoltage, float current_mA, float power_mW){
  display.clearDisplay();
  display.setTextSize(2);   
  display.setCursor(0, 0);  // initial position (X, Y)
  display.print("V: ");
  display.print(busVoltage, 2);
  display.println("V");

  // Display current
  display.setTextSize(2);    
  display.setCursor(0, 24);  
  display.print("I: ");
  display.print(current_mA, 0);
  display.println("mA");

  // Display power
  display.setTextSize(2);    
  display.setCursor(0, 48);  
  display.print("P: ");
  display.print(power_mW / 1000.0, 2);
  display.println("W");
  display.display();
}

//---------------------Loop---------------------
bool is_open = true;
void loop() {
  BP32.update();

  float busVoltage = ina219.getBusVoltage_V();  // Voltage (V)
  float current_mA = ina219.getCurrent_mA();    // Current (mA)
  float power_mW = ina219.getPower_mW();        // Power (mW)
  uint16_t btn = myController->buttons();

  Serial.print(" | busVoltage: ");
  Serial.print(busVoltage);
  Serial.print(" V, current_mA: ");
  Serial.print(current_mA);
  Serial.print(" mA, power_mW: ");
  Serial.print(power_mW);
  Serial.println(" mW");

  if (btn & BUTTON_CIRCLE){
    is_open = !is_open;
    display.clearDisplay();
    display.display();
    delay(300);
  }
  
  if (myController && myController->isConnected()) {
    if (is_open) show_data(busVoltage, current_mA, power_mW );
    int y = myController->axisY();   // forward/backward
    int rx = myController->axisRX(); // steering

    // Forward/backward with deadzone
    if (y < -DEADZONE) Forward(-y);
    else if (y > DEADZONE) Backward(y);
    else if(myController->dpad() == DPAD_UP) Forward();
    else if(myController->dpad() == DPAD_DOWN) Backward();
    else StopMotors();
    

    // Steering with deadzone
    if (abs(rx) < DEADZONE) rx = 0;
    int angle = map(rx, -512, 512, 90 - max_turn, 90 + max_turn);
    myservo.write(angle);

    // Debug info
    Serial.printf("Y=%d RX=%d Angle=%d\n", y, rx, angle);
  }else{
    if (is_open) start_screen();
    StopMotors();
  }
}