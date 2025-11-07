#include <Bluepad32.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

#define BUTTON_CIRCLE 0x0002

MPU6050 mpu;
int light_sensor = 2;

const int motorLeft_A = 14;
const int motorLeft_B = 12;
const int motorRight_A = 15;
const int motorRight_B = 13;

// Controller pointer
ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
  myController = ctl;
  Serial.printf("Controller connected: %s\n", ctl->getModelName().c_str());
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected");
  myController = nullptr;
}

Servo myservo;
const int servoPin = 16;

void Forward(int speed) {
  analogWrite(motorLeft_A, speed);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}

void Stop() {
  analogWrite(motorLeft_A, 0);
  analogWrite(motorLeft_B, 0);
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 0);
}

int normal_speed = 100;
int max_speed = 120;
int min_speed = 80;
int set_servo = 89;
int DEADZONE = 20;
int max_turn = 30;

bool is_run = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(5, 4);
  Stop();
  pinMode(0, INPUT_PULLUP);
  myservo.attach(servoPin);
  myservo.write(set_servo);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false); // prevent DS4 virtual error
  BP32.forgetBluetoothKeys();
  Serial.println("Ready to pair your PS4 controller...");
}

void loop() {
  BP32.update();

  if (myController && myController->isConnected()) {
    uint16_t btn = myController->buttons();

    // Also allow toggling with Circle button
    if (btn & BUTTON_CIRCLE) {
      is_run = !is_run;
      delay(300);
    }

    // Declare these here so we can print them later
    int light_val = 0;
    float accelX = 0.0;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (is_run) {

      light_val = analogRead(light_sensor);
      int rx = myController->axisRX();

      accelX = ax / 16384.0;

      // Steering with deadzone
      if (abs(rx) < DEADZONE) rx = 0;
      int angle = map(rx, -512, 512, 90 - max_turn, 90 + max_turn);
      myservo.write(angle);

      int speed;
      if (light_val < 20) {
        speed = min_speed;
      } else if (accelX > 0.20) {
        speed = max_speed;
      } else if (accelX < 0.13) {
        speed = min_speed;
      } else {
        speed = normal_speed;
      }

      Forward(speed);
    } else {
      Stop();
    }

    // ✅ Now accessible to print safely
    Serial.printf("light: %d | servo: %d | accelX: %f | is_run: %d\n",
                  analogRead(light_sensor),
                  myController->axisRX(),
                  accelX,
                  is_run);
  } else {
    Stop();
  }
}
