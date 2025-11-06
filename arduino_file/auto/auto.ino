#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>

MPU6050 mpu;
int light_sensor = 2;

Adafruit_INA219 ina219;

// screen
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

const int motorLeft_A = 14;
const int motorLeft_B = 12;
const int motorRight_A = 15;
const int motorRight_B = 13;

Servo myservo;
const int servoPin = 16;

void Forward(int speed){
  analogWrite(motorLeft_A, speed);  
  analogWrite(motorLeft_B, 0);     
  analogWrite(motorRight_A, speed);
  analogWrite(motorRight_B, 0);
}

void Stop(){
  analogWrite(motorLeft_A, 0);  
  analogWrite(motorLeft_B, 0);     
  analogWrite(motorRight_A, 0);
  analogWrite(motorRight_B, 0);
}

int normal_speed = 80;
int max_speed = 100;
int min_speed = 60;
int set_servo = 90;

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

void setup() {
  Serial.begin(115200);
  pinMode(0, INPUT_PULLUP); 
  
  // Initialize I2C on SDA=5, SCL=4
  Wire.begin(5, 4);
  myservo.attach(servoPin);
  myservo.write(set_servo);
  
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip!");
    while (1) { delay(10); }
  }
  Serial.println("INA219 connection successful!");

  // Check if the MPU6050 is connected
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  start_screen();
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

bool is_run = false;

void loop() {
  float busVoltage = ina219.getBusVoltage_V();  // Voltage (V)
  float current_mA = ina219.getCurrent_mA();    // Current (mA)
  float power_mW = ina219.getPower_mW();        // Power (mW)
  show_data(busVoltage, current_mA, power_mW );

  if(digitalRead(0) == 0){
    is_run = !is_run;
    delay(300);
  }

  if (is_run){
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int light_val = analogRead(light_sensor);

    // Read raw values
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to acceleration in g and gyro in deg/s (approx)
    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    int speed;
    if(light_val < 100){ speed = min_speed; }
    else if( accelX > 0.22 ) { speed = max_speed; }
    else if( accelX < -0.22 ) { speed = min_speed; }
    else { speed = normal_speed; }
    myservo.write(set_servo);

    Forward(speed = speed);
  }else{
    Stop();
  }

}

  // Serial.print("Light (LX): ");
  // Serial.print(light_val); Serial.print(", ");
  // Serial.print("Accel (g): ");
  // Serial.print(accelX); Serial.print(", ");
  // Serial.print(accelY); Serial.print(", ");
  // Serial.print(accelZ);
  // Serial.print(" | Gyro (°/s): ");
  // Serial.print(gyroX); Serial.print(", ");
  // Serial.print(gyroY); Serial.print(", ");
  // Serial.println(gyroZ);