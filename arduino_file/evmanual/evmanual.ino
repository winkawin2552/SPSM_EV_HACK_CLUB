/*********
 Electric (EV) & Autonomous vehicle (AV) module kit
 By FABLab Bangkok
*********/
#include <Ps3Controller.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include <Preferences.h>   
int player = 3;           // Variables for changinh ID player from 1 into 3
int battery = 0;          // Variable of battery
#define light 2           // analog pin header of light sensor
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// Define channels for each motor
const int motorLeft_A = 13;   // Motor pin for controlling
const int motorLeft_B = 12;   // Motor pin for controlling
const int motorRight_A = 25;  // Motor pin for controlling
const int motorRight_B = 26;  // Motor pin for controlling
// Light sensor constant
const float ADC_REF_VOLTAGE = 5.0;  // Reference voltage (adjust if 3.3V)
const int ADC_RESOLUTION = 4095;    // For 12-bit ADC
unsigned int time_start = 0;
unsigned int time_last = 0;
unsigned int time_current = 0;
float busVoltage = 0;  // Voltage (V)
float current_mA = 0;  // Current (mA)
float power_mW = 0;    // Power (mW)
int light_value;       // Light sensor value 0-4095

Adafruit_INA219 ina219;        // Battery monitoring sensor
static uint32_t lastImu = 0;   // every  100 ms
static uint32_t lastSlow = 0;  // every 1000 ms
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The -1 parameter means no reset pin, which is common for built-in OLEDs.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// 'FabLab Bangkok Logo Black', 128x64px
const unsigned char FABLAB[] PROGMEM = {
  0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xfe, 0x02, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xf8, 0x0f, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xf0, 0x1f, 0x80, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xc0, 0x3f, 0xe0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xc0, 0x7f, 0xe0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x80, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x00, 0x7f, 0xfc, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff,
  0xfe, 0x00, 0x39, 0xfe, 0x07, 0xff, 0x80, 0xf3, 0xf0, 0x1e, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff,
  0xfe, 0x00, 0x00, 0x7f, 0x03, 0xff, 0x80, 0xf1, 0xf0, 0x0e, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff,
  0xfc, 0x00, 0x00, 0x1f, 0x83, 0xff, 0x9f, 0xf1, 0xf3, 0xce, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff,
  0xf8, 0x70, 0x00, 0x0f, 0xe1, 0xff, 0x9f, 0xe1, 0xf3, 0xce, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff,
  0xf8, 0xf8, 0x00, 0x0f, 0xf0, 0xff, 0x9f, 0xe1, 0xf3, 0xce, 0xfe, 0x1c, 0x8f, 0xff, 0xff, 0xff,
  0xf1, 0xfc, 0x00, 0x07, 0xf8, 0xff, 0x9f, 0xe4, 0xf3, 0xce, 0xfd, 0xdc, 0x67, 0xff, 0xff, 0xff,
  0xfb, 0xfc, 0x00, 0x07, 0xfc, 0xff, 0x9f, 0xcc, 0xf0, 0x1e, 0xff, 0xcc, 0xe7, 0xff, 0xff, 0xff,
  0xff, 0xfc, 0x00, 0x07, 0xff, 0xff, 0x80, 0xcc, 0xf0, 0x1e, 0xff, 0xcc, 0xf7, 0xff, 0xff, 0xff,
  0xff, 0xfc, 0x00, 0x07, 0xff, 0xbf, 0x80, 0xce, 0x73, 0xce, 0xff, 0x0c, 0xf7, 0xff, 0xff, 0xff,
  0xef, 0xfc, 0x00, 0x07, 0xff, 0x3f, 0x9f, 0xce, 0x73, 0xee, 0xfe, 0x4c, 0xf7, 0xff, 0xff, 0xff,
  0xcf, 0xfe, 0x00, 0x0f, 0xff, 0x1f, 0x9f, 0x80, 0x73, 0xe6, 0xfc, 0xcc, 0xf7, 0xff, 0xff, 0xff,
  0xcf, 0xff, 0x00, 0x0f, 0xff, 0x1f, 0x9f, 0x80, 0x33, 0xe6, 0xfd, 0xcc, 0xf7, 0xff, 0xff, 0xff,
  0xcf, 0xff, 0x00, 0x1f, 0xff, 0x1f, 0x9f, 0x9f, 0x33, 0xce, 0xfd, 0xcc, 0xf7, 0xff, 0xff, 0xff,
  0x8f, 0xff, 0x80, 0x3f, 0xff, 0x1f, 0x9f, 0x3f, 0x30, 0x0e, 0x04, 0xcc, 0x67, 0xff, 0xff, 0xff,
  0x8f, 0xff, 0xf0, 0xff, 0xff, 0x0f, 0x9f, 0x3f, 0x30, 0x1e, 0x06, 0x2d, 0x0f, 0xff, 0xff, 0xff,
  0x8f, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0xc7, 0xff, 0xfc, 0x18, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x0f, 0x83, 0xff, 0xf8, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x0f, 0x01, 0xff, 0xf8, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x0f, 0x00, 0xff, 0xf0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x0f, 0x00, 0x7f, 0xe0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0x00, 0x7f, 0xe0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0x00, 0x3f, 0xc0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x8f, 0x00, 0x3f, 0xc0, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xfc, 0xff, 0xff,
  0x8f, 0x00, 0x3f, 0xc0, 0x01, 0x0f, 0x80, 0xfe, 0x7e, 0x7c, 0xf8, 0x1e, 0x7c, 0xf0, 0x3e, 0xf9,
  0x8f, 0x00, 0x1f, 0xc0, 0x03, 0x0f, 0x80, 0x7c, 0x7e, 0x7c, 0xf0, 0x0e, 0x79, 0xe0, 0x1e, 0xf3,
  0x8f, 0x00, 0x1f, 0x80, 0x07, 0x0f, 0x9e, 0x7c, 0x3e, 0x3c, 0xe3, 0xce, 0x71, 0xc7, 0x9e, 0xf3,
  0x8f, 0x00, 0x1f, 0x80, 0x0f, 0x0f, 0x9e, 0x7c, 0x3e, 0x1c, 0xe7, 0xfe, 0x73, 0xcf, 0x8e, 0xe7,
  0x8f, 0x80, 0x1f, 0x80, 0x0f, 0x1f, 0x9e, 0x79, 0x3e, 0x1c, 0xe7, 0xfe, 0x67, 0x9f, 0xce, 0xe7,
  0xcf, 0x80, 0x1f, 0x80, 0x0f, 0x1f, 0x9c, 0x79, 0x1e, 0x0c, 0xcf, 0xfe, 0x67, 0x9f, 0xce, 0xcf,
  0xcf, 0xc0, 0x1f, 0x80, 0x0f, 0x1f, 0x80, 0xf9, 0x9e, 0x4c, 0xcf, 0xfe, 0x47, 0x9f, 0xee, 0xcf,
  0xcf, 0xc0, 0x1f, 0x80, 0x1f, 0x1f, 0x80, 0xf9, 0x9e, 0x44, 0xcf, 0x8e, 0x47, 0x9f, 0xee, 0x8f,
  0xef, 0xc0, 0x1f, 0x80, 0x1f, 0x3f, 0x9e, 0x73, 0x9e, 0x64, 0xcf, 0x8e, 0x07, 0x9f, 0xce, 0x07,
  0xef, 0xc0, 0x3f, 0xc0, 0x3f, 0x3f, 0x9e, 0x73, 0x8e, 0x70, 0xef, 0xee, 0x13, 0x9f, 0xce, 0x27,
  0xe7, 0xc0, 0x3f, 0xc0, 0x7c, 0x3f, 0x9f, 0x70, 0x0e, 0x70, 0xe7, 0xee, 0x33, 0x9f, 0xce, 0x63,
  0xf3, 0xc0, 0x7f, 0xe0, 0xf8, 0x7f, 0x9e, 0x60, 0x0e, 0x78, 0xe7, 0xee, 0x79, 0xcf, 0x8e, 0x73,
  0xf0, 0xc0, 0xff, 0xf1, 0xf0, 0x7f, 0x9e, 0x67, 0xc6, 0x78, 0xf3, 0xce, 0x79, 0xc7, 0x1e, 0xf3,
  0xf8, 0x40, 0xff, 0xff, 0xe0, 0xff, 0x80, 0x67, 0xe6, 0x7c, 0xf0, 0x0e, 0x7c, 0xe0, 0x3e, 0xf9,
  0xf8, 0x00, 0xff, 0xff, 0xc0, 0xff, 0x80, 0xe7, 0xe6, 0x7c, 0xfc, 0x1e, 0x7c, 0xf0, 0x7e, 0xf9,
  0xfc, 0x00, 0xff, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xfc, 0x00, 0xff, 0xfe, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xfe, 0x00, 0xff, 0xfc, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x00, 0xff, 0xf8, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x00, 0xff, 0xf0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x80, 0x7f, 0xe0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xc0, 0x3f, 0x80, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xf0, 0x1f, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xf8, 0x0e, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xfc, 0x06, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0x06, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
//-----------------------------------------------------------------------------------------
Servo myservo;  // Create servo object to control a servo
int pos = 0;    // Variable to store the servo position
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin = 17;
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
int servoPin = 7;
#else
int servoPin = 16;  // Setup Servo Pin
#endif
// ---------- Your ranges & scales ----------
//int16_t axr, ayr, azr, gxr, gyr, gzr;
const uint8_t ACCEL_RANGE = 1;  // 0:±2g, 1:±4g, 2:±8g, 3:±16g
const uint8_t GYRO_RANGE = 1;   // 0:±250, 1:±500, 2:±1000, 3:±2000 dps
// Scale factors (LSB per unit) for the chosen ranges:
const float ACC_SCALE = 8192.0f;  // LSB/g for ±4g
const float GYR_SCALE = 65.5f;    // LSB/(°/s) for ±500 dps

// Optional: enter your factory/calibrated offsets (in raw LSB units)
const int16_t AX_OFF = 0, AY_OFF = 0, AZ_OFF = 0;
const int16_t GX_OFF = 0, GY_OFF = 0, GZ_OFF = 0;
// Only zero if nearly level? (optional)
const bool     REQUIRE_FLAT   = false;
const float    FLAT_DEG       = 3.0f;
// Motion/zero detection (tune for your car)
const float    GYRO_STILL_DPS = 1.2f;     // consider still if |gyro| RMS < this
const float    ACC_MAG_LO     = 0.95f;    // |a| near 1g window
const float    ACC_MAG_HI     = 1.05f;
const uint32_t STILL_MS       = 3000;     // must be this still to auto-zero
// Complementary filter state
float alpha = 0.98f;     // 0.96–0.99; higher = more gyro trust
MPU6050 mpu;  //IMU sensor
Preferences prefs; // Preferrence for store data
float roll=0.0f, pitch=0.0f;      // fused absolute angles (deg)
float roll0=0.0f, pitch0=0.0f;    // saved baseline
bool  haveBaseline=false;
uint32_t lastMicros = 0;  // timestamp setup
uint32_t stillStartMs=0;
static inline float rad2deg(float r) {
  return r * 180.0f / PI;
}
// Raw->scaled using your scales (±4g, ±500 dps)
void readScaled(float &ax, float &ay, float &az, float &gx, float &gy, float &gz){
  int16_t rax, ray, raz, rgx, rgy, rgz;
  mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
  ax = rax / ACC_SCALE;   // g
  ay = ray / ACC_SCALE;
  az = raz / ACC_SCALE;
  gx = rgx / GYR_SCALE;   // deg/s
  gy = rgy / GYR_SCALE;
  gz = rgz / GYR_SCALE;
}
// Accel-only attitude from gravity vector
void accelAngles(float ax, float ay, float az, float &accRoll, float &accPitch){
  accRoll  = rad2deg(atan2f(ay, az));                       // X roll
  accPitch = rad2deg(atan2f(-ax, sqrtf(ay*ay + az*az)));    // Y pitch
}
bool isStationary(float gx, float gy, float gz, float ax, float ay, float az){
  float gmag = sqrtf(gx*gx + gy*gy + gz*gz);
  float amag = sqrtf(ax*ax + ay*ay + az*az);
  return (gmag < GYRO_STILL_DPS) && (amag > ACC_MAG_LO) && (amag < ACC_MAG_HI);
}
// prefs.begin ("index", function (true or false); false = read + write, true = read-only
void saveBaseline(){
  prefs.begin("imu", false);
  prefs.putBool("hasBase", true);
  prefs.putFloat("r0", roll0);
  prefs.putFloat("p0", pitch0);
  prefs.end();
  haveBaseline = true;
  Serial.printf("[IMU] Baseline saved: r0=%.2f p0=%.2f\n", roll0, pitch0);
}
void loadBaseline(){
  prefs.begin("imu", true);
  haveBaseline = prefs.getBool("hasBase", false);
  if (haveBaseline){
    roll0  = prefs.getFloat("r0", 0.0f);
    pitch0 = prefs.getFloat("p0", 0.0f);
    Serial.printf("[IMU] Loaded baseline: r0=%.2f p0=%.2f\n", roll0, pitch0);
  } else {
    Serial.println("[IMU] No saved baseline yet.");
  }
  prefs.end();
}
void manualZero(){
  roll0 = roll; pitch0 = pitch;
  saveBaseline();
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
void oled_output() {
  // --- Displaying into OLED ---
  display.clearDisplay();  // Clear Display when you update information
  // Display Voltage
  display.setTextSize(2);   // Scale 2 time for text size
  display.setCursor(0, 0);  // initial position (X, Y)
  display.print("V: ");
  display.print(busVoltage, 2);
  display.println("V");
  // Display current
  display.setTextSize(2);    // Scale 2 time for text size
  display.setCursor(0, 24);  // Set Cursor
  display.print("I: ");
  display.print(current_mA, 0);
  display.println("mA");
  // Display power
  display.setTextSize(2);    // Scale 2 time for text size
  display.setCursor(0, 48);  // Set Cursor
  display.print("P: ");
  display.print(power_mW / 1000.0, 2);
  display.println("W");
  display.display();  // Displaying into OLED
}
void lux_light() {
  light_value = analogRead(light);
  float voltage = light_value * (ADC_REF_VOLTAGE / ADC_RESOLUTION);
  // Convert to lux (approximation)
  // Formula based on tests: lux ≈ (voltage * 2) * 1000
  // This assumes linear output: ~1V = 1000 lux
  // 1V output ≈ 1000 lux (from empirical measurements)
  //Thus, lux = voltage * 1000 / 0.5 = voltage * 2000 (since at 0.5V ≈ 500 lux)
  float lux = (voltage * 2.0) * 1000;
  Serial.print("Raw Light data: ");
  Serial.print(light_value);
  Serial.print(" | Voltage: ");
  Serial.print(voltage);
  Serial.print(" V | Approx Lux: ");
  Serial.println(lux);
}
void imu_balancing() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = nowUs;
  float ax, ay, az, gx, gy, gz;
  readScaled(ax, ay, az, gx, gy, gz);
  float accR, accP;
  accelAngles(ax, ay, az, accR, accP);
  // complementary filter (swap signs/axes if your board is rotated)
  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * accR;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * accP;

  // manual zero (hold ~300ms)
 // if (digitalRead(BTN_ZERO_PIN) == LOW){
 //   static uint32_t t0=0;
 //   if (!t0) t0 = millis();
 //  if (millis() - t0 > 300) { manualZero(); t0 = millis(); }
 //} else {
 //   static uint32_t dummy=0; dummy=0;
 // }

  // auto-zero when parked (fabsf() is the function that returns the absolute value of a floating-point number of type float)
  bool still = isStationary(gx, gy, gz, ax, ay, az);
  uint32_t now = millis();
  if (still){
    if (!stillStartMs) stillStartMs = now;
    if (now - stillStartMs >= STILL_MS){
      if (!REQUIRE_FLAT || (fabsf(roll) < FLAT_DEG && fabsf(pitch) < FLAT_DEG)){
        roll0 = roll; pitch0 = pitch;
        saveBaseline();
      }
      stillStartMs = now; // prevent repeated triggers
    }
  } else {
    stillStartMs = 0;
  }
  // angles relative to baseline (If condition is true, the expression evaluates to value_if_true)
  float roll_rel  = haveBaseline ? (roll  - roll0) : roll;
  float pitch_rel = haveBaseline ? (pitch - pitch0) : pitch;
  // example slope flags (tune thresholds)
  bool uphill   = (pitch_rel >  3.0f);
  bool downhill = (pitch_rel < -3.0f);
  // debug ~20Hz
  static uint32_t lastPrint=0;
  if (millis() - lastPrint > 50){
    lastPrint = millis();
    Serial.printf("abs r=%.2f p=%.2f | rel r=%.2f p=%.2f %s%s\n",
      roll, pitch, roll_rel, pitch_rel,
      uphill?"[UP] ":"", downhill?"[DOWN] ":"");
  }
}
volatile bool justConnected = false;
void onConnect() {
  // Calling when it is connected successfully
  justConnected = true; 
  Serial.println("PS3 Connected");
}
void onDisconnect() {
  Serial.println("PS3 Disconnected");
  // Stop motors to be safety
  Stop_L(0);
  Stop_R(0);
  myservo.write(90);
}
void setup() {
  Serial.begin(115200);
  Ps3.begin("0c:b8:15:f6:1b:0e");  // configurate MAC address of Joystick and ESP32 (Install SixaxisPairTool for equipment mapping)
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Serial.println("Ready.");
  delay(500);
  // Start I2C Communication SDA = 5 and SCL = 4 on Wemos Lolin32 ESP32 with built-in SSD1306 OLED
  // Make sure these pins match your specific ESP32 board and OLED connection.
  Wire.begin(5, 4);
  // Wire.setClock(400000);  // 400 kHz fast mode
  // Wire.setTimeout(20);    // 20 ms I²C transaction timeout
  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  delay(5000);  // Pause for 2 seconds to allow the display to initialize
  // Clear the display buffer before drawing anything
  display.clearDisplay();
  //Display FABLab Bangkok Logo
  display.drawBitmap(0, 0, FABLAB, 128, 64, SSD1306_WHITE);
  display.display();  // Show the content of the buffer on the OLED
  delay(1000);
  display.clearDisplay();
  // --- Code to display text ---
  // You can add more text with different positions and sizes
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20);  // Move cursor to x=0, y=20
  display.println("WELCOME");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.println("Initializing INA219...");
  // Display everything that's been drawn to the buffer on the screen
  display.display();
  delay(2000);
  // Clear the display buffer before drawing anything
  display.clearDisplay();
  while (!Serial) {
    ;
  }

  Serial.println("Initializing INA219 for Battery Monitoring...");
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    // while (1) {
    //   delay(10);
    // }
  }
  ina219.setCalibration_32V_2A();
  // ****Battery voltage calibration******
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
  delay(1000);  //
  Serial.println("INA219 initialized successfully!");

  // Init MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  } else {
    Serial.println("MPU6050 ready");
  }
  mpu.setFullScaleAccelRange(ACCEL_RANGE);   // 1 => ±4g
  mpu.setFullScaleGyroRange(GYRO_RANGE);     // 1 => ±500 dps
   // Bias calibration (keep vehicle still for a few seconds)
  Serial.println("[IMU] Calibrating... keep completely still");
  mpu.CalibrateAccel(6);   // 4–8 iterations are fine
  mpu.CalibrateGyro(6);
  Serial.println("[IMU] Calibration done.");
  // Seed angles from gravity to avoid startup jump
  float ax, ay, az, gx, gy, gz, ar, ap;
  readScaled(ax, ay, az, gx, gy, gz);
  accelAngles(ax, ay, az, ar, ap);
  roll = ar; pitch = ap;
  loadBaseline();
  lastMicros = micros();
  // Set motor controller pins as outputs
  pinMode(motorLeft_A, OUTPUT);   
  pinMode(motorLeft_B, OUTPUT);   
  pinMode(motorRight_A, OUTPUT);  
  pinMode(motorRight_B, OUTPUT);  
  pinMode(light, INPUT);
  light_value = analogRead(light);
  myservo.setPeriodHertz(50);            // standard 50 hz of servo
  myservo.attach(servoPin, 1000, 2000);  // attaches the servo on pin 16 to the servo object
  myservo.write(90);                     // setup servo motor to go to position (0-180 degree) in variable 'pos' (pos=90(Stop), 90 < Pos < 180 (turn right), Pos< 90 (Turn left))
  time_start = millis();
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
  if (Ps3.isConnected() && millis() - lastSend > 50) {  
    lastSend = millis();
    // if (Ps3.isConnected()) {
    // Serial.println("Connected!");
    // Ps3.setPlayer(player);  // Setup Player
    // battery = Ps3.data.status.battery;  // Measuring battery status
    // Serial.print("The controller battery is ");
    // if (battery == ps3_status_battery_charging) Serial.println("charging");
    // else if (battery == ps3_status_battery_full) Serial.println("FULL");
    // else if (battery == ps3_status_battery_high) Serial.println("HIGH");
    // else if (battery == ps3_status_battery_low) Serial.println("LOW");
    // else if (battery == ps3_status_battery_dying) Serial.println("DYING");
    // else if (battery == ps3_status_battery_shutdown) Serial.println("SHUTDOWN");
    // else Serial.println("UNDEFINED");
    //Reading IMU, Light and INA219 sensors---------------------
    busVoltage = ina219.getBusVoltage_V();  // Voltage (V) 
    current_mA = ina219.getCurrent_mA();    // Current (mA)
    power_mW   = ina219.getPower_mW();      // Power (mW)
    // Serial.print("Battery Voltage: ");
    // Serial.print(busVoltage);
    // Serial.println(" V");
    // Serial.print("Current:         ");
    // Serial.print(current_mA);
    // Serial.println(" mA");
    // Serial.print("Power:           ");
    // Serial.print(power_mW);
    // Serial.println(" mW");
    //  Calling feature every 1000 milisecond
    uint32_t now = millis();
    if ((uint32_t)(now - lastSlow) >= 1000) {
      lastSlow += 1000;
      oled_output();
    }
    // When joystick and ESP32 board bluetooth connect already, Progran is running
    // Pad UP, DOWN, LEFT, RIGHT
    if (Ps3.data.button.up) {  // Push Pad UP, motor move forward
      // Serial.println("forward");
      Forward_L(255);
      Forward_R(255);
      if (!Ps3.data.button.square || !Ps3.data.button.circle) {
        myservo.write(90);  // setup servo motor stop position in variable 'pos'
      }
    }
    if (Ps3.data.button.down) {  // Push Pad Down, motor move backward
      // Serial.println("backward");
      Backward_L(125);
      Backward_R(125);
      if (!Ps3.data.button.square || !Ps3.data.button.circle) {
        myservo.write(90);  // setup servo motor stop position in variable 'pos'
      }
    }
    if (Ps3.data.button.square) {  // Push Pad Left, motor turn left
      myservo.write(60);           // setup servo motor moving to position in variable 'pos' and turn lef 30 degree from 90 degree axis
    }
    if (Ps3.data.button.circle) {  // Push Pad Right, motor turn right
      myservo.write(120);          // setup servo motor moving to position in variable 'pos' and turn right 30 degree from 90 degree axis
    }
    if (Ps3.data.button.start) {  // Push Pad Start as AI mode
      Serial.println("AI mode");
      Forward_L(0);
      Forward_R(0);
    }
    if (Ps3.data.button.select) {  // Push Pad Select as Manual mode
      Serial.println("Manual mode");
      Forward_L(0);
      Forward_R(0);
    }
    if (!Ps3.data.button.square && !Ps3.data.button.circle && !Ps3.data.button.down && !Ps3.data.button.up) {
      // Serial.println("stop"); 
      myservo.write(90);  // setup servo motor stop position in variable 'pos'
      Stop_R(0);
      Stop_L(0);
    }
  }
}
