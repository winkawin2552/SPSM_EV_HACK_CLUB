#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

MPU6050 mpu;
int light_sensor = 2;


void setup() {
  Serial.begin(115200);
  
  // Initialize I2C on SDA=5, SCL=4
  Wire.begin(5, 4);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("Initializing INA219...");
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip!");
    while (1) { delay(10); }
  }
  Serial.println("INA219 connection successful!");
}


void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int light_val = analogRead(light_sensor);

  float busVoltage = ina219.getBusVoltage_V();  // Voltage (V)
  float current_mA = ina219.getCurrent_mA();    // Current (mA)
  float power_mW = ina219.getPower_mW();        // Power (mW)

  // Read raw values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to acceleration in g and gyro in deg/s (approx)
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  Serial.print("Light (LX): ");
  Serial.print(light_val); Serial.print(", ");
  Serial.print("Accel (g): ");
  Serial.print(accelX); Serial.print(", ");
  Serial.print(accelY); Serial.print(", ");
  Serial.print(accelZ);
  Serial.print(" | Gyro (°/s): ");
  Serial.print(gyroX); Serial.print(", ");
  Serial.print(gyroY); Serial.print(", ");
  Serial.print(gyroZ);
  Serial.print(" | busVoltage: ");
  Serial.print(busVoltage);
  Serial.print(" V, current_mA: ");
  Serial.print(current_mA);
  Serial.print(" mA, power_mW: ");
  Serial.print(power_mW);
  Serial.println(" mW");

}
