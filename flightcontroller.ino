
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SDA_PIN 13
#define SCL_PIN 14

#define MPU_ADDR 0x68
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1
);

float roll = 0, pitch = 0, yaw = 0;
float gyroZBias = 0;
unsigned long lastTime;

bool writeMPU(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)MPU_ADDR,
                       (size_t)14, true) != 14) return false;

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());

  Wire.read(); Wire.read(); // Temperature

  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());

  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  delay(100);

  if (!writeMPU(0x6B, 0x00)) {
    Serial.println("ERROR: MPU6050 not found");
    while (true) delay(100);
  }

  writeMPU(0x1B, 0x00); // Gyro +/-250 degrees/sec
  writeMPU(0x1C, 0x00); // Accelerometer +/-2g

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("ERROR: OLED not found at 0x3C");
    while (true) delay(100);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Keep IMU still...");
  display.println("Calibrating gyro");
  display.display();

  // Estimate gyro Z bias. Keep sensor completely still.
  long sumZ = 0;
  int16_t ax, ay, az, gx, gy, gz;
  int samples = 0;

  for (int i = 0; i < 500; i++) {
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sumZ += gz;
      samples++;
    }
    delay(4);
  }

  if (samples > 0) gyroZBias = (float)sumZ / samples;

  // Initial roll/pitch from accelerometer
  if (readMPU(ax, ay, az, gx, gy, gz)) {
    roll = atan2((float)ay, (float)az) * 180.0 / PI;
    pitch = atan2(-(float)ax,
             sqrt((float)ay * ay + (float)az * az))
             * 180.0 / PI;
  }

  yaw = 0;
  lastTime = micros();

  Serial.println("roll,pitch,yaw");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  if (!readMPU(ax, ay, az, gx, gy, gz)) {
    delay(20);
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  if (dt <= 0 || dt > 0.2) return;

  // Accelerometer orientation
  float accRoll = atan2((float)ay, (float)az)
                  * 180.0 / PI;

  float accPitch = atan2(-(float)ax,
                    sqrt((float)ay * ay + (float)az * az))
                    * 180.0 / PI;

  // Gyroscope scale: 131 raw units per degree/second
  float gyroRollRate = gx / 131.0;
  float gyroPitchRate = gy / 131.0;
  float gyroYawRate = (gz - gyroZBias) / 131.0;

  // Complementary filter: gyro + accelerometer
  roll = 0.98 * (roll + gyroRollRate * dt)
       + 0.02 * accRoll;

  pitch = 0.98 * (pitch + gyroPitchRate * dt)
        + 0.02 * accPitch;

  // Yaw is gyro-integrated and will drift over time
  yaw += gyroYawRate * dt;

  // CSV data for browser: one line per reading
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.println(yaw, 2);

  // OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("DRONE ORIENTATION");
  display.println("----------------");

  display.print("Roll : ");
  display.println(roll, 1);

  display.print("Pitch: ");
  display.println(pitch, 1);

  display.print("Yaw  : ");
  display.println(yaw, 1);

  display.display();

  delay(20);
}
