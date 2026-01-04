#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AlfredoCRSF.h>
#include <Adafruit_NeoPixel.h>

// ======================================================
// ============= 第一部分：核心调试参数 =====================
// ======================================================

const float ROLL_STAB_GAIN = 30.0f;   // 姿态稳定增益
const float ROLL_STICK_GAIN = 1.0f;   // 遥控横滚杆灵敏度
const float ROLL_DIFF_LIMIT = 0.30f;  // 最大±30%扑翼差动补偿

const int ROLL_TRIM = 0;
const int WING_UP_OFFSET = 600;       // 翅膀初始竖起量

// ======================================================
// ============= 第二部分：硬件定义与全局变量 =============
// ======================================================

#define PIN_SERVO_L  3
#define PIN_SERVO_R  4
#define PIN_RX_ELRS  44
#define PIN_TX_ELRS  43
#define PIN_LED      1
#define PIN_BAT_ADC  10
#define I2C_SDA      8
#define I2C_SCL      9

#define SERVO_MID    1520
#define SERVO_MIN    500
#define SERVO_MAX    2500
#define SERVO_HZ     100
#define NUM_LEDS     4

Servo wingL, wingR;
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
AlfredoCRSF crsf;
HardwareSerial CRSFSerial(0);

float phase = 0;
unsigned long last_update, last_imu;
float angleX = 0, pixel_hue = 0;

// 自定义分段插值函数
float multiStageMap(float t, float stages[], float values[], int size) {
  for (int i = 0; i < size - 1; i++) {
    if (t >= stages[i] && t <= stages[i+1]) {
      return values[i] + (values[i+1] - values[i]) * (t - stages[i]) / (stages[i+1] - stages[i]);
    }
  }
  return values[size-1];
}

void updateLEDs(bool lowBattery) {
  static unsigned long last_led_ms = 0;
  if (millis() - last_led_ms < 20) return;
  last_led_ms = millis();
  if (lowBattery) {
    static bool flash = false;
    static int f_cnt = 0;
    if(++f_cnt > 10) { flash = !flash; f_cnt = 0; }
    strip.fill(flash ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0));
  } else {
    pixel_hue += 300;
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixel_hue + (i * 65536L / NUM_LEDS))));
    }
  }
  strip.show();
}

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(100);
  strip.show();

  Wire.end();
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  delay(500);

  bool mpu_ok = false;
  for(int i = 0; i < 5; i++) {
    if(mpu.begin()) { mpu_ok = true; break; }
    delay(200);
  }

  if(mpu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  CRSFSerial.begin(420000, SERIAL_8N1, PIN_RX_ELRS, PIN_TX_ELRS);
  crsf.begin(CRSFSerial);

  ESP32PWM::allocateTimer(0);
  wingL.setPeriodHertz(SERVO_HZ);
  wingR.setPeriodHertz(SERVO_HZ);
  wingL.attach(PIN_SERVO_L, SERVO_MIN, SERVO_MAX);
  wingR.attach(PIN_SERVO_R, SERVO_MIN, SERVO_MAX);

  // 初始竖起（已修正方向）
  wingL.writeMicroseconds(SERVO_MID + WING_UP_OFFSET);
  wingR.writeMicroseconds(SERVO_MID - WING_UP_OFFSET);

  last_update = micros();
  last_imu = micros();
}

void loop() {
  crsf.update();

  float v = (analogRead(PIN_BAT_ADC) / 4095.0f) * 3.3f * 2.0f;
  updateLEDs(v > 1.0f && v < 3.5f);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long now = micros();
  float dt_imu = (now - last_imu) * 1e-6;
  last_imu = now;

  float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float gyroRateX = g.gyro.x * 180.0 / PI;
  angleX = 0.96 * (angleX + gyroRateX * dt_imu) + 0.04 * accAngleX;

  int thr = crsf.getChannel(3);
  int roll = crsf.getChannel(1);

  // ====== A. 怠速：翅膀竖起 ======
  if (thr < 1100) {
    wingL.writeMicroseconds(SERVO_MID + WING_UP_OFFSET + ROLL_TRIM);
    wingR.writeMicroseconds(SERVO_MID - WING_UP_OFFSET - ROLL_TRIM);
    phase = 0;
    last_update = micros();
    return;
  }

  // ====== B. 扑翼动力学 ======
  float dt = (now - last_update) * 1e-6;
  if (dt > 0.05) dt = 0.01;
  last_update = now;

  float t = constrain((float)(thr - 1100) / 900.0f, 0.0f, 1.0f);

  float stages[] = {0.0f, 0.25f, 0.5f, 0.8f, 1.0f};

  float freq_values[] = {1.2f, 2.8f, 4.8f, 5.8f, 9.0f};
  float freq = multiStageMap(t, stages, freq_values, 5);

  float amp_values[] = {120.0f, 100.0f, 90.0f, 70.0f, 58.0f};
  float total_amp_deg = multiStageMap(t, stages, amp_values, 5);
  float amp_us = (total_amp_deg * 0.5f) * (2000.0f / 180.0f);

  phase += freq * dt;
  if (phase >= 1.0f) phase -= 1.0f;

  // ====== C. 计算差动补偿 ======
  float stick_roll = (roll - 1500) / 500.0f;     // -1 ~ +1
  float diff = stick_roll * ROLL_STICK_GAIN + (angleX / ROLL_STAB_GAIN);

  // 限制在 ±30%
  diff = constrain(diff, -ROLL_DIFF_LIMIT, ROLL_DIFF_LIMIT);

  // 翅膀正弦波
  float wave = sinf(phase * 2.0f * PI);

  // ====== D. 输出 ======
  int pwmL = SERVO_MID + ROLL_TRIM + (int)(wave * amp_us * (1.0f + diff));
  int pwmR = SERVO_MID - ROLL_TRIM - (int)(wave * amp_us * (1.0f - diff));

  wingL.writeMicroseconds(constrain(pwmL, SERVO_MIN, SERVO_MAX));
  wingR.writeMicroseconds(constrain(pwmR, SERVO_MIN, SERVO_MAX));

  static unsigned long lp = 0;
  if (millis() - lp > 500) {
    Serial.printf("Freq: %.1f Hz | Amp: %.1f deg | diff=%.2f | roll=%d | angX=%.1f\n",
                  freq, total_amp_deg, diff, roll, angleX);
    lp = millis();
  }
}
