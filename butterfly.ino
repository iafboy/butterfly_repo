#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AlfredoCRSF.h>
#include <Adafruit_NeoPixel.h>

// ========= 硬件引脚 =========
#define PIN_SERVO_L  3
#define PIN_SERVO_R  4
#define PIN_RX_ELRS  44
#define PIN_TX_ELRS  43
#define PIN_LED      1
#define PIN_BAT_ADC  10
#define I2C_SDA      8
#define I2C_SCL      9

// ========= 舵机配置 =========
#define SERVO_MID 1520
#define SERVO_MIN 500
#define SERVO_MAX 2500
#define SERVO_HZ  100  // PTK 7465 建议 100Hz 刷新率

// ========= 翅膀竖起偏置 (修正方向) =========
// 如果翅膀仍然向下，请将下面的 - 改为 +，+ 改为 -
#define WING_UP_OFFSET 600 

// ========= 映射参数 =========
#define CH_THR  3
#define CH_ROLL 1
#define NUM_LEDS 4
const float LOW_BAT_THRESHOLD = 3.5;

// ========= 全局对象 =========
Servo wingL, wingR;
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
AlfredoCRSF crsf;
HardwareSerial CRSFSerial(0);

float phase = 0;
unsigned long last_update;
float pixel_hue = 0;
float angleX = 0;
unsigned long last_imu = 0;

// ======================================================
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
    for (int i = 0; i < strip.numPixels(); i++) {
      int hue = pixel_hue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue)));
    }
  }
  strip.show();
}

// ======================================================
void setup() {
  Serial.begin(115200);

  // 1. 灯带初始化
  strip.begin();
  strip.setBrightness(100);
  strip.show();

  // 2. MPU6050 强制鲁棒初始化逻辑 (参考用户要求)
  Wire.end(); 
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL, 100000); 
  delay(500);

  bool status = false;
  for(int i = 0; i < 5; i++) {
    status = mpu.begin();
    Serial.print(F("MPU6050 尝试次数 "));
    Serial.print(i+1);
    Serial.print(F(" - 状态: "));
    Serial.println(status ? "成功" : "失败");
    if(status) break;
    delay(200);
  }

  if(status) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // 3. CRSF 接收机
  CRSFSerial.begin(420000, SERIAL_8N1, PIN_RX_ELRS, PIN_TX_ELRS);
  crsf.begin(CRSFSerial);

  // 4. 舵机配置
  ESP32PWM::allocateTimer(0);
  wingL.setPeriodHertz(SERVO_HZ);
  wingR.setPeriodHertz(SERVO_HZ);
  wingL.attach(PIN_SERVO_L, SERVO_MIN, SERVO_MAX);
  wingR.attach(PIN_SERVO_R, SERVO_MIN, SERVO_MAX);

  // 5. 初始状态：翅膀向上竖起 (修正符号)
  wingL.writeMicroseconds(SERVO_MID - WING_UP_OFFSET);
  wingR.writeMicroseconds(SERVO_MID + WING_UP_OFFSET);

  last_update = micros();
  last_imu = micros();
}

// ======================================================
void loop() {
  crsf.update();

  // --- 电池检测 ---
  float v = (analogRead(PIN_BAT_ADC) / 4095.0f) * 3.3f * 2.0f;
  updateLEDs(v > 1.0f && v < LOW_BAT_THRESHOLD);

  // --- 姿态计算 (互补滤波) ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long now = micros();
  float dt_imu = (now - last_imu) * 1e-6;
  last_imu = now;

  float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float gyroRateX = g.gyro.x * 180.0 / PI;
  angleX = 0.96 * (angleX + gyroRateX * dt_imu) + 0.04 * accAngleX;

  // --- 通道获取 ---
  int thr = crsf.getChannel(CH_THR);
  int roll = crsf.getChannel(CH_ROLL);

  // ========== A. 怠速状态：翅膀竖起 (修正符号) ==========
  if (thr < 1100) {
    wingL.writeMicroseconds(SERVO_MID - WING_UP_OFFSET);
    wingR.writeMicroseconds(SERVO_MID + WING_UP_OFFSET);
    phase = 0;
    last_update = micros();
    return;
  }

  // ========== B. 扑翼运动增量 ==========
  float dt = (now - last_update) * 1e-6;
  if (dt > 0.05) dt = 0.01;
  last_update = now;

  float t = constrain((float)(thr - 1100) / 900.0f, 0.0f, 1.0f);

  // 摆角映射：低油门 100° (防掉高) -> 高油门 65°
  float total_amp_deg = 100.0f - t * 55.0f;
  float amp_us = (total_amp_deg * 0.5f) * (2000.0f / 180.0f);

  // 频率映射：1.2Hz -> 10.0Hz
  float lowFreq=1.2f;
  float highFreq=10.0f;
  float freq = lowFreq + t * highFreq;

  phase += freq * dt;
  if (phase >= 1.0f) phase -= 1.0f;

  // ========== C. 混合控制 (摇杆 + 自稳) ==========
  // turn_sum：正数向左倾，负数向右倾
  float turn_sum = (roll - 1500) / 500.0f + (angleX / 30.0f);

  // ========== D. 输出波形 ==========
  float wave = sinf(phase * 2.0f * PI);

  int pwmL = SERVO_MID + (int)(wave * amp_us * (1.0f + turn_sum));
  int pwmR = SERVO_MID - (int)(wave * amp_us * (1.0f - turn_sum));

  wingL.writeMicroseconds(constrain(pwmL, SERVO_MIN, SERVO_MAX));
  wingR.writeMicroseconds(constrain(pwmR, SERVO_MIN, SERVO_MAX));

  // --- 调试打印 ---
  static unsigned long lp = 0;
  if (millis() - lp > 500) {
    Serial.printf("Bat:%.2fV | Roll:%.1f | Amp:%.1f | Freq:%.1f\n", v, angleX, total_amp_deg, freq);
    lp = millis();
  }
}
