#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AlfredoCRSF.h>
#include <Adafruit_NeoPixel.h>

// ======================================================
// ============= 第一部分：核心调试参数 (最常修改) =============
// ======================================================

// 1. 自稳力度 (分母值)。数值越小，自稳越强；数值越大，自稳越弱。
// 建议范围：20.0 (强) ~ 50.0 (弱)。如果飞机在空中左右抖动，调大它。
const float STABILIZE_GAIN = 30.0f; 

// 2. 操控灵敏度。数值越大，你拨动摇杆时飞机转向越快。
const float STICK_SENSITIVITY = 1.0f; 

// 3. 左右平衡修正。正数往左修，负数往右修。单位：us。
const int ROLL_TRIM = 0; 

// 4. 摆角范围 (度)。低油门时大摆角提供升力，高油门时减小摆角换取推力。
const float AMP_MAX = 100.0f; // 低油门起始摆角
const float AMP_MIN = 60.0f;  // 高油门最高频率时的摆角

// 5. 频率范围 (Hz)。扑翼频率。
const float FREQ_LOW  = 1.2f; 
const float FREQ_HIGH = 10.0f; // PTK 7465 建议最高不要超过 5Hz

// 6. 竖起翅膀的角度。数值越大竖得越高。
const int WING_UP_OFFSET = 600; 

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

// ======================================================
// ============= 第三部分：初始化逻辑 =====================
// ======================================================

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(100);
  strip.show();

  // MPU6050 强制鲁棒初始化
  Wire.end(); 
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL, 100000); 
  delay(500);

  bool mpu_ok = false;
  for(int i = 0; i < 5; i++) {
    if(mpu.begin()) { mpu_ok = true; break; }
    Serial.printf("MPU6050 尝试 %d 失败\n", i+1);
    delay(200);
  }

  if(mpu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 初始化成功");
  }

  CRSFSerial.begin(420000, SERIAL_8N1, PIN_RX_ELRS, PIN_TX_ELRS);
  crsf.begin(CRSFSerial);

  ESP32PWM::allocateTimer(0);
  wingL.setPeriodHertz(SERVO_HZ);
  wingR.setPeriodHertz(SERVO_HZ);
  wingL.attach(PIN_SERVO_L, SERVO_MIN, SERVO_MAX);
  wingR.attach(PIN_SERVO_R, SERVO_MIN, SERVO_MAX);

  // 初始姿态：翅膀向上竖起
  wingL.writeMicroseconds(SERVO_MID - WING_UP_OFFSET);
  wingR.writeMicroseconds(SERVO_MID + WING_UP_OFFSET);

  last_update = micros();
  last_imu = micros();
}

// ======================================================
// ============= 第四部分：主循环控制 =====================
// ======================================================

void loop() {
  crsf.update();

  // 1. 电池与姿态计算 (互补滤波)
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

  // 2. 遥控数据
  int thr = crsf.getChannel(3);
  int roll = crsf.getChannel(1);

  // ========== A. 怠速状态：竖起翅膀 ==========
  if (thr < 1100) {
    wingL.writeMicroseconds(SERVO_MID - WING_UP_OFFSET + ROLL_TRIM);
    wingR.writeMicroseconds(SERVO_MID + WING_UP_OFFSET - ROLL_TRIM);
    phase = 0;
    last_update = micros();
    return;
  }

  // ========== B. 扑翼动力学计算 ==========
  float dt = (now - last_update) * 1e-6;
  if (dt > 0.05) dt = 0.01;
  last_update = now;

  float t = constrain((float)(thr - 1100) / 900.0f, 0.0f, 1.0f);
  
  // 使用常量映射：大角度到小角度
  float total_amp_deg = AMP_MAX - t * (AMP_MAX - AMP_MIN);
  float amp_us = (total_amp_deg * 0.5f) * (2000.0f / 180.0f);

  // 使用常量映射：低频率到高频率
  float freq = FREQ_LOW + t * (FREQ_HIGH - FREQ_LOW);

  phase += freq * dt;
  if (phase >= 1.0f) phase -= 1.0f;

  // ========== C. 混合控制逻辑 (引入调试常量) ==========
  // turn_sum 由两部分组成：摇杆输入 (Scaling) + 自动补偿 (Gain)
  float turn_sum = ((roll - 1500) / 500.0f) * STICK_SENSITIVITY + (angleX / STABILIZE_GAIN);

  // ========== D. 最终输出 ==========
  float wave = sinf(phase * 2.0f * PI);

  int pwmL = SERVO_MID + ROLL_TRIM + (int)(wave * amp_us * (1.0f + turn_sum));
  int pwmR = SERVO_MID - ROLL_TRIM - (int)(wave * amp_us * (1.0f - turn_sum));

  wingL.writeMicroseconds(constrain(pwmL, SERVO_MIN, SERVO_MAX));
  wingR.writeMicroseconds(constrain(pwmR, SERVO_MIN, SERVO_MAX));

  // 每 500ms 打印一次关键数据，辅助调试
  static unsigned long lp = 0;
  if (now/1000 - lp > 500) {
    Serial.printf("Roll:%.1f | T_Sum:%.2f | Amp:%.1f | Freq:%.1f\n", angleX, turn_sum, total_amp_deg, freq);
    lp = now/1000;
  }
}
