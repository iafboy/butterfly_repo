#include <Arduino.h>
#include <ESP32Servo.h>
#include <AlfredoCRSF.h> 
#include <Wire.h>
#include <MPU6050_light.h> 
#include <Adafruit_NeoPixel.h>

// --- 硬件引脚配置 ---
#define PIN_SERVO_L  3
#define PIN_SERVO_R  4
#define PIN_RX_ELRS  44
#define PIN_TX_ELRS  43
#define I2C_SDA      8
#define I2C_SCL      9
#define PIN_LED      1
#define PIN_BAT_ADC  10

// --- 参数配置 ---
#define SERVO_MID    1520
#define SERVO_HZ     100
#define NUM_LEDS     4
#define WING_UP_OFFSET 500  // 停机时竖起的角度偏置(us)，约45度

// --- 全局对象 ---
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);
Servo wingL, wingR;
AlfredoCRSF crsf;
HardwareSerial CRSFSerial(0);
MPU6050 mpu(Wire);

float phase = 0;
unsigned long last_update;
float pixel_hue = 0;

// --- 灯光控制逻辑 ---
void updateLEDs(bool lowBattery) {
  static unsigned long last_led_ms = 0;
  if (millis() - last_led_ms < 20) return; 
  last_led_ms = millis();

  if (lowBattery) {
    // 低电量红闪
    static bool flash = false;
    static int flash_cnt = 0;
    if (++flash_cnt > 10) { flash = !flash; flash_cnt = 0; }
    strip.fill(flash ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0));
  } else {
    // 正常模式：彩虹跑马灯
    pixel_hue += 300;
    if (pixel_hue >= 5 * 65536) pixel_hue = 0;
    for (int i = 0; i < strip.numPixels(); i++) {
      int hue = pixel_hue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue)));
    }
  }
  strip.show();
}

// =======================================================
void setup() {
  Serial.begin(115200);

  // 1. 灯光初始化
  strip.begin();
  strip.setBrightness(100);
  strip.show();

  // 2. MPU6050 强制初始化逻辑
  Wire.end(); // 先关闭可能存在的 I2C 实例
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL, 100000); // 显式在 begin 中指定频率
  delay(500);

  // 尝试多次初始化直到成功或跳过
  byte status;
  for(int i = 0; i < 5; i++) {
    status = mpu.begin();
    Serial.print(F("MPU6050 尝试次数 "));
    Serial.print(i+1);
    Serial.print(F(" - 状态: "));
    Serial.println(status);
    if(status == 0) break;
    delay(200);
  }
  
  // 即使失败也强制进行一次偏移计算，防止库内部变量未定义
  Serial.println(F("正在校准，请保持水平静止..."));
  mpu.calcOffsets(); 
  Serial.println(F("校准流程结束"));

  // 3. 接收机与舵机
  CRSFSerial.begin(420000, SERIAL_8N1, PIN_RX_ELRS, PIN_TX_ELRS);
  crsf.begin(CRSFSerial);

  ESP32PWM::allocateTimer(0);
  wingL.setPeriodHertz(SERVO_HZ);
  wingR.setPeriodHertz(SERVO_HZ);
  wingL.attach(PIN_SERVO_L, 500, 2500);
  wingR.attach(PIN_SERVO_R, 500, 2500);

  // 4. 初始姿态：竖起翅膀
  wingL.writeMicroseconds(SERVO_MID + WING_UP_OFFSET);
  wingR.writeMicroseconds(SERVO_MID - WING_UP_OFFSET);

  last_update = micros();
}

// =======================================================
void loop() {
  crsf.update();
  mpu.update();

  // --- 电池与灯光 ---
  float v = (analogRead(PIN_BAT_ADC) / 4095.0f) * 3.3f * 2.0f;
  updateLEDs(v > 1.0f && v < 3.5f);

  int thr = crsf.getChannel(3);
  int roll = crsf.getChannel(1);

  // ========== A. 怠速/最低油门：竖起翅膀 ==========
  if (thr < 1100) {
    wingL.writeMicroseconds(SERVO_MID + WING_UP_OFFSET);
    wingR.writeMicroseconds(SERVO_MID - WING_UP_OFFSET);
    phase = 0;
    last_update = micros();
    return;
  }

  // ========== B. 扑翼物理增量 ==========
  unsigned long now = micros();
  float dt = (now - last_update) * 1e-6;
  if (dt > 0.05) dt = 0.01;
  last_update = now;
  float low_feq=1.2f;
  float high_feq=10.0f;
  float t = constrain((float)(thr - 1100) / 900.0f, 0.0f, 1.0f);
  
  // 角度映射：120° (底油门防掉高) -> 65° (高油门)
  float total_amp_deg = 120.0f - (t * 55.0f); 
  float amp_us = (total_amp_deg * 0.5f) * (2000.0f / 180.0f);

  // 频率映射：0.8Hz -> 4.2Hz
  float freq = low_feq + t * high_feq;

  phase += freq * dt;
  if (phase >= 1.0f) phase -= 1.0f;

  // ========== C. 自稳控制与输出 ==========
  float roll_angle = mpu.getAngleX();
  float turn_sum = (roll - 1500) / 500.0f + (roll_angle / 30.0f);

  float wave = sinf(phase * 2.0f * PI);
  int pwmL = SERVO_MID + (int)(wave * amp_us * (1.0f + turn_sum));
  int pwmR = SERVO_MID - (int)(wave * amp_us * (1.0f - turn_sum));

  wingL.writeMicroseconds(constrain(pwmL, 500, 2500));
  wingR.writeMicroseconds(constrain(pwmR, 500, 2500));
}
