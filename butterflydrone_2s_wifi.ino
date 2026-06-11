#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

// ==================== 引脚配置（适配 S3 Super Mini） ====================
#define SDA_PIN            8      // I2C SDA
#define SCL_PIN            9      // I2C SCL
#define LEFT_SERVO_PIN     4
#define RIGHT_SERVO_PIN    5
#define BATTERY_PIN        1      // ADC 电压检测（GPIO1）

// ==================== 舵机参数 ====================
#define SERVO_MID          1520
#define SERVO_MIN          500
#define SERVO_MAX          2500
#define SERVO_HZ           50

// ==================== 控制参数 ====================
const float ROLL_STAB_GAIN   = 30.0f;     // 姿态稳定增益
const float ROLL_D_GAIN      = 18.0f;
const float ROLL_STICK_GAIN  = 1.0f;
const float ROLL_DIFF_LIMIT  = 0.32f;
const int   ROLL_TRIM        = 0;

// ==================== 电池参数 ====================
const float VOLTAGE_DIVIDER  = 3.40f;     // 校正电压分压比
const float LOW_VOLTAGE      = 3.60f;
const float CRITICAL_VOLTAGE = 3.45f;

const char* AP_SSID = "ButterflyDrone";
const char* AP_PASS = "12345678";

WebServer server(80);
Adafruit_MPU6050 mpu; 

enum FlightMode { MODE_STOP, MODE_FLY };

struct ControlState {
  FlightMode mode = MODE_STOP;
  float flapPhase = 0.0f;
  float throttle = 0.03f; 
  int   maxAmplitudeDeg = 120;
  float rightPhaseOffsetDeg = 0.0f; 
  int   trimBias = 0;           // 手动横滚配平补偿（范围 -15 到 15）
  float lastRoll = 0.0f;
  float lastRollError = 0.0f;
  int   lastLeftUs = SERVO_MID;
  int   lastRightUs = SERVO_MID;
  bool  running = false;
  unsigned long lastFlapTime = 0;
  unsigned long lastImuTime = 0;  // 融合滤波时间戳
  float batteryVoltage = 0.0f;
  bool  lowVoltageWarning = false;
} state;

bool mpuInitialized = false; // 记录6050是否正常工作
bool reverseLeft = false;
bool reverseRight = false;

// ====================== 电压读取 ======================
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  return (raw / 4095.0f) * 3.3f * VOLTAGE_DIVIDER;
}

// ====================== 控制页面 ======================
String htmlPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>扑翼蝴蝶控制器</title>
<style>
body { font-family: Arial, "Microsoft YaHei", sans-serif; margin:18px; background:#101418; color:#f2f2f2; }
.card { background:#1c232b; padding:16px; margin:12px 0; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,.25); }
button { font-size:16px; padding:10px 16px; margin:5px; border:0; border-radius:8px; cursor:pointer; }
.on { background:#37c871; color:#07140b; font-weight:bold; }
.off { background:#ff6767; color:#210303; font-weight:bold; }
.btn-trim { background:#5596ff; color:white; }
.value { color:#78dce8; font-weight:bold; }
.warning { color:#ffaa00; animation: blink 1s infinite; }
.critical { color:#ff4444; animation: blink 0.6s infinite; }
input[type=range] { width: 100%; margin: 10px 0 20px 0; }
.label-group { display: flex; justify-content: space-between; }
@keyframes blink { 50% { opacity: 0.4; } }
</style>
</head>
<body>
<h2>扑翼蝴蝶 WiFi 控制台</h2>
<div class="card">
  <p>运行状态：<span id="mode" class="value">-</span></p>
  <p>俯仰角：<span id="pitch" class="value">-</span>° 横滚角：<span id="roll" class="value">-</span>°</p>
  <p>左翼PWM：<span id="left" class="value">-</span> 右翼PWM：<span id="right" class="value">-</span></p>
  <p>当前速度：<span id="throttle" class="value">-</span> 横滚配平：<span id="trim" class="value">0</span></p>
  <p>电池电压：<span id="voltage" class="value">-</span>V <span id="vstatus"></span></p>
</div>

<div class="card" style="text-align:center;">
  <button class="on" onclick="cmd('/start')">启 动</button>
  <button class="off" onclick="cmd('/stop')">停 止</button>
</div>

<div class="card">
  <h3>● 姿态手动机械配平 (微调左右倾)</h3>
  <button class="btn-trim" onclick="nudge('left')">◀ 微调左倾</button>
  <button class="btn-trim" onclick="nudge('right')">微调右倾 ▶</button>
  <button class="btn-trim" onclick="nudge('center')">配平回中</button>
</div>

<div class="card">
  <div class="label-group">
    <label><b>扑翼角度上限：</b><span id="ampv" class="value">120</span>°</label>
  </div>
  <input id="amp" type="range" min="60" max="130" value="120" oninput="ampv.innerText=this.value" onchange="apply()">
  
  <div class="label-group">
    <label><b>扑翼速度：</b><span id="speedv" class="value">0.03</span></label>
  </div>
  <input id="speed" type="range" min="0" max="6" value="3" oninput="speedv.innerText=(this.value/100).toFixed(2)" onchange="apply()">
</div>

<script>
function cmd(path){ fetch(path).then(update); }
function nudge(a){ fetch('/nudge?action='+a).then(update); }
function apply(){
  fetch('/set?amp='+document.getElementById('amp').value+
        '&speed='+(document.getElementById('speed').value/100).toFixed(2)).then(update);
}
function update(){
  fetch('/status').then(r=>r.json()).then(s=>{
    document.getElementById('mode').innerText = s.running ? "运行中" : "已停止";
    document.getElementById('pitch').innerText = s.pitch.toFixed(1);
    document.getElementById('roll').innerText = s.roll.toFixed(1);
    document.getElementById('left').innerText = s.left;
    document.getElementById('right').innerText = s.right;
    document.getElementById('throttle').innerText = s.throttle.toFixed(2);
    document.getElementById('trim').innerText = s.trim;

    let v = s.voltage.toFixed(2);
    let vspan = document.getElementById('voltage');
    vspan.innerText = v;
    let status = document.getElementById('vstatus');
    if(s.critical){
      vspan.className = "critical";
      status.innerHTML = " <b>严重低压！</b>";
    } else if(s.lowWarning){
      vspan.className = "warning";
      status.innerHTML = " <b>低压警告</b>";
    } else {
      vspan.className = "value";
      status.innerHTML = "";
    }
  });
}
setInterval(update, 500);
update();
</script>
</body>
</html>
)rawliteral";
}

float multiStageMap(float t, const float s[], const float v[], int n) {
  for(int i=0; i<n-1; i++) if(t >= s[i] && t <= s[i+1])
    return v[i] + (v[i+1]-v[i])*(t-s[i])/(s[i+1]-s[i]);
  return v[n-1];
}

// ★ 兼容新版 ESP32 Core 3.x 及以上版本的舵机直接输出函数
void writeServos(int l, int r) {
  l = constrain(l, SERVO_MIN, SERVO_MAX);
  r = constrain(r, SERVO_MIN, SERVO_MAX);
  if(reverseLeft) l = SERVO_MIN + SERVO_MAX - l;
  if(reverseRight) r = SERVO_MIN + SERVO_MAX - r;
  state.lastLeftUs = l; state.lastRightUs = r;
  
  // 计算占空比：频率 50Hz (周期 20000us)，16位精度最大值 65535
  uint32_t dutyL = (uint32_t)((l / 20000.0f) * 65535.0f);
  uint32_t dutyR = (uint32_t)((r / 20000.0f) * 65535.0f);

  ledcWrite(LEFT_SERVO_PIN, dutyL);
  ledcWrite(RIGHT_SERVO_PIN, dutyR);
}

// ====================== 控制任务（带安全容错机制） ======================
void controlTask(void *pv) {
  const float stages[] = {0.0, 0.25, 0.5, 0.8, 1.0};
  const float freqs[]  = {1.2, 2.8, 4.8, 5.8, 9.0};
  const float amps[]   = {120, 100, 90, 70, 58};

  while(true) {
    float roll = 0.0f;
    float pitch = 0.0f;
    
    if (mpuInitialized) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      unsigned long now_us = micros();
      float dt_imu = (now_us - state.lastImuTime) * 1e-6f;
      state.lastImuTime = now_us;
      if (dt_imu <= 0 || dt_imu > 0.1f) dt_imu = 0.01f;

      float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
      float accAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
      
      float gyroRateX = g.gyro.x * 180.0 / PI;
      float gyroRateY = g.gyro.y * 180.0 / PI;
      
      roll = 0.96f * (state.lastRoll + gyroRateX * dt_imu) + 0.04f * accAngleX;
      pitch = 0.96f * (0 + gyroRateY * dt_imu) + 0.04f * accAngleY;
    }

    state.batteryVoltage = readBatteryVoltage();
    state.lowVoltageWarning = state.batteryVoltage < LOW_VOLTAGE;
    bool critical = state.batteryVoltage < CRITICAL_VOLTAGE;

    float dt = (micros() - state.lastFlapTime) * 1e-6f;
    state.lastFlapTime = micros();
    if(dt <= 0 || dt > 0.1f) dt = 0.01f;

    float effectiveThrottle = state.throttle;
    if(critical) effectiveThrottle = min(effectiveThrottle, 0.02f);
    else if(state.lowVoltageWarning) effectiveThrottle = min(effectiveThrottle, 0.04f);

    if(!state.running || effectiveThrottle < 0.001f) {
      writeServos(SERVO_MID, SERVO_MID); 
      state.flapPhase = 0;
      state.lastRoll = roll;
      vTaskDelay(12);
      continue;
    }

    float t = constrain(effectiveThrottle / 0.06f, 0.0f, 1.0f);
    float freq = multiStageMap(t, stages, freqs, 5);
    float ampDeg = min(multiStageMap(t, stages, amps, 5), (float)state.maxAmplitudeDeg);
    float ampUs = (ampDeg * 0.5f) * (2000.0f / 180.0f);

    state.flapPhase += freq * dt;
    if(state.flapPhase >= 1.0f) state.flapPhase -= 1.0f;

    float P = 0.0f, D = 0.0f;
    if (mpuInitialized) {
      P = roll * ROLL_STAB_GAIN;
      D = (roll - state.lastRollError) * ROLL_D_GAIN;
    }
    
    float trimManual = state.trimBias / 15.0f; 
    float total = trimManual * ROLL_STICK_GAIN + (P + D) / 100.0f;

    float diff = constrain(total, -ROLL_DIFF_LIMIT, ROLL_DIFF_LIMIT);
    state.lastRollError = roll;
    state.lastRoll = roll; 

    float waveL = sinf(state.flapPhase * TWO_PI);
    float waveR = sinf(state.flapPhase * TWO_PI + radians(state.rightPhaseOffsetDeg));

    int pwmL = SERVO_MID + ROLL_TRIM + (int)(waveL * ampUs * (1.0f + diff));
    int pwmR = SERVO_MID - ROLL_TRIM - (int)(waveR * ampUs * (1.0f - diff));

    writeServos(pwmL, pwmR);

    vTaskDelay(12); 
  }
}

// ====================== WebServer ======================
void setupServer() {
  server.on("/", [](){ server.send(200, "text/html", htmlPage()); });

  server.on("/status", [](){
    StaticJsonDocument<512> doc;
    doc["running"] = state.running;
    doc["pitch"] = 0; 
    doc["roll"] = state.lastRoll;
    doc["left"] = state.lastLeftUs;
    doc["right"] = state.lastRightUs;
    doc["throttle"] = state.throttle;
    doc["trim"] = state.trimBias;
    doc["voltage"] = state.batteryVoltage;
    doc["lowWarning"] = state.lowVoltageWarning;
    doc["critical"] = (state.batteryVoltage < CRITICAL_VOLTAGE);
    String json; serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  server.on("/start", [](){ state.running = true; server.send(200,"text/plain", "OK"); });
  server.on("/stop", [](){ state.running = false; server.send(200,"text/plain", "OK"); });

  server.on("/set", [](){
    if(server.hasArg("amp")) state.maxAmplitudeDeg = constrain(server.arg("amp").toInt(), 60, 130);
    if(server.hasArg("speed")) state.throttle = constrain(server.arg("speed").toFloat(), 0.0f, 0.06f);
    server.send(200, "text/plain", "OK");
  });

  server.on("/nudge", [](){
    String a = server.arg("action");
    if(a=="left") state.trimBias = constrain(state.trimBias-1, -15, 15);
    else if(a=="right") state.trimBias = constrain(state.trimBias+1, -15, 15);
    else if(a=="center") state.trimBias = 0;
    server.send(200,"text/plain", "OK");
  });

  server.on("/reverse", [](){
    if(server.hasArg("left")) reverseLeft = server.arg("left").toInt() == 1;
    if(server.hasArg("right")) reverseRight = server.arg("right").toInt() == 1;
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

// ====================== Setup ======================
void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n正在启动系统...");
  pinMode(BATTERY_PIN, INPUT);

  Wire.end();
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN, 100000); 
  delay(500);

  bool mpu_ok = false;
  for(int i = 0; i < 5; i++) {
    if(mpu.begin(0x68, &Wire)) { mpu_ok = true; break; } 
    delay(200);
  }

  if(mpu_ok) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuInitialized = true;
    Serial.println("MPU6050 初始化成功");
  } else {
    mpuInitialized = false;
    Serial.println("！未检测到 MPU6050，已忽略传感器，继续启动系统...");
  }

  // ★ 修复拼写：使用正确的 ledcAttach 绑定舵机引脚至 LEDC 输出（50Hz 频率，16位精度）
  ledcAttach(LEFT_SERVO_PIN, 50, 16);
  ledcAttach(RIGHT_SERVO_PIN, 50, 16);

  writeServos(SERVO_MID, SERVO_MID);

  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASS);
  
  Serial.print("WiFi 热点广播状态: ");
  if (apStarted) {
    Serial.println("启动成功！");
    Serial.print("SSID: "); Serial.println(AP_SSID);
    Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("启动失败！");
  }

  setupServer();

  xTaskCreatePinnedToCore(controlTask, "FlapCtrl", 8192, NULL, 3, NULL, 1);
  Serial.println("控制任务创建完成，系统就绪。");
}

void loop() {
  server.handleClient();
  vTaskDelay(5);
}
