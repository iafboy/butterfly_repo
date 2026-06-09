#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <ESP32Servo.h>
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
const float ROLL_STAB_GAIN   = 22.0f;
const float ROLL_D_GAIN      = 18.0f;
const float ROLL_STICK_GAIN  = 1.0f;
const float ROLL_DIFF_LIMIT  = 0.32f;
const int   WING_UP_OFFSET    = 620;
const int   ROLL_TRIM        = 0;

// ==================== 电池参数 ====================
const float VOLTAGE_DIVIDER = 3.40f;     // 校正电压分压比
const float LOW_VOLTAGE      = 3.60f;
const float CRITICAL_VOLTAGE = 3.45f;

const char* AP_SSID = "ButterflyDrone";
const char* AP_PASS = "12345678";

WebServer server(80);
MPU6050 mpu(Wire);
Servo leftServo, rightServo;

enum FlightMode { MODE_STOP, MODE_TAKEOFF, MODE_CRUISE, MODE_CORRECT, MODE_LAND };

struct ControlState {
  FlightMode mode = MODE_STOP;
  float flapPhase = 0.0f;
  float throttle = 0.03f; // 默认给中间值
  int   maxAmplitudeDeg = 120;
  float rightPhaseOffsetDeg = 0.0f; 
  int   steerBias = 0;
  float lastRoll = 0.0f;
  float lastRollError = 0.0f;
  int   lastLeftUs = SERVO_MID;
  int   lastRightUs = SERVO_MID;
  bool  running = false;
  unsigned long lastFlapTime = 0;
  float batteryVoltage = 0.0f;
  bool  lowVoltageWarning = false;
} state;

bool reverseLeft = false;
bool reverseRight = false;

// ====================== 电压读取 ======================
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  return (raw / 4095.0f) * 3.3f * VOLTAGE_DIVIDER;
}

// ====================== HTML页面（带电压显示） ======================
String htmlPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>扑翼蝴蝶控制台</title>
<style>
body { font-family: Arial, "Microsoft YaHei", sans-serif; margin:18px; background:#101418; color:#f2f2f2; }
.card { background:#1c232b; padding:16px; margin:12px 0; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,.25); }
button { font-size:16px; padding:10px 14px; margin:5px; border:0; border-radius:8px; }
.on { background:#37c871; color:#07140b; }
.off { background:#ff6767; color:#210303; }
.mode { background:#5596ff; color:white; }
.value { color:#78dce8; font-weight:bold; }
.warning { color:#ffaa00; animation: blink 1s infinite; }
.critical { color:#ff4444; animation: blink 0.6s infinite; }
@keyframes blink { 50% { opacity: 0.4; } }
</style>
</head>
<body>
<h2>扑翼蝴蝶 WiFi 控制台 (S3 Super Mini)</h2>
<div class="card">
  <p>当前模式：<span id="mode" class="value">-</span></p>
  <p>俯仰：<span id="pitch" class="value">-</span>° 横滚：<span id="roll" class="value">-</span>°</p>
  <p>左PWM：<span id="left" class="value">-</span> 右PWM：<span id="right" class="value">-</span></p>
  <p>速度：<span id="throttle" class="value">-</span> 转向：<span id="steer" class="value">-</span></p>
  <p>电池电压：<span id="voltage" class="value">-</span>V <span id="vstatus"></span></p>
</div>

<div class="card">
  <button class="arrow up" onclick="nudge('faster')">▲ 加速</button>
  <button class="arrow left" onclick="nudge('left')">◀ 左转</button>
  <button class="arrow right" onclick="nudge('right')">▶ 右转</button>
  <button class="arrow down" onclick="nudge('slower')">▼ 减速</button>
  <button class="mode" onclick="nudge('center')">方向回正</button>
</div>

<div class="card">
  <button class="on" onclick="cmd('/start')">启动</button>
  <button class="off" onclick="cmd('/stop')">停止</button>
  <br>
  <button class="mode" onclick="setMode('TAKEOFF')">起飞</button>
  <button class="mode" onclick="setMode('CRUISE')">巡航</button>
  <button class="mode" onclick="setMode('CORRECT')">纠偏</button>
  <button class="mode" onclick="setMode('LAND')">降落</button>
</div>

<div class="card">
  <label>幅度上限：<span id="ampv">120</span>°</label>
  <input id="amp" type="range" min="60" max="130" value="120" oninput="ampv.innerText=this.value" onchange="apply()">
  <label>速度：<span id="speedv">0.03</span></label>
  <input id="speed" type="range" min="0" max="6" value="3" oninput="speedv.innerText=(this.value/100).toFixed(2)" onchange="apply()">
  <label>相位差：<span id="phasev">0</span>°</label>
  <input id="phase" type="range" min="0" max="180" value="0" oninput="phasev.innerText=this.value" onchange="apply()">
</div>

<script>
function cmd(path){ fetch(path).then(update); }
function setMode(m){ fetch('/mode?m='+m).then(update); }
function nudge(a){ fetch('/nudge?action='+a).then(update); }
function apply(){
  fetch('/set?amp='+document.getElementById('amp').value+
        '&speed='+(document.getElementById('speed').value/100).toFixed(2)+
        '&phase='+document.getElementById('phase').value).then(update);
}
function update(){
  fetch('/status').then(r=>r.json()).then(s=>{
    document.getElementById('mode').innerText = s.mode;
    document.getElementById('pitch').innerText = s.pitch.toFixed(1);
    document.getElementById('roll').innerText = s.roll.toFixed(1);
    document.getElementById('left').innerText = s.left;
    document.getElementById('right').innerText = s.right;
    document.getElementById('throttle').innerText = s.throttle.toFixed(2);
    document.getElementById('steer').innerText = s.steer;

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

// ====================== 辅助函数 ======================
String modeName(FlightMode m) {
  switch(m) { case MODE_STOP: return "STOP"; case MODE_TAKEOFF: return "TAKEOFF";
    case MODE_CRUISE: return "CRUISE"; case MODE_CORRECT: return "CORRECT";
    case MODE_LAND: return "LAND"; default: return "UNKNOWN"; }
}

// ★ 修改：各飞行模式下的油门默认值按 0~0.06 的比例重新估算分配
void applyModeDefaults() {
  switch(state.mode) {
    case MODE_STOP:    state.throttle = 0.00f; state.maxAmplitudeDeg = 120; break;
    case MODE_TAKEOFF: state.throttle = 0.05f; state.maxAmplitudeDeg = 125; break;
    case MODE_CRUISE:  state.throttle = 0.03f; state.maxAmplitudeDeg = 105; break;
    case MODE_CORRECT: state.throttle = 0.03f; state.maxAmplitudeDeg = 100; break;
    case MODE_LAND:    state.throttle = 0.01f; state.maxAmplitudeDeg = 75;  break;
  }
}

float multiStageMap(float t, const float s[], const float v[], int n) {
  for(int i=0; i<n-1; i++) if(t >= s[i] && t <= s[i+1])
    return v[i] + (v[i+1]-v[i])*(t-s[i])/(s[i+1]-s[i]);
  return v[n-1];
}

void writeServos(int l, int r) {
  l = constrain(l, SERVO_MIN, SERVO_MAX);
  r = constrain(r, SERVO_MIN, SERVO_MAX);
  if(reverseLeft) l = SERVO_MIN + SERVO_MAX - l;
  if(reverseRight) r = SERVO_MIN + SERVO_MAX - r;
  state.lastLeftUs = l; state.lastRightUs = r;
  leftServo.writeMicroseconds(l);
  rightServo.writeMicroseconds(r);
}

// ====================== 控制任务（含低压保护） ======================
void controlTask(void *pv) {
  const float stages[] = {0.0, 0.25, 0.5, 0.8, 1.0};
  const float freqs[]  = {1.2, 2.8, 4.8, 5.8, 9.0};
  const float amps[]   = {120, 100, 90, 70, 58};

  while(true) {
    mpu.update();
    float roll = mpu.getAngleX();

    state.batteryVoltage = readBatteryVoltage();
    state.lowVoltageWarning = state.batteryVoltage < LOW_VOLTAGE;
    bool critical = state.batteryVoltage < CRITICAL_VOLTAGE;

    float dt = (micros() - state.lastFlapTime) * 1e-6f;
    state.lastFlapTime = micros();
    if(dt <= 0 || dt > 0.1f) dt = 0.01f;

    float effectiveThrottle = state.throttle;
    if(critical) effectiveThrottle = min(effectiveThrottle, 0.02f);
    else if(state.lowVoltageWarning) effectiveThrottle = min(effectiveThrottle, 0.04f);

    if(!state.running || state.mode == MODE_STOP || effectiveThrottle < 0.001f) {
      writeServos(SERVO_MID, SERVO_MID); 
      state.flapPhase = 0;
      vTaskDelay(20);
      continue;
    }

    // ★ 修改：基准归一化分母从 0.20f 改为 0.06f 对应全油门区间
    float t = constrain(effectiveThrottle / 0.06f, 0.0f, 1.0f);
    float freq = multiStageMap(t, stages, freqs, 5);
    float ampDeg = min(multiStageMap(t, stages, amps, 5), (float)state.maxAmplitudeDeg);
    float ampUs = (ampDeg * 0.5f) * (2000.0f / 180.0f);

    state.flapPhase += freq * dt;
    if(state.flapPhase >= 1.0f) state.flapPhase -= 1.0f;

    float stick = state.steerBias / 15.0f;
    float P = roll * ROLL_STAB_GAIN;
    float D = (roll - state.lastRollError) * ROLL_D_GAIN;
    float total = stick * ROLL_STICK_GAIN + (P + D) / 100.0f;

    if(state.mode == MODE_CORRECT) total += roll * 1.2f;
    if(state.mode == MODE_LAND) total *= 0.6f;

    float diff = constrain(total, -ROLL_DIFF_LIMIT, ROLL_DIFF_LIMIT);
    state.lastRollError = roll;

    float waveL = sinf(state.flapPhase * TWO_PI);
    float waveR = sinf(state.flapPhase * TWO_PI + radians(state.rightPhaseOffsetDeg));

    int pwmL = SERVO_MID + ROLL_TRIM + (int)(waveL * ampUs * (1.0f + diff));
    int pwmR = SERVO_MID - ROLL_TRIM + (int)(waveR * ampUs * (1.0f - diff));

    writeServos(pwmL, pwmR);
    state.lastRoll = roll;

    vTaskDelay(8);
  }
}

// ====================== WebServer ======================
void setupServer() {
  server.on("/", [](){ server.send(200, "text/html", htmlPage()); });

  server.on("/status", [](){
    StaticJsonDocument<512> doc;
    doc["mode"] = modeName(state.mode);
    doc["pitch"] = mpu.getAngleY();
    doc["roll"] = state.lastRoll;
    doc["left"] = state.lastLeftUs;
    doc["right"] = state.lastRightUs;
    doc["throttle"] = state.throttle;
    doc["steer"] = state.steerBias;
    doc["phase"] = state.rightPhaseOffsetDeg;
    doc["voltage"] = state.batteryVoltage;
    doc["lowWarning"] = state.lowVoltageWarning;
    doc["critical"] = (state.batteryVoltage < CRITICAL_VOLTAGE);
    String json; serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  server.on("/start", [](){ state.running = true; if(state.mode==MODE_STOP) state.mode=MODE_CRUISE; server.send(200,"ok"); });
  server.on("/stop", [](){ state.running = false; state.mode = MODE_STOP; server.send(200,"ok"); });

  server.on("/mode", [](){
    if(server.hasArg("m")) {
      String m = server.arg("m"); m.toUpperCase();
      if(m=="TAKEOFF") state.mode = MODE_TAKEOFF;
      else if(m=="CRUISE") state.mode = MODE_CRUISE;
      else if(m=="CORRECT") state.mode = MODE_CORRECT;
      else if(m=="LAND") state.mode = MODE_LAND;
      else state.mode = MODE_STOP;
      applyModeDefaults();
    }
    server.send(200, "text/plain", modeName(state.mode));
  });

  server.on("/set", [](){
    if(server.hasArg("amp")) state.maxAmplitudeDeg = constrain(server.arg("amp").toInt(), 60, 130);
    // ★ 修改：油门设置上限约束收窄至 0.06f
    if(server.hasArg("speed")) state.throttle = constrain(server.arg("speed").toFloat(), 0.0f, 0.06f);
    if(server.hasArg("phase")) state.rightPhaseOffsetDeg = constrain(server.arg("phase").toFloat(), 0.0f, 180.0f);
    server.send(200, "ok");
  });

  server.on("/reverse", [](){
    if(server.hasArg("left")) reverseLeft = server.arg("left").toInt() == 1;
    if(server.hasArg("right")) reverseRight = server.arg("right").toInt() == 1;
    server.send(200, "ok");
  });

  server.on("/nudge", [](){
    String a = server.arg("action");
    if(a=="left") state.steerBias = constrain(state.steerBias-5, -15, 15);
    else if(a=="right") state.steerBias = constrain(state.steerBias+5, -15, 15);
    else if(a=="center") state.steerBias = 0;
    // ★ 修改：细微加减速的步长改为 0.01f，上限锁定为 0.06f
    else if(a=="faster") { state.throttle = constrain(state.throttle+0.01f, 0.0f, 0.06f); state.running=true; }
    else if(a=="slower") state.throttle = constrain(state.throttle-0.01f, 0.0f, 0.06f);
    server.send(200, "ok");
  });

  server.begin();
}

// ====================== Setup ======================
void setup() {
  Serial.begin(115200);
  pinMode(BATTERY_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.begin();
  mpu.calcOffsets(true, true);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  leftServo.setPeriodHertz(SERVO_HZ);
  rightServo.setPeriodHertz(SERVO_HZ);
  leftServo.attach(LEFT_SERVO_PIN, SERVO_MIN, SERVO_MAX);
  rightServo.attach(RIGHT_SERVO_PIN, SERVO_MIN, SERVO_MAX);

  writeServos(SERVO_MID, SERVO_MID);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  setupServer();

  xTaskCreatePinnedToCore(controlTask, "FlapCtrl", 8192, NULL, 3, NULL, 1);

  Serial.println("\nButterfly Drone (ESP32-S3 Super Mini) 已启动！");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();
  vTaskDelay(5);
}
