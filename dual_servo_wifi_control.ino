#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <ESP32Servo.h>

#define SDA_PIN 6
#define SCL_PIN 5
#define LEFT_SERVO_PIN 4
#define RIGHT_SERVO_PIN 7

// 参考 butterfly_repo 的扑翼动力学参数：使用微秒 PWM、频率/幅度分段映射、横滚差动补偿。
#define SERVO_MID 1520
#define SERVO_MIN 500
#define SERVO_MAX 2500
#define SERVO_HZ 100

const float ROLL_STAB_GAIN = 30.0f;   // 姿态稳定增益：越小，MPU6050 横滚补偿越强
const float ROLL_STICK_GAIN = 1.0f;   // 网页左右箭头转向灵敏度
const float ROLL_DIFF_LIMIT = 0.30f;  // 最大 ±30% 扑翼差动补偿
const int ROLL_TRIM = 0;
const int WING_UP_OFFSET = 600;       // 停止/怠速时翅膀竖起量，单位 us

const char* AP_SSID = "ButterflyDrone";
const char* AP_PASS = "12345678";

WebServer server(80);
MPU6050 mpu(Wire);
Servo leftServo;
Servo rightServo;

enum FlightMode {
  MODE_STOP,
  MODE_TAKEOFF,
  MODE_CRUISE,
  MODE_CORRECT,
  MODE_LAND
};

FlightMode mode = MODE_STOP;

float phase = 0.0;                 // 0.0 ~ 1.0 的扑翼相位，不再使用弧度
float speedValue = 0.10;            // 网页速度控制量，0.00 ~ 0.20，内部映射为油门 t=0~1
int amplitude = 120;                // 扑翼总幅度上限，单位：角度。参考程序会随速度自动降低幅度。
int center = SERVO_MID;
float rightPhaseOffsetDeg = 180.0;  // 保留给网页显示/后续扩展；当前参考逻辑采用左右反相输出

bool reverseLeft = false;
bool reverseRight = false;
bool running = false;

// 手动转向偏置：负数偏左，正数偏右。通过网页左右大箭头控制。
int steerBias = 0;
const int STEER_STEP = 5;
const int STEER_MAX = 15;

float lastPitch = 0;
float lastRoll = 0;
int lastLeftAngle = SERVO_MID;
int lastRightAngle = SERVO_MID;
unsigned long lastFlapUpdate = 0;
unsigned long lastControl = 0;

String modeName() {
  switch (mode) {
    case MODE_STOP: return "STOP";
    case MODE_TAKEOFF: return "TAKEOFF";
    case MODE_CRUISE: return "CRUISE";
    case MODE_CORRECT: return "CORRECT";
    case MODE_LAND: return "LAND";
  }
  return "UNKNOWN";
}

void setModeByName(String m) {
  m.toUpperCase();
  if (m == "STOP") {
    mode = MODE_STOP;
    running = false;
  } else if (m == "TAKEOFF") {
    mode = MODE_TAKEOFF;
    running = true;
  } else if (m == "CRUISE") {
    mode = MODE_CRUISE;
    running = true;
  } else if (m == "CORRECT") {
    mode = MODE_CORRECT;
    running = true;
  } else if (m == "LAND") {
    mode = MODE_LAND;
    running = true;
  }
}

void applyModeDefaults() {
  switch (mode) {
    case MODE_STOP:
      amplitude = 120;
      speedValue = 0.00;
      break;
    case MODE_TAKEOFF:
      amplitude = 120;
      speedValue = 0.16;
      break;
    case MODE_CRUISE:
      amplitude = 100;
      speedValue = 0.10;
      break;
    case MODE_CORRECT:
      amplitude = 100;
      speedValue = 0.11;
      break;
    case MODE_LAND:
      amplitude = 80;
      speedValue = 0.05;
      break;
  }
}

float multiStageMap(float t, const float stages[], const float values[], int size) {
  for (int i = 0; i < size - 1; i++) {
    if (t >= stages[i] && t <= stages[i + 1]) {
      return values[i] + (values[i + 1] - values[i]) * (t - stages[i]) / (stages[i + 1] - stages[i]);
    }
  }
  return values[size - 1];
}

void writeServos(int leftUs, int rightUs) {
  leftUs = constrain(leftUs, SERVO_MIN, SERVO_MAX);
  rightUs = constrain(rightUs, SERVO_MIN, SERVO_MAX);

  if (reverseLeft) leftUs = SERVO_MIN + SERVO_MAX - leftUs;
  if (reverseRight) rightUs = SERVO_MIN + SERVO_MAX - rightUs;

  lastLeftAngle = leftUs;
  lastRightAngle = rightUs;

  leftServo.writeMicroseconds(leftUs);
  rightServo.writeMicroseconds(rightUs);
}

String htmlPage() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>扑翼蝴蝶控制台</title>
<style>
body { font-family: Arial, "Microsoft YaHei", "PingFang SC", sans-serif; margin: 18px; background:#101418; color:#f2f2f2; }
.card { background:#1c232b; padding:16px; margin:12px 0; border-radius:12px; box-shadow:0 2px 10px rgba(0,0,0,.25); }
button { font-size:16px; padding:10px 14px; margin:5px; border:0; border-radius:8px; }
.on { background:#37c871; color:#07140b; }
.off { background:#ff6767; color:#210303; }
.mode { background:#5596ff; color:white; }
input { width: 100%; }
label { display:block; margin-top:12px; }
.value { color:#78dce8; font-weight:bold; }
.hint { color:#aab3bd; font-size:13px; line-height:1.5; }
.pad { display:grid; grid-template-columns:1fr 1fr 1fr; gap:10px; max-width:360px; margin:12px auto; align-items:center; }
.arrow { font-size:42px; min-height:78px; border-radius:18px; background:#2d3d4f; color:#fff; box-shadow:0 3px 12px rgba(0,0,0,.3); }
.arrow:active { transform:scale(.96); background:#78dce8; color:#101418; }
.up { grid-column:2; }
.left { grid-column:1; }
.mid { grid-column:2; font-size:14px; color:#aab3bd; text-align:center; }
.right { grid-column:3; }
.down { grid-column:2; }
.readout { text-align:center; color:#78dce8; font-weight:bold; }
</style>
</head>
<body>
<h2>扑翼蝴蝶 WiFi 控制台</h2>
<div class="card">
  <p>当前模式：<span id="mode" class="value">-</span></p>
  <p>俯仰角：<span id="pitch" class="value">-</span>°　横滚角：<span id="roll" class="value">-</span>°</p>
  <p>左舵机 PWM：<span id="left" class="value">-</span>us　右舵机 PWM：<span id="right" class="value">-</span>us</p>
  <p>速度：<span id="speedNow" class="value">-</span>　转向偏置：<span id="steer" class="value">-</span></p>
</div>
<div class="card">
  <div class="pad">
    <button class="arrow up" onclick="nudge('faster')">▲</button>
    <button class="arrow left" onclick="nudge('left')">◀</button>
    <div class="mid">方向 / 速度<br>快速控制</div>
    <button class="arrow right" onclick="nudge('right')">▶</button>
    <button class="arrow down" onclick="nudge('slower')">▼</button>
  </div>
  <div class="readout">▲ 加速　▼ 减速　◀ 左转　▶ 右转</div>
  <p class="hint">左右箭头会增加转向偏置；上下箭头会直接调节扑翼速度。点击“方向回正”可清除转向偏置。</p>
  <button class="mode" onclick="nudge('center')">方向回正</button>
</div>
<div class="card">
  <button class="on" onclick="cmd('/start')">启动扑翼</button>
  <button class="off" onclick="cmd('/stop')">停止 / 翅膀竖起</button>
  <br>
  <button class="mode" onclick="mode('TAKEOFF')">起飞模式</button>
  <button class="mode" onclick="mode('CRUISE')">巡航模式</button>
  <button class="mode" onclick="mode('CORRECT')">纠偏模式</button>
  <button class="mode" onclick="mode('LAND')">降落模式</button>
  <p class="hint">建议先空载测试，确认舵机方向和供电稳定后再安装翅膀。</p>
</div>
<div class="card">
  <label>扑翼幅度上限：<span id="ampv">120</span>°</label>
  <input id="amp" type="range" min="40" max="120" value="120" oninput="ampv.innerText=this.value" onchange="apply()">
  <label>扑翼速度：<span id="speedv">0.10</span></label>
  <input id="speed" type="range" min="0" max="20" value="10" oninput="speedv.innerText=(this.value/100).toFixed(2)" onchange="apply()">
  <label>右翅相位差：<span id="phasev">150</span>°</label>
  <input id="phase" type="range" min="90" max="210" value="150" oninput="phasev.innerText=this.value" onchange="apply()">
  <p class="hint">相位差 180° 为完全反相；150° 更柔和；120° 会产生明显左右延迟。</p>
</div>
<div class="card">
  <label><input id="revL" type="checkbox" onchange="reverse()"> 左舵机反向</label>
  <label><input id="revR" type="checkbox" onchange="reverse()"> 右舵机反向</label>
  <p class="hint">如果某一侧翅膀方向不对，先勾选对应舵机反向，再考虑调整舵盘安装角度。</p>
</div>
<script>
function modeText(m){
  const map = {
    STOP:'停止',
    TAKEOFF:'起飞',
    CRUISE:'巡航',
    CORRECT:'纠偏',
    LAND:'降落'
  };
  return map[m] || m;
}
function cmd(path){ fetch(path).then(update); }
function mode(m){ fetch('/mode?m=' + m).then(update); }
function nudge(action){
  fetch('/nudge?action=' + action).then(update);
}
function apply(){
  let amp = document.getElementById('amp').value;
  let speed = (document.getElementById('speed').value / 100).toFixed(2);
  let phase = document.getElementById('phase').value;
  fetch('/set?amp=' + amp + '&speed=' + speed + '&phase=' + phase).then(update);
}
function reverse(){
  let l = document.getElementById('revL').checked ? 1 : 0;
  let r = document.getElementById('revR').checked ? 1 : 0;
  fetch('/reverse?left=' + l + '&right=' + r).then(update);
}
function update(){
  fetch('/status').then(r=>r.json()).then(s=>{
    document.getElementById('mode').innerText = modeText(s.mode);
    document.getElementById('pitch').innerText = s.pitch.toFixed(1);
    document.getElementById('roll').innerText = s.roll.toFixed(1);
    document.getElementById('left').innerText = s.left;
    document.getElementById('right').innerText = s.right;
    document.getElementById('speedNow').innerText = s.speed.toFixed(2);
    document.getElementById('steer').innerText = s.steer;
  });
}
setInterval(update, 500);
update();
</script>
</body>
</html>
)rawliteral";
  return html;
}

void handleRoot() {
  server.send(200, "text/html; charset=utf-8", htmlPage());
}

void handleStatus() {
  String json = "{";
  json += "\"mode\":\"" + modeName() + "\",";
  json += "\"pitch\":" + String(lastPitch, 2) + ",";
  json += "\"roll\":" + String(lastRoll, 2) + ",";
  json += "\"left\":" + String(lastLeftAngle) + ",";
  json += "\"right\":" + String(lastRightAngle) + ",";
  json += "\"amp\":" + String(amplitude) + ",";
  json += "\"speed\":" + String(speedValue, 3) + ",";
  json += "\"steer\":" + String(steerBias) + ",";
  json += "\"phase\":" + String(rightPhaseOffsetDeg, 1);
  json += "}";
  server.send(200, "application/json", json);
}

void setupServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);

  server.on("/start", []() {
    running = true;
    if (mode == MODE_STOP) mode = MODE_CRUISE;
    server.send(200, "text/plain", "started");
  });

  server.on("/stop", []() {
    running = false;
    mode = MODE_STOP;
    writeServos(SERVO_MID + WING_UP_OFFSET + ROLL_TRIM, SERVO_MID - WING_UP_OFFSET - ROLL_TRIM);
    server.send(200, "text/plain", "stopped");
  });

  server.on("/mode", []() {
    if (server.hasArg("m")) {
      setModeByName(server.arg("m"));
      applyModeDefaults();
    }
    server.send(200, "text/plain", modeName());
  });

  server.on("/set", []() {
    if (server.hasArg("amp")) amplitude = constrain(server.arg("amp").toInt(), 40, 120);
    if (server.hasArg("speed")) speedValue = constrain(server.arg("speed").toFloat(), 0.0f, 0.20f);
    if (server.hasArg("phase")) rightPhaseOffsetDeg = constrain(server.arg("phase").toFloat(), 90.0f, 210.0f);
    server.send(200, "text/plain", "ok");
  });

  server.on("/reverse", []() {
    if (server.hasArg("left")) reverseLeft = server.arg("left").toInt() == 1;
    if (server.hasArg("right")) reverseRight = server.arg("right").toInt() == 1;
    server.send(200, "text/plain", "ok");
  });

  server.on("/nudge", []() {
    if (server.hasArg("action")) {
      String action = server.arg("action");
      if (action == "left") {
        steerBias = constrain(steerBias - STEER_STEP, -STEER_MAX, STEER_MAX);
      } else if (action == "right") {
        steerBias = constrain(steerBias + STEER_STEP, -STEER_MAX, STEER_MAX);
      } else if (action == "center") {
        steerBias = 0;
      } else if (action == "faster") {
        speedValue = constrain(speedValue + 0.02f, 0.00f, 0.20f);
        running = true;
        if (mode == MODE_STOP) mode = MODE_CRUISE;
      } else if (action == "slower") {
        speedValue = constrain(speedValue - 0.02f, 0.00f, 0.20f);
      }
    }
    server.send(200, "text/plain", "ok");
  });

  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  delay(1500);
  mpu.calcOffsets(true, true);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  leftServo.setPeriodHertz(SERVO_HZ);
  rightServo.setPeriodHertz(SERVO_HZ);
  leftServo.attach(LEFT_SERVO_PIN, SERVO_MIN, SERVO_MAX);
  rightServo.attach(RIGHT_SERVO_PIN, SERVO_MIN, SERVO_MAX);
  writeServos(SERVO_MID + WING_UP_OFFSET + ROLL_TRIM, SERVO_MID - WING_UP_OFFSET - ROLL_TRIM);
  lastFlapUpdate = micros();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  setupServer();
}

void controlLoop() {
  mpu.update();
  lastPitch = mpu.getAngleY();
  lastRoll = mpu.getAngleX();

  unsigned long now = micros();
  float dt = (now - lastFlapUpdate) * 1e-6f;
  if (dt <= 0 || dt > 0.05f) dt = 0.01f;
  lastFlapUpdate = now;

  // 停止/怠速：参考原程序，让翅膀竖起，而不是简单回中。
  if (!running || mode == MODE_STOP || speedValue <= 0.001f) {
    writeServos(SERVO_MID + WING_UP_OFFSET + ROLL_TRIM,
                SERVO_MID - WING_UP_OFFSET - ROLL_TRIM);
    phase = 0;
    return;
  }

  // 网页速度 speedValue 0.00~0.20 映射成原程序中的 throttle t 0.0~1.0。
  float t = constrain(speedValue / 0.20f, 0.0f, 1.0f);

  const float stages[] = {0.0f, 0.25f, 0.5f, 0.8f, 1.0f};
  const float freqValues[] = {1.2f, 2.8f, 4.8f, 5.8f, 9.0f};
  const float ampValues[] = {120.0f, 100.0f, 90.0f, 70.0f, 58.0f};

  float freq = multiStageMap(t, stages, freqValues, 5);
  float totalAmpDeg = multiStageMap(t, stages, ampValues, 5);
  totalAmpDeg = min(totalAmpDeg, (float)amplitude);
  float ampUs = (totalAmpDeg * 0.5f) * (2000.0f / 180.0f);

  phase += freq * dt;
  if (phase >= 1.0f) phase -= 1.0f;

  // 左右箭头相当于遥控横滚杆：steerBias=-15~+15 映射为 -1~+1。
  float stickRoll = (float)steerBias / (float)STEER_MAX;
  float diff = stickRoll * ROLL_STICK_GAIN + (lastRoll / ROLL_STAB_GAIN);

  if (mode == MODE_CORRECT) diff += lastRoll / ROLL_STAB_GAIN;
  if (mode == MODE_LAND) diff *= 0.6f;

  diff = constrain(diff, -ROLL_DIFF_LIMIT, ROLL_DIFF_LIMIT);

  float wave = sinf(phase * 2.0f * PI);

  int pwmL = SERVO_MID + ROLL_TRIM + (int)(wave * ampUs * (1.0f + diff));
  int pwmR = SERVO_MID - ROLL_TRIM - (int)(wave * ampUs * (1.0f - diff));

  writeServos(pwmL, pwmR);
}

void loop() {
  server.handleClient();

  if (millis() - lastControl >= 20) {
    lastControl = millis();
    controlLoop();
  }
}
