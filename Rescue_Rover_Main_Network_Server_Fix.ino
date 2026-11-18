
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <math.h>
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ------------------------- USER SETTINGS -------------------------
// Replace these with the router credentials used by BOTH ESP32 boards.
// The router will assign an IP address automatically through DHCP.
static const char *WIFI_SSID     = "OPPO_A5s";
static const char *WIFI_PASSWORD = "25802580";

// Keep false if the rover's left/right motor wiring is already oriented correctly.
// Change to true only if that whole side runs opposite to the requested direction.
static const bool INVERT_LEFT_SIDE  = false;
static const bool INVERT_RIGHT_SIDE = false;

// ------------------------- GPIO MAP -------------------------
// BTS7960 #1: left front + left rear motors (parallel, subject to current check)
// Tie this driver's R_EN and L_EN together to one GPIO.
static const int PIN_LEFT_RPWM = 25;
static const int PIN_LEFT_LPWM = 26;
static const int PIN_LEFT_EN   = 18;

// BTS7960 #2: right front + right rear motors (parallel, subject to current check)
// Tie this driver's R_EN and L_EN together to one GPIO.
static const int PIN_RIGHT_RPWM = 27;
static const int PIN_RIGHT_LPWM = 32;
static const int PIN_RIGHT_EN   = 19;

// Sensors
static const int PIN_HCSR04_TRIG = 4;
static const int PIN_HCSR04_ECHO = 34;  // input-only; use voltage divider from HC-SR04 ECHO
static const int PIN_I2C_SDA     = 21;
static const int PIN_I2C_SCL     = 22;
static uint8_t mpuAddress = 0x68; // 0x68 or 0x69 depending on AD0 pin

// UART2 to ESP32-S3-CAM. Cross TX/RX between boards.
// Main GPIO13 (RX) <- Camera TX; Main GPIO14 (TX) -> Camera RX.
static const int PIN_CAMERA_UART_RX = 13;
static const int PIN_CAMERA_UART_TX = 14;
static const uint32_t UART_BAUD = 115200;


static const int PIN_I2S_BCLK = 5;   // shared: INMP441 SCK + MAX98357A BCLK
static const int PIN_I2S_WS   = 15;  // shared: INMP441 WS  + MAX98357A LRC
static const int PIN_I2S_DOUT = 23; // ESP32 -> MAX98357A DIN
static const int PIN_I2S_DIN  = 35; // INMP441 SD -> ESP32 (input-only GPIO)
static const uint32_t AUDIO_SAMPLE_RATE = 16000;
static const float AUDIO_SPEAKER_GAIN = 0.45f; // conservative initial gain for bench testing
static const size_t AUDIO_SAMPLES_PER_PACKET = 320; // 20 ms at 16 kHz
static const uint32_t AUDIO_PACKET_BYTES = AUDIO_SAMPLES_PER_PACKET * sizeof(int16_t);

// Motor PWM for Arduino-ESP32 core 3.x API
static const uint32_t MOTOR_PWM_FREQ = 20000;
static const uint8_t  MOTOR_PWM_BITS = 8;  // 0..255
static const uint32_t MOTOR_COMMAND_TIMEOUT_MS = 800;

// Timing
static const uint32_t SENSOR_INTERVAL_MS = 150;
static const uint32_t WS_BROADCAST_INTERVAL_MS = 400;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;

// ------------------------- OBJECTS / STATE -------------------------
WebServer server(80);
WebSocketsServer webSocket(81);

bool pwmReady = false;
bool wifiConfigured = false;
bool networkServersStarted = false;
bool mpuReady = false;
bool cameraReady = false;
bool cameraWifiConnected = false;
bool cameraValidIp = false;
bool cameraReachable = false;

int currentLeft = 0;
int currentRight = 0;
uint32_t lastDriveCommandMs = 0;
uint32_t lastSensorReadMs = 0;
uint32_t lastWsBroadcastMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastCameraMessageMs = 0;

float distanceCm = -1.0f;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float mpuTemperatureC = 0;

String cameraIp = "";
String cameraStatus = "NOT_SEEN";
String cameraError = "";
String uartLine = "";

// ------------------------- AUDIO STATE -------------------------
struct AudioPacket {
  int16_t samples[AUDIO_SAMPLES_PER_PACKET];
};

i2s_chan_handle_t audioTxChannel = nullptr;
i2s_chan_handle_t audioRxChannel = nullptr;
QueueHandle_t microphoneQueue = nullptr; // ESP32 mic -> browser
QueueHandle_t speakerQueue = nullptr;    // browser -> ESP32 speaker
volatile bool audioStreamingEnabled = false;
bool audioReady = false;
int audioClientId = -1;
static int32_t microphoneI2sBuffer[AUDIO_SAMPLES_PER_PACKET * 2];
static int32_t speakerI2sBuffer[AUDIO_SAMPLES_PER_PACKET * 2];

// ------------------------- DASHBOARD -------------------------
static const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>Rescue Rover Control</title>
<style>
:root{color-scheme:dark;--bg:#10151e;--panel:#1a2330;--line:#334155;--text:#edf2f7;--muted:#9fb0c3;--accent:#39a0ed;--danger:#d94b4b;--ok:#27ae79}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,Arial,sans-serif}
header{padding:18px 16px;background:#151e2a;border-bottom:1px solid var(--line)}h1{font-size:21px;margin:0 0 5px}header small{color:var(--muted)}
main{max-width:980px;margin:0 auto;padding:14px;display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:15px;min-width:0}.card h2{font-size:17px;margin:0 0 12px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.btn{border:0;border-radius:10px;padding:15px 9px;background:#2a4058;color:white;font-size:15px;font-weight:650;touch-action:none;user-select:none;min-height:54px}.btn:active{filter:brightness(1.3)}.stop{background:var(--danger)}.scan{background:#235f9d}
.row{display:flex;justify-content:space-between;gap:10px;padding:7px 0;border-bottom:1px solid #2b3747}.row:last-child{border:0}.label{color:var(--muted)}.value{font-weight:650;text-align:right;overflow-wrap:anywhere}
.status{display:inline-block;padding:4px 9px;border-radius:20px;background:#394657;font-size:12px}.good{background:#155b43}.bad{background:#633333}
input[type=range]{width:100%}pre{white-space:pre-wrap;overflow-wrap:anywhere;color:var(--muted)}.hint{font-size:12px;color:var(--muted);line-height:1.45}
</style>
</head>
<body>
<header><h1>Rescue Rover</h1><small>ESP32 DevKit V1 · Main Control Dashboard</small></header>
<main>
<section class="card">
<h2>Manual Drive</h2>
<div class="row"><span class="label">Motor command timeout</span><span class="value">800 ms</span></div>
<p class="label">Speed: <b id="speedText">150</b> / 255</p><input id="speed" type="range" min="70" max="255" value="150">
<div class="grid" style="margin-top:12px">
<div></div><button class="btn drive" data-left="1" data-right="1">▲ Forward</button><div></div>
<button class="btn drive" data-left="-1" data-right="1">◀ Left</button><button class="btn stop" id="stop">STOP</button><button class="btn drive" data-left="1" data-right="-1">Right ▶</button>
<div></div><button class="btn drive" data-left="-1" data-right="-1">▼ Reverse</button><div></div>
</div>
<p class="hint">Press and hold a direction button. Releasing it sends STOP. Start with the rover lifted and low speed.</p>
</section>
<section class="card">
<h2>Network & System</h2>
<div class="row"><span class="label">Wi-Fi</span><span class="value" id="wifi">--</span></div>
<div class="row"><span class="label">Main ESP32 IP (DHCP)</span><span class="value" id="ip">--</span></div>
<div class="row"><span class="label">RSSI</span><span class="value" id="rssi">--</span></div>
<div class="row"><span class="label">WebSocket</span><span class="value" id="wsState">Connecting...</span></div>
<div class="row"><span class="label">PWM ready</span><span class="value" id="pwm">--</span></div>
<div class="row"><span class="label">Left / Right command</span><span class="value" id="driveState">0 / 0</span></div>
</section>
<section class="card">
<h2>Sensor Monitor</h2>
<div class="row"><span class="label">MPU6050</span><span class="value" id="mpu">--</span></div>
<div class="row"><span class="label">Acceleration X / Y / Z (g)</span><span class="value" id="accel">--</span></div>
<div class="row"><span class="label">Gyro X / Y / Z (°/s)</span><span class="value" id="gyro">--</span></div>
<div class="row"><span class="label">Temperature</span><span class="value" id="temp">--</span></div>
<div class="row"><span class="label">HC-SR04 Distance</span><span class="value" id="distance">--</span></div>
</section>
<section class="card">
<h2>Two-Way Audio</h2>
<div class="row"><span class="label">I2S audio hardware</span><span class="value" id="audioReady">--</span></div>
<div class="row"><span class="label">Audio stream</span><span class="value" id="audioStream">--</span></div>
<p class="hint">For microphone permission in Chrome/Edge, open the included Audio_Console.html as a local file (file://), enter this ESP32's DHCP IP, then connect. The normal HTTP dashboard cannot request microphone access on a LAN IP.</p>
</section>
<section class="card">
<h2>Camera / QR</h2>
<div class="row"><span class="label">Camera controller</span><span class="value" id="camOnline">--</span></div>
<div class="row"><span class="label">Camera status</span><span class="value" id="camReady">--</span></div>
<div class="row"><span class="label">Camera Wi-Fi</span><span class="value" id="camWifi">--</span></div>
<div class="row"><span class="label">Decoded IP</span><span class="value" id="camIp">--</span></div>
<div class="row"><span class="label">IP valid / ping reachable</span><span class="value" id="camPing">--</span></div>
<div class="row"><span class="label">Camera message</span><span class="value" id="camError">--</span></div>
<button class="btn scan" id="scan" style="width:100%;margin-top:12px">Start QR Scan</button>
<p class="hint">The S3-CAM must connect to the same routed network to ping the decoded IP. A failed ping does not mean QR decoding failed.</p>
</section>
</main>
<script>
const $=id=>document.getElementById(id);let held=null,ws=null,heartbeat=null;
function api(path){return fetch(path,{cache:'no-store'}).catch(()=>{});}
function stopDrive(){if(held){held=null;clearInterval(heartbeat);heartbeat=null;}api('/api/stop');}
function sendHeld(){if(!held)return;api('/api/drive?left='+held.l+'&right='+held.r);}
document.querySelectorAll('.drive').forEach(btn=>{
 const start=e=>{e.preventDefault();const s=Number($('speed').value);held={l:Number(btn.dataset.left)*s,r:Number(btn.dataset.right)*s};sendHeld();if(heartbeat)clearInterval(heartbeat);heartbeat=setInterval(sendHeld,220);};
 btn.addEventListener('pointerdown',start);
});
window.addEventListener('pointerup',stopDrive);window.addEventListener('pointercancel',stopDrive);window.addEventListener('blur',stopDrive);window.addEventListener('pagehide',()=>api('/api/stop'));
$('stop').addEventListener('click',stopDrive);$('speed').addEventListener('input',()=>{$('speedText').textContent=$('speed').value;if(held){const s=Number($('speed').value);held.l=Math.sign(held.l)*s;held.r=Math.sign(held.r)*s;sendHeld();}});
$('scan').addEventListener('click',()=>api('/api/scan'));
function setText(id,v){$(id).textContent=v;}
function applyState(d){
 setText('wifi',d.wifi||'--');setText('ip',d.ip||'--');setText('rssi',d.rssi===null?'--':d.rssi+' dBm');setText('pwm',d.pwmReady?'YES':'NO');
 setText('driveState',(d.left||0)+' / '+(d.right||0));
 setText('mpu',d.mpuReady?'READY':'NOT DETECTED');
 setText('accel',d.mpuReady?[d.ax,d.ay,d.az].map(x=>Number(x).toFixed(2)).join(' / '):'--');
 setText('gyro',d.mpuReady?[d.gx,d.gy,d.gz].map(x=>Number(x).toFixed(1)).join(' / '):'--');
 setText('temp',d.mpuReady?Number(d.temp).toFixed(1)+' °C':'--');
 setText('distance',d.distanceCm===null?'No echo':Number(d.distanceCm).toFixed(1)+' cm');
 setText('camOnline',d.camOnline?'ONLINE':'OFFLINE');setText('camReady',d.camReady?'READY':(d.camStatus||'--'));
 setText('camWifi',d.camWifi?'CONNECTED':'DISCONNECTED');setText('camIp',d.camIp||'--');
 setText('camPing',d.camIpValid?(d.camReachable?'VALID / REACHABLE':'VALID / NO PING REPLY'):'NOT VALID / NOT SCANNED');
 setText('camError',d.camError||'--');
 setText('audioReady',d.audioReady?'READY':'NOT READY');
 setText('audioStream',d.audioStreaming?'ACTIVE':'IDLE');
}
function connectWS(){
 ws=new WebSocket('ws://'+location.hostname+':81/');
 ws.onopen=()=>setText('wsState','Connected');ws.onclose=()=>{setText('wsState','Disconnected');setTimeout(connectWS,1500);};
 ws.onerror=()=>{try{ws.close();}catch(e){}};ws.onmessage=e=>{try{applyState(JSON.parse(e.data));}catch(err){}};
}
connectWS();
// HTTP fallback for telemetry if WebSocket is not available.
setInterval(()=>{if(!ws||ws.readyState!==1)fetch('/api/state').then(r=>r.json()).then(applyState).catch(()=>{});},1200);
</script>
</body>
</html>
)HTML";

// ------------------------- MOTOR CONTROL -------------------------
void stopLeftSide() {
  if (pwmReady) {
    ledcWrite(PIN_LEFT_RPWM, 0);
    ledcWrite(PIN_LEFT_LPWM, 0);
  }
  digitalWrite(PIN_LEFT_EN, LOW);
}

void stopRightSide() {
  if (pwmReady) {
    ledcWrite(PIN_RIGHT_RPWM, 0);
    ledcWrite(PIN_RIGHT_LPWM, 0);
  }
  digitalWrite(PIN_RIGHT_EN, LOW);
}

void stopMotors() {
  stopLeftSide();
  stopRightSide();
  currentLeft = 0;
  currentRight = 0;
}

void driveSide(int value, int rpwmPin, int lpwmPin, int enablePin) {
  value = constrain(value, -255, 255);
  if (value == 0 || !pwmReady) {
    if (pwmReady) {
      ledcWrite(rpwmPin, 0);
      ledcWrite(lpwmPin, 0);
    }
    digitalWrite(enablePin, LOW);
    return;
  }

  digitalWrite(enablePin, HIGH);

  if (value > 0) {
    ledcWrite(lpwmPin, 0);
    ledcWrite(rpwmPin, (uint32_t)value);
  } else {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, (uint32_t)(-value));
  }
}

void setDrive(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  if (INVERT_LEFT_SIDE) left = -left;
  if (INVERT_RIGHT_SIDE) right = -right;

  currentLeft = left;
  currentRight = right;
  lastDriveCommandMs = millis();

  driveSide(left, PIN_LEFT_RPWM, PIN_LEFT_LPWM, PIN_LEFT_EN);
  driveSide(right, PIN_RIGHT_RPWM, PIN_RIGHT_LPWM, PIN_RIGHT_EN);
}

// ------------------------- MPU6050 -------------------------
bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadRegisters(uint8_t startReg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t received = Wire.requestFrom((int)mpuAddress, (int)length, (int)true);
  if (received != length) return false;
  for (size_t i = 0; i < length; i++) buffer[i] = Wire.read();
  return true;
}

void initMPU6050() {
  uint8_t who = 0;
  mpuAddress = 0x68;
  if (!mpuReadRegisters(0x75, &who, 1)) {
    mpuAddress = 0x69;
    if (!mpuReadRegisters(0x75, &who, 1)) {
      mpuReady = false;
      Serial.println("MPU6050 not detected at I2C address 0x68 or 0x69.");
      return;
    }
  }
  if (who != 0x68 && who != 0x69) {
    Serial.printf("Unexpected MPU WHO_AM_I: 0x%02X\n", who);
    mpuReady = false;
    return;
  }

  bool ok = true;
  ok &= mpuWriteRegister(0x6B, 0x00); // wake up
  ok &= mpuWriteRegister(0x1B, 0x00); // gyro +/- 250 deg/s
  ok &= mpuWriteRegister(0x1C, 0x00); // accel +/- 2g
  ok &= mpuWriteRegister(0x1A, 0x03); // DLPF
  mpuReady = ok;
  Serial.println(mpuReady ? "MPU6050 initialized." : "MPU6050 initialization failed.");
}

void readMPU6050() {
  if (!mpuReady) return;
  uint8_t b[14];
  if (!mpuReadRegisters(0x3B, b, sizeof(b))) {
    mpuReady = false;
    Serial.println("MPU6050 read failed; sensor marked unavailable.");
    return;
  }
  int16_t ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t az = (int16_t)((b[4] << 8) | b[5]);
  int16_t tr = (int16_t)((b[6] << 8) | b[7]);
  int16_t gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  accelX = ax / 16384.0f;
  accelY = ay / 16384.0f;
  accelZ = az / 16384.0f;
  mpuTemperatureC = (tr / 340.0f) + 36.53f;
  gyroX = gx / 131.0f;
  gyroY = gy / 131.0f;
  gyroZ = gz / 131.0f;
}

// ------------------------- HC-SR04 -------------------------
float readHCSR04DistanceCm() {
  digitalWrite(PIN_HCSR04_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_HCSR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_HCSR04_TRIG, LOW);

  // 25 ms timeout is approximately a 4 m maximum range.
  unsigned long duration = pulseIn(PIN_HCSR04_ECHO, HIGH, 25000UL);
  if (duration == 0) return -1.0f;
  return (duration * 0.0343f) / 2.0f;
}

void updateSensors() {
  uint32_t now = millis();
  if (now - lastSensorReadMs < SENSOR_INTERVAL_MS) return;
  lastSensorReadMs = now;
  readMPU6050();
  distanceCm = readHCSR04DistanceCm();
}

// ------------------------- UART JSON HELPERS -------------------------
String jsonStringValue(const String &json, const char *key) {
  String token = String("\"") + key + "\"";
  int p = json.indexOf(token);
  if (p < 0) return "";
  p = json.indexOf(':', p + token.length());
  if (p < 0) return "";
  p++;
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;
  if (p >= (int)json.length() || json[p] != '"') return "";
  int start = ++p;
  int end = json.indexOf('"', start);
  if (end < 0) return "";
  return json.substring(start, end);
}

bool jsonBoolValue(const String &json, const char *key, bool defaultValue = false) {
  String token = String("\"") + key + "\"";
  int p = json.indexOf(token);
  if (p < 0) return defaultValue;
  p = json.indexOf(':', p + token.length());
  if (p < 0) return defaultValue;
  p++;
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;
  if (json.substring(p, p + 4) == "true") return true;
  if (json.substring(p, p + 5) == "false") return false;
  return defaultValue;
}

String jsonEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    if (c == '"' || c == '\\') out += '\\';
    if (c == '\n' || c == '\r') continue;
    out += c;
  }
  return out;
}

void processCameraJson(const String &line) {
  String type = jsonStringValue(line, "type");
  if (type.length() == 0) return;
  lastCameraMessageMs = millis();

  if (type == "CAM_STATUS") {
    String cam = jsonStringValue(line, "camera");
    String wifi = jsonStringValue(line, "wifi");
    cameraReady = (cam == "READY");
    cameraWifiConnected = (wifi == "CONNECTED");
    cameraStatus = cam.length() ? cam : "UNKNOWN";
  } else if (type == "QR_RESULT") {
    cameraIp = jsonStringValue(line, "ip");
    cameraValidIp = jsonBoolValue(line, "valid", false);
    cameraReachable = jsonBoolValue(line, "reachable", false);
    cameraError = "";
  } else if (type == "QR_ERROR") {
    cameraError = jsonStringValue(line, "error");
  }
}

void readCameraUART() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      uartLine.trim();
      if (uartLine.length()) processCameraJson(uartLine);
      uartLine = "";
    } else if (c != '\r') {
      if (uartLine.length() < 300) uartLine += c;
      else uartLine = ""; // discard oversized/corrupt line
    }
  }
}

void sendCameraCommand(const char *command) {
  Serial2.print("{\"type\":\"");
  Serial2.print(command);
  Serial2.println("\"}");
}

// ------------------------- FULL-DUPLEX I2S AUDIO -------------------------
void microphoneCaptureTask(void *parameter) {
  (void)parameter;
  AudioPacket packet;

  for (;;) {
    size_t bytesRead = 0;
    esp_err_t result = i2s_channel_read(
      audioRxChannel,
      microphoneI2sBuffer,
      sizeof(microphoneI2sBuffer),
      &bytesRead,
      100
    );

    if (result != ESP_OK || bytesRead == 0) continue;
    if (!audioStreamingEnabled || microphoneQueue == nullptr) continue;

    const size_t framesRead = bytesRead / (sizeof(int32_t) * 2);
    for (size_t i = 0; i < AUDIO_SAMPLES_PER_PACKET; ++i) {
      // INMP441 L/R is wired to GND, so use the LEFT slot.
      // The microphone's 24-bit sample is left-aligned in the 32-bit I2S slot.
      int32_t raw = (i < framesRead) ? microphoneI2sBuffer[i * 2] : 0;
      packet.samples[i] = (int16_t)(raw >> 16);
    }

    // Keep latency low: discard the oldest packet if the browser is behind.
    if (xQueueSend(microphoneQueue, &packet, 0) != pdTRUE) {
      AudioPacket discard;
      xQueueReceive(microphoneQueue, &discard, 0);
      xQueueSend(microphoneQueue, &packet, 0);
    }
  }
}

bool initAudioSystem() {
  microphoneQueue = xQueueCreate(4, sizeof(AudioPacket));
  speakerQueue = xQueueCreate(6, sizeof(AudioPacket));
  if (microphoneQueue == nullptr || speakerQueue == nullptr) {
    Serial.println("Audio queue allocation failed.");
    return false;
  }

  i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 6;
  channelConfig.dma_frame_num = 320;

  esp_err_t err = i2s_new_channel(&channelConfig, &audioTxChannel, &audioRxChannel);
  if (err != ESP_OK) {
    Serial.printf("I2S channel allocation failed: %s\n", esp_err_to_name(err));
    return false;
  }

  i2s_std_config_t standardConfig = {};
  standardConfig.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE);
  standardConfig.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
    I2S_DATA_BIT_WIDTH_32BIT,
    I2S_SLOT_MODE_STEREO
  );
  standardConfig.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  standardConfig.gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  standardConfig.gpio_cfg.ws = (gpio_num_t)PIN_I2S_WS;
  standardConfig.gpio_cfg.dout = (gpio_num_t)PIN_I2S_DOUT;
  standardConfig.gpio_cfg.din = (gpio_num_t)PIN_I2S_DIN;
  standardConfig.gpio_cfg.invert_flags.mclk_inv = false;
  standardConfig.gpio_cfg.invert_flags.bclk_inv = false;
  standardConfig.gpio_cfg.invert_flags.ws_inv = false;

  err = i2s_channel_init_std_mode(audioTxChannel, &standardConfig);
  if (err != ESP_OK) {
    Serial.printf("I2S TX initialization failed: %s\n", esp_err_to_name(err));
    return false;
  }
  err = i2s_channel_init_std_mode(audioRxChannel, &standardConfig);
  if (err != ESP_OK) {
    Serial.printf("I2S RX initialization failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = i2s_channel_enable(audioTxChannel);
  if (err != ESP_OK) {
    Serial.printf("I2S TX enable failed: %s\n", esp_err_to_name(err));
    return false;
  }
  err = i2s_channel_enable(audioRxChannel);
  if (err != ESP_OK) {
    Serial.printf("I2S RX enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  BaseType_t taskResult = xTaskCreatePinnedToCore(
    microphoneCaptureTask,
    "audio_mic_capture",
    4096,
    nullptr,
    2,
    nullptr,
    1
  );
  if (taskResult != pdPASS) {
    Serial.println("Audio microphone task creation failed.");
    return false;
  }

  audioReady = true;
  Serial.println("Full-duplex I2S audio initialized: 16 kHz, 16-bit PCM, mono.");
  return true;
}

void serviceAudio() {
  if (!audioReady || !audioStreamingEnabled) return;

  // Rover microphone -> active browser audio console.
  if (audioClientId >= 0 && webSocket.connectedClients() > 0) {
    AudioPacket microphonePacket;
    if (xQueueReceive(microphoneQueue, &microphonePacket, 0) == pdTRUE) {
      webSocket.sendBIN(
        (uint8_t)audioClientId,
        (const uint8_t *)microphonePacket.samples,
        sizeof(microphonePacket.samples)
      );
    }
  }

  // Browser microphone -> Rover speaker. Duplicate mono into L/R I2S slots.
  AudioPacket speakerPacket;
  if (xQueueReceive(speakerQueue, &speakerPacket, 0) == pdTRUE) {
    for (size_t i = 0; i < AUDIO_SAMPLES_PER_PACKET; ++i) {
      int32_t sample32 = (int32_t)((float)speakerPacket.samples[i] * AUDIO_SPEAKER_GAIN * 65536.0f);
      speakerI2sBuffer[i * 2] = sample32;
      speakerI2sBuffer[i * 2 + 1] = sample32;
    }
    size_t bytesWritten = 0;
    i2s_channel_write(
      audioTxChannel,
      speakerI2sBuffer,
      sizeof(speakerI2sBuffer),
      &bytesWritten,
      20
    );
  }
}

void stopAudioStreaming() {
  audioStreamingEnabled = false;
  audioClientId = -1;
  if (microphoneQueue) xQueueReset(microphoneQueue);
  if (speakerQueue) xQueueReset(speakerQueue);
}

// ------------------------- JSON STATE -------------------------
String buildStateJson() {
  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  bool camOnline = (millis() - lastCameraMessageMs) < 10000UL;
  String ip = wifiConnected ? WiFi.localIP().toString() : "";
  String json = "{";
  json += "\"wifi\":\"" + String(wifiConnected ? "CONNECTED" : "DISCONNECTED") + "\",";
  json += "\"ip\":\"" + jsonEscape(ip) + "\",";
  json += "\"rssi\":" + String(wifiConnected ? WiFi.RSSI() : 0) + ",";
  json += "\"pwmReady\":" + String(pwmReady ? "true" : "false") + ",";
  json += "\"left\":" + String(currentLeft) + ",\"right\":" + String(currentRight) + ",";
  json += "\"mpuReady\":" + String(mpuReady ? "true" : "false") + ",";
  json += "\"ax\":" + (mpuReady ? String(accelX, 3) : "0") + ",";
  json += "\"ay\":" + (mpuReady ? String(accelY, 3) : "0") + ",";
  json += "\"az\":" + (mpuReady ? String(accelZ, 3) : "0") + ",";
  json += "\"gx\":" + (mpuReady ? String(gyroX, 2) : "0") + ",";
  json += "\"gy\":" + (mpuReady ? String(gyroY, 2) : "0") + ",";
  json += "\"gz\":" + (mpuReady ? String(gyroZ, 2) : "0") + ",";
  json += "\"temp\":" + (mpuReady ? String(mpuTemperatureC, 2) : "0") + ",";
  json += "\"distanceCm\":" + (distanceCm < 0 ? String("null") : String(distanceCm, 1)) + ",";
  json += "\"camOnline\":" + String(camOnline ? "true" : "false") + ",";
  json += "\"camReady\":" + String(cameraReady ? "true" : "false") + ",";
  json += "\"camWifi\":" + String(cameraWifiConnected ? "true" : "false") + ",";
  json += "\"camStatus\":\"" + jsonEscape(cameraStatus) + "\",";
  json += "\"camIp\":\"" + jsonEscape(cameraIp) + "\",";
  json += "\"camIpValid\":" + String(cameraValidIp ? "true" : "false") + ",";
  json += "\"camReachable\":" + String(cameraReachable ? "true" : "false") + ",";
  json += "\"camError\":\"" + jsonEscape(cameraError) + "\",";
  json += "\"audioReady\":" + String(audioReady ? "true" : "false") + ",";
  json += "\"audioStreaming\":" + String(audioStreamingEnabled ? "true" : "false");
  json += "}";
  return json;
}

// ------------------------- HTTP HANDLERS -------------------------
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
}

void handleState() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildStateJson());
}

void handleDrive() {
  if (!server.hasArg("left") || !server.hasArg("right")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"left_and_right_required\"}");
    return;
  }
  int left = constrain(server.arg("left").toInt(), -255, 255);
  int right = constrain(server.arg("right").toInt(), -255, 255);
  setDrive(left, right);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  stopMotors();
  lastDriveCommandMs = millis();
  server.send(200, "application/json", "{\"ok\":true,\"stopped\":true}");
}

void handleScanStart() {
  sendCameraCommand("SCAN_START");
  server.send(200, "application/json", "{\"ok\":true,\"sent\":\"SCAN_START\"}");
}

void handleScanStop() {
  sendCameraCommand("SCAN_STOP");
  server.send(200, "application/json", "{\"ok\":true,\"sent\":\"SCAN_STOP\"}");
}

void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length);

void setupHttpServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/drive", HTTP_GET, handleDrive);
  server.on("/api/stop", HTTP_GET, handleStop);
  server.on("/api/scan", HTTP_GET, handleScanStart);
  server.on("/api/scan-stop", HTTP_GET, handleScanStop);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.println("HTTP dashboard server started on port 80.");
}

void startNetworkServers() {
  if (networkServersStarted) return;

  setupHttpServer();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  networkServersStarted = true;

  Serial.print("Dashboard URL: http://");
  Serial.println(WiFi.localIP());
  Serial.println("WebSocket server started on port 81.");
}

void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    String stateMessage = buildStateJson();
    webSocket.sendTXT(clientNum, stateMessage);
    return;
  }

  if (type == WStype_TEXT && payload != nullptr && length > 0 && length <= 128) {
    String message;
    message.reserve(length + 1);
    for (size_t i = 0; i < length; ++i) message += (char)payload[i];
    String command = jsonStringValue(message, "type");

    if (command == "AUDIO_START") {
      if (audioReady) {
        stopAudioStreaming();
        audioClientId = clientNum;
        audioStreamingEnabled = true;
        webSocket.sendTXT(clientNum, "{\"type\":\"AUDIO_ACK\",\"enabled\":true}");
        Serial.printf("Audio stream started for WebSocket client %u.\n", clientNum);
      } else {
        webSocket.sendTXT(clientNum, "{\"type\":\"AUDIO_ACK\",\"enabled\":false,\"error\":\"I2S_NOT_READY\"}");
      }
    } else if (command == "AUDIO_STOP") {
      if (audioClientId == clientNum) {
        stopAudioStreaming();
        webSocket.sendTXT(clientNum, "{\"type\":\"AUDIO_ACK\",\"enabled\":false}");
        Serial.println("Audio stream stopped.");
      }
    }
    return;
  }

  if (type == WStype_BIN) {
    
    if (audioStreamingEnabled && audioClientId == clientNum &&
        payload != nullptr && length == AUDIO_PACKET_BYTES && speakerQueue != nullptr) {
      AudioPacket packet;
      memcpy(packet.samples, payload, AUDIO_PACKET_BYTES);
      if (xQueueSend(speakerQueue, &packet, 0) != pdTRUE) {
        AudioPacket discard;
        xQueueReceive(speakerQueue, &discard, 0);
        xQueueSend(speakerQueue, &packet, 0);
      }
    }
    return;
  }

  if (type == WStype_DISCONNECTED) {
    if (audioClientId == clientNum) {
      stopAudioStreaming();
      Serial.println("Audio WebSocket client disconnected.");
    }
    if (webSocket.connectedClients() == 0) {
      
      stopMotors();
    }
  }
}

// ------------------------- WIFI -------------------------
void beginWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  
  if (strlen(WIFI_SSID) == 0 || strlen(WIFI_PASSWORD) == 0) {
    wifiConfigured = false;
    Serial.println("Wi-Fi SSID or password is empty. Set both near the top of the sketch.");
    return;
  }
  wifiConfigured = true;
  Serial.print("Connecting to Wi-Fi SSID: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptMs = millis();
}

void maintainWiFi() {
  // Report the connection result once, including the DHCP-assigned IP address.
  static bool wasConnected = false;
  const bool isConnected = (WiFi.status() == WL_CONNECTED);

  if (isConnected && !wasConnected) {
    Serial.println();
    Serial.println("========== WI-FI CONNECTED ==========");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Subnet Mask: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    Serial.println("Open the IP address above in your browser.");
    Serial.println("=====================================");
    startNetworkServers();
  } else if (!isConnected && wasConnected) {
    Serial.println("Wi-Fi connection lost.");
  }
  wasConnected = isConnected;

  if (!wifiConfigured || isConnected) return;
  if (millis() - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiAttemptMs = millis();
    Serial.println("Wi-Fi reconnect attempt...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    stopMotors();
  }
}

// ------------------------- SETUP / LOOP -------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== Rescue Rover Main Controller ===");

  // Fail-safe first: motor outputs remain disabled at boot.
  pinMode(PIN_LEFT_EN, OUTPUT);
  pinMode(PIN_RIGHT_EN, OUTPUT);
  digitalWrite(PIN_LEFT_EN, LOW);
  digitalWrite(PIN_RIGHT_EN, LOW);

  bool pwm1 = ledcAttach(PIN_LEFT_RPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  bool pwm2 = ledcAttach(PIN_LEFT_LPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  bool pwm3 = ledcAttach(PIN_RIGHT_RPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  bool pwm4 = ledcAttach(PIN_RIGHT_LPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  pwmReady = pwm1 && pwm2 && pwm3 && pwm4;
  stopMotors();
  Serial.printf("Motor PWM ready: %s\n", pwmReady ? "YES" : "NO");

  pinMode(PIN_HCSR04_TRIG, OUTPUT);
  pinMode(PIN_HCSR04_ECHO, INPUT);
  digitalWrite(PIN_HCSR04_TRIG, LOW);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  initMPU6050();

  Serial2.begin(UART_BAUD, SERIAL_8N1, PIN_CAMERA_UART_RX, PIN_CAMERA_UART_TX);
  Serial.println("Camera UART2 initialized at 115200 baud (RX=GPIO13, TX=GPIO14).");

  audioReady = initAudioSystem();
  Serial.printf("Audio hardware ready: %s\n", audioReady ? "YES" : "NO");

  
  beginWiFi();

  lastDriveCommandMs = millis();
  Serial.println("Startup complete. Motors are DISABLED until a dashboard drive command arrives.");
}

void loop() {
  maintainWiFi();

  if (networkServersStarted) {
    server.handleClient();
    webSocket.loop();
    serviceAudio();
  }

  readCameraUART();
  updateSensors();

  // Stop motors if browser commands stop arriving.
  if ((currentLeft != 0 || currentRight != 0) &&
      (millis() - lastDriveCommandMs > MOTOR_COMMAND_TIMEOUT_MS)) {
    Serial.println("Motor command timeout: stopping motors.");
    stopMotors();
  }

  // Stop if Wi-Fi is lost; do not keep driving without an active dashboard link.
  if (WiFi.status() != WL_CONNECTED && (currentLeft != 0 || currentRight != 0)) {
    stopMotors();
  }

  if (networkServersStarted &&
      millis() - lastWsBroadcastMs >= WS_BROADCAST_INTERVAL_MS) {
    lastWsBroadcastMs = millis();
    String stateMessage = buildStateJson();
    webSocket.broadcastTXT(stateMessage);
  }

  delay(2);
}
