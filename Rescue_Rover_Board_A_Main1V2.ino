/*
  =============================================================================
  Rescue Rover - Board A: Main Deck Master Controller (Rev 1.0 Spec + Keyboard)
  MCU: ESP32-WROOM-32 (DevKit V1, 30-Pin)
  - HyperFlux ESP32-C3 Motor Driver: 32-byte FlySky i-BUS @ 50Hz on TX2 (GPIO 17)
  - Optional FlySky i6X Receiver: i-BUS RX on RX2 (GPIO 16)
  - 4x HC-SR04 Sonars: Common TRIG=27, ECHO Front=32, Rear=33, Left=25, Right=26
  - MPU6050 6-Axis IMU: SDA=21, SCL=22 (400 kHz) -> Roll & Pitch Fusion
  - Inter-Board UART1 (250000 baud): TX1=19 -> S3 RX1(2), RX1=18 <- S3 TX1(1)
  - Input-only pins (GPIO 34, 35, 36, 39) strictly unconnected (NC)
  =============================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>

// ------------------------- WIFI SETTINGS -------------------------
static const char *WIFI_SSID     = "UIU-STUDENT";
static const char *WIFI_PASSWORD = "12345678";
static const char *AP_SSID       = "Rescue_Rover_Master";
static const char *AP_PASS       = "12345678";

// ------------------------- BOARD A PINOUT (REV 1.0 SPEC) -------------------------
// 4x HC-SR04 Ultrasonic Sensors
static const int PIN_SONAR_TRIG       = 27;  // D27: Common TRIG Bus
static const int PIN_SONAR_FRONT_ECHO = 32;  // D32: Sonar 1 (Front)
static const int PIN_SONAR_REAR_ECHO  = 33;  // D33: Sonar 2 (Rear)
static const int PIN_SONAR_LEFT_ECHO  = 25;  // D25: Sonar 3 (Left)
static const int PIN_SONAR_RIGHT_ECHO = 26;  // D26: Sonar 4 (Right)

// MPU6050 I2C Bus (400 kHz)
static const int PIN_I2C_SDA = 21;  // D21
static const int PIN_I2C_SCL = 22;  // D22
static uint8_t mpuAddress = 0x68;

// UART1 to Board B (ESP32-S3-CAM) @ 250000 baud
static const int PIN_S3_UART1_TX = 19;  // D19 -> JST_UART Pin 3 -> S3 GPIO 2
static const int PIN_S3_UART1_RX = 18;  // D18 <- JST_UART Pin 4 <- S3 GPIO 1
static const uint32_t S3_UART_BAUD = 250000;

// UART2 for HyperFlux i-BUS OUT & Optional FlySky i6X IN @ 115200 baud
static const int PIN_IBUS_TX2 = 17;  // TX2 -> HyperFlux GPIO 6 (i-BUS RX)
static const int PIN_IBUS_RX2 = 16;  // RX2 <- Optional FlySky i6X RX
static const uint32_t IBUS_BAUD = 115200;

// Onboard Status LED
static const int PIN_STATUS_LED = 2;  // D2

// ------------------------- TIMING CONSTANTS -------------------------
static const uint32_t IBUS_TX_INTERVAL_MS    = 20;   // 50 Hz deterministic i-BUS
static const uint32_t S3_TLM_INTERVAL_MS     = 20;   // 50 Hz Board A -> Board B TLM
static const uint32_t SONAR_INTERVAL_MS      = 60;   // ~16 Hz 4-sensor burst
static const uint32_t IMU_INTERVAL_MS        = 10;   // 100 Hz IMU fusion
static const uint32_t WS_BROADCAST_MS        = 250;  // 4 Hz Dashboard push
static const uint32_t WEB_DRIVE_TIMEOUT_MS   = 500;  // Auto-neutral failsafe
static const uint32_t FLYSKY_ACTIVE_WINDOW   = 250;  // RC override timeout

// ------------------------- NETWORK & STATE -------------------------
WebServer server(80);
WebSocketsServer webSocket(81);
Preferences prefs;

bool networkReady = false;
String wifiModeStr = "BOOTING";
String savedCamIp = "";

// FlySky i-BUS 14-Channel Array (1000..2000, 1500 = center)
uint16_t ibusCh[14] = {
  1500, 1500, 1500, 1500, 1500, 1500, 1500,
  1500, 1500, 1500, 1500, 1500, 1500, 1500
};

// Drive Command States
int webSteerPct = 0;      // -100..+100
int webThrottlePct = 0;   // -100..+100
int webSpeedPct = 60;     // 5..100 (mapped to CH5 1000..2000)
uint32_t lastWebDriveMs = 0;

// S3-CAM Remote Drive Override (DRV, CH1:..., CH2:..., CH5:...)
uint16_t s3DriveCh1 = 1500;
uint16_t s3DriveCh2 = 1500;
uint16_t s3DriveCh5 = 1500;
uint32_t lastS3DriveMs = 0;

// Hardware FlySky i6X Receiver Parser
uint8_t flyskyRxBuf[32];
uint8_t flyskyRxIdx = 0;
uint32_t lastFlyskyFrameMs = 0;
String activeDriveMode = "WIFI_STOP";

// Sensors
float sonarDist[4] = {-1.0f, -1.0f, -1.0f, -1.0f}; // Front, Rear, Left, Right
bool mpuReady = false;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float mpuTempC = 0.0f;
uint32_t lastImuMicros = 0;

// Board B (S3-CAM) Telemetry from UART1
String s3RxLine = "";
uint32_t lastS3HeartbeatMs = 0;
uint32_t s3LastJpegBytes = 0;
int s3LastMicRms = 0;
String s3LastRawStatus = "WAITING";

// Timers
uint32_t lastIbusTxMs = 0;
uint32_t lastS3TlmMs = 0;
uint32_t lastSonarMs = 0;
uint32_t lastImuMs = 0;
uint32_t lastWsMs = 0;

// ------------------------- MASTER UNIFIED DASHBOARD -------------------------
static const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>Rescue Rover Master Control</title>
<style>
:root{color-scheme:dark;--bg:#0e141d;--panel:#182230;--line:#2d3b4e;--text:#edf2f7;--muted:#9fb0c3;--accent:#38bdf8;--danger:#ef4444;--ok:#10b981}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,Arial,sans-serif}
header{padding:14px 16px;background:#131c27;border-bottom:1px solid var(--line);display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
h1{font-size:19px;margin:0}header small{color:var(--muted)}
main{max-width:1160px;margin:0 auto;padding:14px;display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:15px;min-width:0}
.card h2{font-size:16px;margin:0 0 12px;color:#7dd3fc;display:flex;justify-content:space-between;align-items:center}
.badge{font-size:12px;padding:3px 8px;border-radius:999px;background:#1e293b;color:#38bdf8;border:1px solid #334155}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.btn{border:0;border-radius:10px;padding:14px 8px;background:#263b52;color:#fff;font-size:14px;font-weight:700;touch-action:none;user-select:none;min-height:50px;cursor:pointer}
.btn:active,.btn.kb-active{filter:brightness(1.35);outline:2px solid var(--accent)}.stop{background:var(--danger)}.scan{background:#0284c7}.okbtn{background:var(--ok)}
.btnRow{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:6px 0;border-bottom:1px solid #263345}.row:last-child{border:0}
.label{color:var(--muted);font-size:14px}.value{font-weight:650;text-align:right;font-variant-numeric:tabular-nums}
.vidBox{width:100%;aspect-ratio:4/3;background:#070b10;border:1px solid var(--line);border-radius:10px;object-fit:contain;display:block;margin-bottom:10px}
input[type=text]{width:100%;padding:9px 10px;border-radius:8px;border:1px solid var(--line);background:#0c131d;color:var(--text);font-size:14px}
input[type=range]{width:100%;accent-color:#38bdf8}.hint{font-size:12px;color:var(--muted);line-height:1.45;margin-top:8px}
.sonarGrid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px}
.sBox{background:#111924;border:1px solid #283548;border-radius:10px;padding:10px;text-align:center}
.sBox small{display:block;color:var(--muted);font-size:12px}.sBox b{font-size:17px;color:#f8fafc}
</style>
</head>
<body>
<header>
  <div><h1>Rescue Rover Master Control</h1><small>Board A (ESP32) + HyperFlux i-BUS + Board B (S3-CAM)</small></div>
  <small id="wsBadge">WS: Connecting...</small>
</header>
<main>

<section class="card">
  <h2>Live Camera & Duplex I2S Audio <span class="badge" id="s3Badge">S3: WAIT</span></h2>
  <img id="camStream" class="vidBox" alt="S3-CAM Stream (Auto-connects via UART1 or enter IP)">
  <div style="display:flex;gap:8px;margin-bottom:8px">
    <input type="text" id="camIpInput" placeholder="S3-CAM IP (e.g. 192.168.43.120)">
    <button class="btn okbtn" id="btnSaveIp" style="min-height:40px;padding:8px 14px">Connect</button>
  </div>
  <div class="row"><span class="label">Stream Stats</span><span class="value" id="camTel">Waiting for S3-CAM...</span></div>
  <div class="row"><span class="label">UART1 Link (250k)</span><span class="value" id="s3Uart">--</span></div>
  <div class="btnRow">
    <button class="btn scan" id="btnFeedToggle">Pause / Resume Video</button>
    <button class="btn okbtn" id="btnAuto25">Auto 25+ FPS</button>
    <button class="btn okbtn" id="btnListenS3">Listen S3 Mic (CU20BK40)</button>
    <button class="btn stop" id="btnTalkS3">Hold to Talk (MAX98357A)</button>
  </div>
  <button class="btn" id="btnChime" style="width:100%;margin-top:8px;min-height:42px">Ring S3 Chime</button>
</section>

<section class="card">
  <h2>HyperFlux i-BUS Drive <span class="badge" id="modeBadge">WIFI_STOP</span></h2>
  <p class="label">CH5 Master Speed Limit: <b id="speedText">60</b>% (5%–100%)</p>
  <input id="speed" type="range" min="5" max="100" value="60">
  <div class="grid" style="margin-top:12px">
    <button class="btn drive" data-thr="100" data-str="-65">↖ Fwd-L</button>
    <button class="btn drive" data-thr="100" data-str="0">▲ Forward</button>
    <button class="btn drive" data-thr="100" data-str="65">↗ Fwd-R</button>
    <button class="btn drive" data-thr="0" data-str="-100">◀ Spin L</button>
    <button class="btn stop" id="stop">STOP</button>
    <button class="btn drive" data-thr="0" data-str="100">Spin R ▶</button>
    <button class="btn drive" data-thr="-100" data-str="-65">↙ Rev-L</button>
    <button class="btn drive" data-thr="-100" data-str="0">▼ Reverse</button>
    <button class="btn drive" data-thr="-100" data-str="65">↘ Rev-R</button>
  </div>
  <div class="row" style="margin-top:10px"><span class="label">i-BUS CH1 (Steer) / CH2 (Thr) / CH5</span><span class="value" id="ibusState">1500 / 1500 / 1579</span></div>
  <p class="hint"><b>Keyboard:</b> WASD / Arrows to drive, Space = STOP, +/- = Speed. Plugging a FlySky i6X RX into RX2 (GPIO 16) automatically takes priority.</p>
</section>

<section class="card">
  <h2>4x HC-SR04 Sonars & MPU6050 IMU</h2>
  <div class="sonarGrid">
    <div class="sBox"><small>FRONT (D32)</small><b id="sFront">--</b></div>
    <div class="sBox"><small>REAR (D33)</small><b id="sRear">--</b></div>
    <div class="sBox"><small>LEFT (D25)</small><b id="sLeft">--</b></div>
    <div class="sBox"><small>RIGHT (D26)</small><b id="sRight">--</b></div>
  </div>
  <div class="row"><span class="label">MPU6050 IMU Status</span><span class="value" id="mpu">--</span></div>
  <div class="row"><span class="label">Roll / Pitch Angle</span><span class="value" id="rpAngle">--</span></div>
  <div class="row"><span class="label">Accel X / Y / Z (g)</span><span class="value" id="accel">--</span></div>
  <div class="row"><span class="label">Gyro X / Y / Z (°/s)</span><span class="value" id="gyro">--</span></div>
  <div class="row"><span class="label">Board A Wi-Fi / IP</span><span class="value" id="netInfo">--</span></div>
</section>

</main>
<script>
const $=id=>document.getElementById(id);
let held=null,ws=null,heartbeat=null,camIp='',feedOn=1,rxTimer=null,audioCtx=null,nextPlayTime=0,txStream=null,txProc=null;

function api(path){return fetch(path,{cache:'no-store'}).catch(()=>{});}
function highlightDriveBtn(thr,str){
  document.querySelectorAll('.drive').forEach(b=>{
    const match=(Number(b.dataset.thr)===thr && Number(b.dataset.str)===str);
    b.classList.toggle('kb-active',match);
  });
}
function stopDrive(){
  if(held){held=null;clearInterval(heartbeat);heartbeat=null;}
  highlightDriveBtn(0,0);
  api('/api/stop');
}
function sendHeld(){
  if(!held)return;
  const spd=Number($('speed').value);
  highlightDriveBtn(held.thr,held.str);
  api('/api/drive?thr='+held.thr+'&str='+held.str+'&spd='+spd);
}

document.querySelectorAll('.drive').forEach(btn=>{
  const start=e=>{
    e.preventDefault();
    held={thr:Number(btn.dataset.thr),str:Number(btn.dataset.str)};
    sendHeld();
    if(heartbeat)clearInterval(heartbeat);
    heartbeat=setInterval(sendHeld,150);
  };
  btn.addEventListener('pointerdown',start);
});
window.addEventListener('pointerup',stopDrive);
window.addEventListener('pointercancel',stopDrive);
$('stop').addEventListener('click',stopDrive);
$('speed').addEventListener('input',()=>{
  $('speedText').textContent=$('speed').value;
  api('/api/speed?spd='+$('speed').value);
  if(held)sendHeld();
});

// Keyboard Drive Controls (WASD / Arrows / Space / +/-)
const keysDown=new Set();
function updateKeyboardDrive(){
  if(document.activeElement && document.activeElement.tagName==='INPUT')return;
  let thr=0,str=0;
  if(keysDown.has('KeyW')||keysDown.has('ArrowUp'))thr+=100;
  if(keysDown.has('KeyS')||keysDown.has('ArrowDown'))thr-=100;
  if(keysDown.has('KeyA')||keysDown.has('ArrowLeft'))str-=(thr!==0?65:100);
  if(keysDown.has('KeyD')||keysDown.has('ArrowRight'))str+=(thr!==0?65:100);

  if(thr===0 && str===0){
    stopDrive();
  }else{
    held={thr,str};
    sendHeld();
    if(!heartbeat)heartbeat=setInterval(sendHeld,150);
  }
}

window.addEventListener('keydown',e=>{
  if(document.activeElement && document.activeElement.tagName==='INPUT')return;
  if(e.code==='Space'){e.preventDefault();keysDown.clear();stopDrive();return;}
  if(e.key==='+'||e.key==='='||e.key===']'){
    $('speed').value=Math.min(100,Number($('speed').value)+5);
    $('speed').dispatchEvent(new Event('input'));
    return;
  }
  if(e.key==='-'||e.key==='_'||e.key==='['){
    $('speed').value=Math.max(5,Number($('speed').value)-5);
    $('speed').dispatchEvent(new Event('input'));
    return;
  }
  if(['KeyW','KeyA','KeyS','KeyD','ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.code)){
    e.preventDefault();
    if(!keysDown.has(e.code)){
      keysDown.add(e.code);
      updateKeyboardDrive();
    }
  }
});

window.addEventListener('keyup',e=>{
  if(keysDown.delete(e.code)){
    updateKeyboardDrive();
  }
});
window.addEventListener('blur',()=>{keysDown.clear();stopDrive();});

function attachCamStream(ip){
  if(!ip)return;
  camIp=ip.trim();
  $('camIpInput').value=camIp;
  $('camStream').src='http://'+camIp+':81/stream?t='+Date.now();
}

$('btnSaveIp').onclick=()=>{
  const ip=$('camIpInput').value.trim();
  if(!ip)return;
  api('/api/cam/setip?ip='+encodeURIComponent(ip)).then(()=>attachCamStream(ip));
};

$('btnFeedToggle').onclick=()=>{
  feedOn=feedOn?0:1;
  api('/api/cam/cmd?path='+encodeURIComponent('/api/feed?on='+feedOn));
  if(feedOn && camIp) attachCamStream(camIp);
  else $('camStream').src='';
};
$('btnAuto25').onclick=()=>api('/api/cam/cmd?path='+encodeURIComponent('/api/auto25'));
$('btnChime').onclick=()=>api('/api/cam/cmd?path='+encodeURIComponent('/api/audio/chime'));

$('btnListenS3').onclick=async()=>{
  if(!camIp){alert('Enter S3-CAM IP first');return;}
  if(rxTimer){clearInterval(rxTimer);rxTimer=null;$('btnListenS3').textContent='Listen S3 Mic (CU20BK40)';return;}
  audioCtx=audioCtx||new (window.AudioContext||window.webkitAudioContext)({sampleRate:16000});
  await audioCtx.resume();
  nextPlayTime=audioCtx.currentTime;
  $('btnListenS3').textContent='Stop Listening';
  rxTimer=setInterval(async()=>{
    try{
      const r=await fetch('http://'+camIp+'/api/audio/rx');
      const buf=await r.arrayBuffer();
      if(buf.byteLength<2)return;
      const i16=new Int16Array(buf);
      const ab=audioCtx.createBuffer(1,i16.length,16000);
      const ch=ab.getChannelData(0);
      for(let i=0;i<i16.length;i++)ch[i]=i16[i]/32768.0;
      const src=audioCtx.createBufferSource();
      src.buffer=ab;src.connect(audioCtx.destination);
      if(nextPlayTime<audioCtx.currentTime)nextPlayTime=audioCtx.currentTime;
      src.start(nextPlayTime);nextPlayTime+=ab.duration;
    }catch(e){}
  },120);
};

async function startTalkS3(){
  if(!camIp)return;
  try{
    audioCtx=audioCtx||new (window.AudioContext||window.webkitAudioContext)({sampleRate:16000});
    txStream=await navigator.mediaDevices.getUserMedia({audio:true});
    const src=audioCtx.createMediaStreamSource(txStream);
    txProc=audioCtx.createScriptProcessor(2048,1,1);
    txProc.onaudioprocess=e=>{
      const inp=e.inputBuffer.getChannelData(0);
      const out=new Int16Array(inp.length);
      for(let i=0;i<inp.length;i++){
        let s=Math.max(-1,Math.min(1,inp[i]));
        out[i]=s<0?s*32768:s*32767;
      }
      const fd=new FormData();
      fd.append('pcm',new Blob([out.buffer],{type:'application/octet-stream'}),'voice.pcm');
      fetch('http://'+camIp+'/api/audio/tx',{method:'POST',body:fd,mode:'no-cors'}).catch(()=>{});
    };
    src.connect(txProc);txProc.connect(audioCtx.destination);
    $('btnTalkS3').textContent='Talking to Rover...';
  }catch(err){alert('Mic access requires HTTPS or browser flag: '+err);}
}
function stopTalkS3(){
  if(txProc){txProc.disconnect();txProc=null;}
  if(txStream){txStream.getTracks().forEach(t=>t.stop());txStream=null;}
  $('btnTalkS3').textContent='Hold to Talk (MAX98357A)';
}
$('btnTalkS3').addEventListener('pointerdown',startTalkS3);
window.addEventListener('pointerup',stopTalkS3);

function fmtSonar(v){return (v===null||v<0)?'Out of range':Number(v).toFixed(1)+' cm';}
function setText(id,v){$(id).textContent=v;}

function applyState(d){
  setText('modeBadge',d.mode||'WIFI_STOP');
  setText('ibusState',`${d.ch1} / ${d.ch2} / ${d.ch5}`);
  setText('sFront',fmtSonar(d.s1));
  setText('sRear',fmtSonar(d.s2));
  setText('sLeft',fmtSonar(d.s3));
  setText('sRight',fmtSonar(d.s4));
  setText('mpu',d.mpuReady?'READY (400kHz)':'NOT DETECTED');
  setText('rpAngle',d.mpuReady?`${Number(d.roll).toFixed(1)}° / ${Number(d.pitch).toFixed(1)}°`:'--');
  setText('accel',d.mpuReady?[d.ax,d.ay,d.az].map(x=>Number(x).toFixed(2)).join(' / '):'--');
  setText('gyro',d.mpuReady?[d.gx,d.gy,d.gz].map(x=>Number(x).toFixed(1)).join(' / '):'--');
  setText('netInfo',`${d.wifiMode} · ${d.ip}`);
  setText('s3Badge',d.s3Online?'S3: ONLINE':'S3: WAIT');
  setText('s3Uart',d.s3Online?`JPEG:${d.s3CamBytes}B | MicRMS:${d.s3MicRms}`:'Waiting for S3_OK...');
  if(d.savedCamIp && !camIp){
    attachCamStream(d.savedCamIp);
  }
}

function connectWS(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=()=>setText('wsBadge','WS: Connected');
  ws.onclose=()=>{setText('wsBadge','WS: Reconnecting...');setTimeout(connectWS,1500);};
  ws.onerror=()=>{try{ws.close();}catch(e){}};
  ws.onmessage=e=>{try{applyState(JSON.parse(e.data));}catch(err){}};
}
connectWS();

setInterval(()=>{
  if(!camIp)return;
  fetch('/api/cam/status').then(r=>r.json()).then(s=>{
    if(s.fps!==undefined){
      setText('camTel',`${Number(s.fps).toFixed(1)} FPS | ${Number(s.kbps).toFixed(0)} kbps | Q:${s.quality} | RMS:${s.micRms}`);
    }else{
      setText('camTel','S3-CAM unreachable at '+camIp);
    }
  }).catch(()=>{});
},1500);

setInterval(()=>{if(!ws||ws.readyState!==1)fetch('/api/state').then(r=>r.json()).then(applyState).catch(()=>{});},1000);
</script>
</body>
</html>
)HTML";

// ------------------------- 32-BYTE FLYSKY i-BUS ENGINE -------------------------
uint16_t mapPctToIbus(int pct) {
  pct = constrain(pct, -100, 100);
  return (uint16_t)(1500 + (pct * 5)); // -100 -> 1000, 0 -> 1500, +100 -> 2000
}

uint16_t mapSpeedPctToIbus(int spdPct) {
  spdPct = constrain(spdPct, 5, 100);
  return (uint16_t)map(spdPct, 5, 100, 1000, 2000);
}

void sendIbusFrameToHyperFlux() {
  uint8_t frame[32];
  frame[0] = 0x20; // Packet Length = 32
  frame[1] = 0x40; // Command Code = Channel Data

  uint16_t chksum = 0xFFFF - frame[0] - frame[1];
  for (int i = 0; i < 14; i++) {
    uint16_t val = constrain(ibusCh[i], (uint16_t)1000, (uint16_t)2000);
    uint8_t lo = val & 0xFF;
    uint8_t hi = (val >> 8) & 0xFF;
    frame[2 + i * 2] = lo;
    frame[3 + i * 2] = hi;
    chksum -= lo;
    chksum -= hi;
  }

  frame[30] = chksum & 0xFF;         // LSB
  frame[31] = (chksum >> 8) & 0xFF;  // MSB

  Serial2.write(frame, 32);
}

void pollFlyskyReceiverUART2() {
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    if (flyskyRxIdx == 0) {
      if (b == 0x20) flyskyRxBuf[flyskyRxIdx++] = b;
    } else if (flyskyRxIdx == 1) {
      if (b == 0x40) flyskyRxBuf[flyskyRxIdx++] = b;
      else flyskyRxIdx = (b == 0x20) ? 1 : 0;
    } else {
      flyskyRxBuf[flyskyRxIdx++] = b;
      if (flyskyRxIdx == 32) {
        uint16_t calcSum = 0xFFFF;
        for (int i = 0; i < 30; i++) calcSum -= flyskyRxBuf[i];
        uint16_t rxSum = (uint16_t)flyskyRxBuf[30] | ((uint16_t)flyskyRxBuf[31] << 8);
        if (calcSum == rxSum) {
          for (int ch = 0; ch < 14; ch++) {
            uint16_t v = (uint16_t)flyskyRxBuf[2 + ch * 2] | ((uint16_t)flyskyRxBuf[3 + ch * 2] << 8);
            if (v >= 950 && v <= 2050) {
              ibusCh[ch] = constrain(v, (uint16_t)1000, (uint16_t)2000);
            }
          }
          lastFlyskyFrameMs = millis();
        }
        flyskyRxIdx = 0;
      }
    }
  }
}

void updateDriveControlAndIbus() {
  uint32_t now = millis();
  if (now - lastIbusTxMs < IBUS_TX_INTERVAL_MS) return;
  lastIbusTxMs = now;

  bool flyskyActive = (lastFlyskyFrameMs > 0) && ((now - lastFlyskyFrameMs) < FLYSKY_ACTIVE_WINDOW);
  bool s3OverrideActive = (lastS3DriveMs > 0) && ((now - lastS3DriveMs) < WEB_DRIVE_TIMEOUT_MS);

  if (flyskyActive) {
    // Hardware FlySky i6X Receiver takes priority
    activeDriveMode = "FLYSKY_RC";
  } else if ((webSteerPct != 0 || webThrottlePct != 0) && (now - lastWebDriveMs < WEB_DRIVE_TIMEOUT_MS)) {
    // Master Web Dashboard Drive
    ibusCh[0] = mapPctToIbus(webSteerPct);     // CH1: Steering
    ibusCh[1] = mapPctToIbus(webThrottlePct);  // CH2: Throttle
    ibusCh[2] = 1500;
    ibusCh[3] = 1500;
    ibusCh[4] = mapSpeedPctToIbus(webSpeedPct); // CH5: Master Speed Limit
    for (int i = 5; i < 14; i++) ibusCh[i] = 1500;
    activeDriveMode = "WIFI_DRV";
  } else if (s3OverrideActive) {
    // Optional S3-CAM Web UI Drive Injection
    ibusCh[0] = constrain(s3DriveCh1, (uint16_t)1000, (uint16_t)2000);
    ibusCh[1] = constrain(s3DriveCh2, (uint16_t)1000, (uint16_t)2000);
    ibusCh[2] = 1500;
    ibusCh[3] = 1500;
    ibusCh[4] = constrain(s3DriveCh5, (uint16_t)1000, (uint16_t)2000);
    for (int i = 5; i < 14; i++) ibusCh[i] = 1500;
    activeDriveMode = "S3_DRV";
  } else {
    // Neutral Failsafe Stop
    webSteerPct = 0;
    webThrottlePct = 0;
    ibusCh[0] = 1500;
    ibusCh[1] = 1500;
    ibusCh[2] = 1500;
    ibusCh[3] = 1500;
    ibusCh[4] = mapSpeedPctToIbus(webSpeedPct);
    for (int i = 5; i < 14; i++) ibusCh[i] = 1500;
    activeDriveMode = "WIFI_STOP";
  }

  sendIbusFrameToHyperFlux();
}

// ------------------------- 4x HC-SR04 SIMULTANEOUS CAPTURE -------------------------
void updateFourSonars() {
  uint32_t now = millis();
  if (now - lastSonarMs < SONAR_INTERVAL_MS) return;
  lastSonarMs = now;

  const int echoPins[4] = {
    PIN_SONAR_FRONT_ECHO,
    PIN_SONAR_REAR_ECHO,
    PIN_SONAR_LEFT_ECHO,
    PIN_SONAR_RIGHT_ECHO
  };

  // Single 10us pulse on Common TRIG Bus (GPIO 27)
  digitalWrite(PIN_SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);

  uint32_t startRise[4] = {0, 0, 0, 0};
  uint32_t pulseWidth[4] = {0, 0, 0, 0};
  uint8_t state[4] = {0, 0, 0, 0}; // 0=wait high, 1=wait low, 2=done

  uint32_t t0 = micros();
  const uint32_t MAX_WAIT_US = 18000UL; // ~310 cm max range

  while ((micros() - t0) < MAX_WAIT_US) {
    bool allDone = true;
    uint32_t cur = micros();
    for (int i = 0; i < 4; i++) {
      if (state[i] == 2) continue;
      allDone = false;
      int level = digitalRead(echoPins[i]);
      if (state[i] == 0 && level == HIGH) {
        startRise[i] = cur;
        state[i] = 1;
      } else if (state[i] == 1 && level == LOW) {
        pulseWidth[i] = cur - startRise[i];
        state[i] = 2;
      }
    }
    if (allDone) break;
  }

  for (int i = 0; i < 4; i++) {
    if (state[i] == 2 && pulseWidth[i] > 100 && pulseWidth[i] < MAX_WAIT_US) {
      sonarDist[i] = (pulseWidth[i] * 0.0343f) * 0.5f;
    } else {
      sonarDist[i] = -1.0f;
    }
  }
}

// ------------------------- MPU6050 6-AXIS IMU (400 kHz) -------------------------
bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadRegs(uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddress, (int)len, (int)true) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void initMPU6050() {
  uint8_t who = 0;
  mpuAddress = 0x68;
  if (!mpuReadRegs(0x75, &who, 1)) {
    mpuAddress = 0x69;
    if (!mpuReadRegs(0x75, &who, 1)) {
      mpuReady = false;
      return;
    }
  }
  bool ok = true;
  ok &= mpuWriteReg(0x6B, 0x00); // Wake up
  ok &= mpuWriteReg(0x1B, 0x00); // Gyro +/- 250 deg/s
  ok &= mpuWriteReg(0x1C, 0x00); // Accel +/- 2g
  ok &= mpuWriteReg(0x1A, 0x03); // DLPF 44Hz
  mpuReady = ok;
  lastImuMicros = micros();
}

void updateMPU6050() {
  uint32_t now = millis();
  if (!mpuReady || (now - lastImuMs < IMU_INTERVAL_MS)) return;
  lastImuMs = now;

  uint8_t b[14];
  if (!mpuReadRegs(0x3B, b, sizeof(b))) {
    mpuReady = false;
    return;
  }

  uint32_t curUs = micros();
  float dt = (curUs - lastImuMicros) / 1000000.0f;
  lastImuMicros = curUs;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

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
  mpuTempC = (tr / 340.0f) + 36.53f;
  gyroX = gx / 131.0f;
  gyroY = gy / 131.0f;
  gyroZ = gz / 131.0f;

  float accRoll = atan2f(accelY, accelZ) * 57.29578f;
  float accPitch = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * 57.29578f;

  // Complementary filter for smooth Roll & Pitch
  rollDeg = 0.96f * (rollDeg + gyroX * dt) + 0.04f * accRoll;
  pitchDeg = 0.96f * (pitchDeg + gyroY * dt) + 0.04f * accPitch;
}

// ------------------------- INTER-BOARD UART1 PROTOCOL (250000 BAUD) -------------------------
int parseKeyInt(const String &line, const char *key, int fallback) {
  int idx = line.indexOf(key);
  if (idx < 0) return fallback;
  idx += strlen(key);
  return line.substring(idx).toInt();
}

String parseKeyToken(const String &line, const char *key) {
  int idx = line.indexOf(key);
  if (idx < 0) return "";
  idx += strlen(key);
  int end = line.indexOf(',', idx);
  String val = (end < 0) ? line.substring(idx) : line.substring(idx, end);
  val.trim();
  return val;
}

void processS3Uart1Line(const String &line) {
  lastS3HeartbeatMs = millis();
  s3LastRawStatus = line;

  if (line.startsWith("S3_OK")) {
    s3LastJpegBytes = (uint32_t)parseKeyInt(line, "CAM:", 0);
    s3LastMicRms = parseKeyInt(line, "MIC_RMS:", 0);
    String ipTok = parseKeyToken(line, "IP:");
    if (ipTok.length() > 6 && ipTok != "0.0.0.0" && savedCamIp.length() == 0) {
      savedCamIp = ipTok;
    }
  } else if (line.startsWith("DRV,")) {
    // Optional Web Drive Override from S3-CAM: DRV, CH1:steer, CH2:thr, CH5:spd
    s3DriveCh1 = (uint16_t)parseKeyInt(line, "CH1:", 1500);
    s3DriveCh2 = (uint16_t)parseKeyInt(line, "CH2:", 1500);
    s3DriveCh5 = (uint16_t)parseKeyInt(line, "CH5:", 1500);
    lastS3DriveMs = millis();
  }
}

void serviceS3Uart1() {
  // Read incoming frames from Board B (GPIO 18)
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      s3RxLine.trim();
      if (s3RxLine.length() > 0) processS3Uart1Line(s3RxLine);
      s3RxLine = "";
    } else if (c != '\r') {
      if (s3RxLine.length() < 200) s3RxLine += c;
      else s3RxLine = "";
    }
  }

  // Transmit 50 Hz Telemetry Stream to Board B (GPIO 19):
  // TLM, S:s1/s2/s3/s4, R:roll, P:pitch, M:mode\n
  uint32_t now = millis();
  if (now - lastS3TlmMs >= S3_TLM_INTERVAL_MS) {
    lastS3TlmMs = now;
    char tlmBuf[128];
    snprintf(
      tlmBuf, sizeof(tlmBuf),
      "TLM, S:%.1f/%.1f/%.1f/%.1f, R:%.1f, P:%.1f, M:%s\n",
      sonarDist[0], sonarDist[1], sonarDist[2], sonarDist[3],
      rollDeg, pitchDeg, activeDriveMode.c_str()
    );
    Serial1.print(tlmBuf);
  }
}

// ------------------------- JSON & WEB HANDLERS -------------------------
String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') out += '\\';
    if (c != '\n' && c != '\r') out += c;
  }
  return out;
}

String buildStateJson() {
  bool s3Online = (lastS3HeartbeatMs > 0) && ((millis() - lastS3HeartbeatMs) < 2000UL);
  String ip = (wifiModeStr == "STA") ? WiFi.localIP().toString() : WiFi.softAPIP().toString();

  String j = "{";
  j += "\"wifiMode\":\"" + wifiModeStr + "\",";
  j += "\"ip\":\"" + ip + "\",";
  j += "\"mode\":\"" + activeDriveMode + "\",";
  j += "\"ch1\":" + String(ibusCh[0]) + ",";
  j += "\"ch2\":" + String(ibusCh[1]) + ",";
  j += "\"ch5\":" + String(ibusCh[4]) + ",";
  j += "\"s1\":" + String(sonarDist[0], 1) + ",";
  j += "\"s2\":" + String(sonarDist[1], 1) + ",";
  j += "\"s3\":" + String(sonarDist[2], 1) + ",";
  j += "\"s4\":" + String(sonarDist[3], 1) + ",";
  j += "\"mpuReady\":" + String(mpuReady ? "true" : "false") + ",";
  j += "\"roll\":" + String(rollDeg, 1) + ",";
  j += "\"pitch\":" + String(pitchDeg, 1) + ",";
  j += "\"ax\":" + String(accelX, 2) + ",";
  j += "\"ay\":" + String(accelY, 2) + ",";
  j += "\"az\":" + String(accelZ, 2) + ",";
  j += "\"gx\":" + String(gyroX, 1) + ",";
  j += "\"gy\":" + String(gyroY, 1) + ",";
  j += "\"gz\":" + String(gyroZ, 1) + ",";
  j += "\"s3Online\":" + String(s3Online ? "true" : "false") + ",";
  j += "\"s3CamBytes\":" + String(s3LastJpegBytes) + ",";
  j += "\"s3MicRms\":" + String(s3LastMicRms) + ",";
  j += "\"savedCamIp\":\"" + jsonEscape(savedCamIp) + "\"";
  j += "}";
  return j;
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  (void)payload;
  (void)len;
  if (type == WStype_CONNECTED) {
    String msg = buildStateJson();
    webSocket.sendTXT(num, msg);
  } else if (type == WStype_DISCONNECTED) {
    if (webSocket.connectedClients() == 0) {
      webSteerPct = 0;
      webThrottlePct = 0;
    }
  }
}

void setupWebAndNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting Board A to Wi-Fi: ");
  Serial.println(WIFI_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0 < 12000)) {
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiModeStr = "STA";
    digitalWrite(PIN_STATUS_LED, HIGH);
    Serial.print("Board A Connected! Dashboard IP: http://");
    Serial.println(WiFi.localIP());
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    wifiModeStr = "AP";
    digitalWrite(PIN_STATUS_LED, HIGH);
    Serial.print("Board A AP Mode! Dashboard IP: http://");
    Serial.println(WiFi.softAPIP());
  }

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
  });

  server.on("/api/state", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buildStateJson());
  });

  server.on("/api/drive", HTTP_GET, []() {
    webThrottlePct = constrain(server.arg("thr").toInt(), -100, 100);
    webSteerPct    = constrain(server.arg("str").toInt(), -100, 100);
    if (server.hasArg("spd")) {
      webSpeedPct = constrain(server.arg("spd").toInt(), 5, 100);
    }
    lastWebDriveMs = millis();
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/speed", HTTP_GET, []() {
    if (server.hasArg("spd")) {
      webSpeedPct = constrain(server.arg("spd").toInt(), 5, 100);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/stop", HTTP_GET, []() {
    webSteerPct = 0;
    webThrottlePct = 0;
    lastS3DriveMs = 0;
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/cam/setip", HTTP_GET, []() {
    String ip = server.arg("ip");
    ip.trim();
    if (ip.length() > 0) {
      savedCamIp = ip;
      prefs.begin("rover_a", false);
      prefs.putString("s3ip", savedCamIp);
      prefs.end();
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/cam/status", HTTP_GET, []() {
    if (savedCamIp.length() == 0) {
      server.send(200, "application/json", "{\"ok\":false}");
      return;
    }
    HTTPClient http;
    http.setConnectTimeout(300);
    http.setTimeout(450);
    http.begin("http://" + savedCamIp + "/api/status");
    int code = http.GET();
    String body = (code == 200) ? http.getString() : "{\"ok\":false}";
    http.end();
    server.send(200, "application/json", body);
  });

  server.on("/api/cam/cmd", HTTP_GET, []() {
    String path = server.arg("path");
    if (savedCamIp.length() == 0 || path.length() == 0) {
      server.send(200, "application/json", "{\"ok\":false}");
      return;
    }
    HTTPClient http;
    http.setConnectTimeout(300);
    http.setTimeout(450);
    http.begin("http://" + savedCamIp + path);
    int code = http.GET();
    String body = (code == 200) ? http.getString() : "{\"ok\":false}";
    http.end();
    server.send(200, "application/json", body);
  });

  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  networkReady = true;
}

// ------------------------- SETUP & LOOP -------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== Rescue Rover Board A (Main Deck) Booting ===");

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  prefs.begin("rover_a", true);
  savedCamIp = prefs.getString("s3ip", "");
  prefs.end();

  // 4x HC-SR04 Pin Setup (Note: GPIO 34, 35, 36, 39 untouched)
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  pinMode(PIN_SONAR_FRONT_ECHO, INPUT);
  pinMode(PIN_SONAR_REAR_ECHO, INPUT);
  pinMode(PIN_SONAR_LEFT_ECHO, INPUT);
  pinMode(PIN_SONAR_RIGHT_ECHO, INPUT);

  // MPU6050 I2C @ 400 kHz
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  initMPU6050();

  // UART1 to Board B (S3-CAM) @ 250000 baud: RX1=18, TX1=19
  Serial1.begin(S3_UART_BAUD, SERIAL_8N1, PIN_S3_UART1_RX, PIN_S3_UART1_TX);

  // UART2 to HyperFlux (TX2=17) & FlySky i6X (RX2=16) @ 115200 baud
  Serial2.begin(IBUS_BAUD, SERIAL_8N1, PIN_IBUS_RX2, PIN_IBUS_TX2);

  setupWebAndNetwork();
}

void loop() {
  if (networkReady) {
    server.handleClient();
    webSocket.loop();
  }

  pollFlyskyReceiverUART2();
  updateDriveControlAndIbus();
  updateMPU6050();
  updateFourSonars();
  serviceS3Uart1();

  if (networkReady && (millis() - lastWsMs >= WS_BROADCAST_MS)) {
    lastWsMs = millis();
    String stateJson = buildStateJson();
    webSocket.broadcastTXT(stateJson);
  }

  delay(1);
}