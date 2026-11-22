/*
  =============================================================================
  Rescue Rover - Board B: Media Deck Firmware (Instant Zero-Latency Stream)
  MCU: ESP32-S3-CAM (44-Pin N16R8 DevKit)
  - Low-latency MJPEG Stream Server on Port 81 (/stream) with TCP_NODELAY
  - Dynamic Resolution API (/api/cam/cfg?res=qvga|hvga|vga&q=13) for QR & Speed
  - Shared-Clock Full-Duplex I2S (16 kHz): WS=42, BCLK=41, DOUT=40, DIN=39
  - Inter-Board UART1 @ 250000 baud: TX1=1, RX1=2
  =============================================================================
*/

#include "esp_camera.h"
#include "esp_wifi.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <math.h>
#include "driver/i2s_std.h"

// ------------------------- WIFI SETTINGS -------------------------
static const char *DEFAULT_STA_SSID = "OPPO_A5s";
static const char *DEFAULT_STA_PASS = "25802580";
static const char *AP_FALLBACK_SSID = "ESP32_S3_CAM_PRO";
static const char *AP_FALLBACK_PASS = "12345678";

// ------------------------- BOARD B PINOUT (REV 1.0 SPEC) -------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

static const int PIN_UART1_TX = 1;   // Pin 26: GPIO 1 -> Board A D18 (RX1)
static const int PIN_UART1_RX = 2;   // Pin 27: GPIO 2 <- Board A D19 (TX1)
static const uint32_t UART1_BAUD = 250000;

static const int PIN_I2S_WS   = 42;  // Pin 28: Shared LRC / WS
static const int PIN_I2S_BCLK = 41;  // Pin 29: Shared BCLK / SCK
static const int PIN_I2S_DOUT = 40;  // Pin 30: Speaker Out -> MAX98357A DIN
static const int PIN_I2S_DIN  = 39;  // Pin 31: Mic In <- CU20BK40 SD/DOUT
static const uint32_t AUDIO_RATE = 16000;

static const int PIN_RGB_LED = 48;

// ------------------------- GLOBAL STATE -------------------------
WebServer webServer(80);
WiFiServer streamServer(81);
Preferences prefs;

bool cameraReady = false;
bool i2sReady = false;
volatile bool feedEnabled = true;
volatile float currentFps = 0.0f;
volatile float currentKbps = 0.0f;
volatile size_t lastFrameBytes = 0;
volatile int micRmsLevel = 0;
int currentQuality = 13;
String currentResName = "HVGA";

String wifiModeStr = "BOOTING";
String activeIpStr = "";
String uartRxLine = "";
String latestBoardATlm = "Waiting for Board A TLM...";
uint32_t lastHeartbeatTxMs = 0;

i2s_chan_handle_t txChan = nullptr;
i2s_chan_handle_t rxChan = nullptr;

// ------------------------- RGB LED HELPER -------------------------
void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  rgbLedWrite(PIN_RGB_LED, r, g, b);
#else
  neopixelWrite(PIN_RGB_LED, r, g, b);
#endif
}

// ------------------------- SHARED-CLOCK FULL-DUPLEX I2S -------------------------
bool initSharedClockI2S() {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.dma_desc_num = 6;
  chanCfg.dma_frame_num = 320;

  if (i2s_new_channel(&chanCfg, &txChan, &rxChan) != ESP_OK) return false;

  i2s_std_config_t stdCfg = {};
  stdCfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE);
  stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  stdCfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  stdCfg.gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  stdCfg.gpio_cfg.ws   = (gpio_num_t)PIN_I2S_WS;
  stdCfg.gpio_cfg.dout = (gpio_num_t)PIN_I2S_DOUT;
  stdCfg.gpio_cfg.din  = (gpio_num_t)PIN_I2S_DIN;

  if (i2s_channel_init_std_mode(txChan, &stdCfg) != ESP_OK) return false;
  if (i2s_channel_init_std_mode(rxChan, &stdCfg) != ESP_OK) return false;
  if (i2s_channel_enable(txChan) != ESP_OK) return false;
  if (i2s_channel_enable(rxChan) != ESP_OK) return false;
  return true;
}

void sampleMicRmsBackground() {
  static uint32_t lastRmsSampleMs = 0;
  if (!i2sReady || rxChan == nullptr || (millis() - lastRmsSampleMs < 150)) return;
  lastRmsSampleMs = millis();

  int32_t rawBuf[128];
  size_t bytesRead = 0;
  if (i2s_channel_read(rxChan, rawBuf, sizeof(rawBuf), &bytesRead, 5) == ESP_OK && bytesRead > 0) {
    size_t frames = bytesRead / (sizeof(int32_t) * 2);
    double sumSq = 0.0;
    for (size_t i = 0; i < frames; i++) {
      int16_t s = (int16_t)(rawBuf[i * 2] >> 16);
      sumSq += (double)s * (double)s;
    }
    if (frames > 0) {
      micRmsLevel = (int)sqrt(sumSq / (double)frames);
    }
  }
}

void playChimeTone() {
  if (!i2sReady || txChan == nullptr) return;
  const float notes[] = {880.0f, 1174.66f};
  int32_t stereoBuf[320];
  for (int n = 0; n < 2; n++) {
    float freq = notes[n];
    for (int block = 0; block < 6; block++) {
      for (int i = 0; i < 160; i++) {
        float t = (float)(block * 160 + i) / (float)AUDIO_RATE;
        int16_t s16 = (int16_t)(sinf(2.0f * PI * freq * t) * 9000.0f);
        int32_t s32 = ((int32_t)s16) << 16;
        stereoBuf[i * 2] = s32;
        stereoBuf[i * 2 + 1] = s32;
      }
      size_t written = 0;
      i2s_channel_write(txChan, stereoBuf, sizeof(stereoBuf), &written, 50);
    }
  }
}

// ------------------------- CAMERA INIT -------------------------
bool initS3Camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    // HVGA (480x320) gives instant low-latency stream + crisp QR code edges
    config.frame_size = FRAMESIZE_HVGA;
    config.jpeg_quality = 13;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    currentResName = "HVGA";
    currentQuality = 13;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    currentResName = "QVGA";
    currentQuality = 15;
  }

  if (esp_camera_init(&config) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s != nullptr) {
    s->set_contrast(s, 1);   // Boost contrast slightly for sharp QR detection
    s->set_sharpness(s, 1);  // Boost edge sharpness
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1);
    }
  }
  return true;
}

// ------------------------- PORT 81 INSTANT MJPEG STREAM TASK -------------------------
void mjpegStreamTask(void *param) {
  (void)param;
  streamServer.begin();
  streamServer.setNoDelay(true);

  uint32_t lastIdleSampleMs = 0;

  for (;;) {
    WiFiClient client = streamServer.available();
    if (!client) {
      if (cameraReady && feedEnabled && (millis() - lastIdleSampleMs >= 600)) {
        lastIdleSampleMs = millis();
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
          lastFrameBytes = fb->len;
          esp_camera_fb_return(fb);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Disable Nagle algorithm for zero-delay frame delivery
    client.setNoDelay(true);
    client.setTimeout(1);

    uint32_t reqStart = millis();
    while (client.connected() && (millis() - reqStart < 800)) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() == 0) break;
      }
      vTaskDelay(1);
    }

    client.print(
      "HTTP/1.1 200 OK\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-store, no-cache, must-revalidate\r\n"
      "Pragma: no-cache\r\n\r\n"
    );

    uint32_t frameCounter = 0;
    size_t byteCounter = 0;
    uint32_t statWindowStart = millis();

    while (client.connected()) {
      if (!feedEnabled || !cameraReady) {
        currentFps = 0.0f;
        currentKbps = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }

      lastFrameBytes = fb->len;

      char hdr[112];
      int hLen = snprintf(
        hdr, sizeof(hdr),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        (unsigned int)fb->len
      );

      size_t w1 = client.write((const uint8_t *)hdr, hLen);
      size_t w2 = (w1 > 0) ? client.write(fb->buf, fb->len) : 0;
      size_t w3 = (w2 > 0) ? client.write((const uint8_t *)"\r\n", 2) : 0;

      esp_camera_fb_return(fb);

      if (w1 == 0 || w2 == 0 || w3 == 0) {
        break;
      }

      frameCounter++;
      byteCounter += lastFrameBytes;

      uint32_t now = millis();
      if (now - statWindowStart >= 1000) {
        float sec = (now - statWindowStart) / 1000.0f;
        currentFps = frameCounter / sec;
        currentKbps = (byteCounter * 8.0f) / (sec * 1000.0f);
        frameCounter = 0;
        byteCounter = 0;
        statWindowStart = now;
      }
      taskYIELD();
    }

    client.stop();
    currentFps = 0.0f;
    currentKbps = 0.0f;
  }
}

// ------------------------- PORT 80 WEB & API HANDLERS -------------------------
void sendCorsHeaders() {
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.sendHeader("Cache-Control", "no-store");
}

void handleS3Root() {
  sendCorsHeaders();
  String html = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>ESP32-S3-CAM Media Deck</title>"
                "<style>body{background:#0f172a;color:#f8fafc;font-family:system-ui;padding:16px;max-width:640px;margin:auto}"
                "img{width:100%;border-radius:10px;border:1px solid #334155;background:#000}"
                "input,button{padding:10px;border-radius:8px;border:1px solid #334155;margin:4px 0;width:100%}"
                "button{background:#2563eb;color:#fff;font-weight:700;border:0;cursor:pointer}</style></head><body>"
                "<h2>ESP32-S3-CAM Media Deck (Board B)</h2>"
                "<p>IP: <b>" + activeIpStr + "</b> (" + wifiModeStr + ")</p>"
                "<img id='v'>"
                "<p style='font-size:13px;color:#94a3b8'>Board A Telemetry: " + latestBoardATlm + "</p>"
                "<script>document.getElementById('v').src='http://'+location.hostname+':81/stream';</script>"
                "<h3>Wi-Fi Settings</h3><form action='/api/wifi' method='POST'>"
                "<input name='ssid' placeholder='Wi-Fi SSID' required>"
                "<input name='pass' type='password' placeholder='Wi-Fi Password'>"
                "<button type='submit'>Save Wi-Fi & Reboot</button></form></body></html>";
  webServer.send(200, "text/html", html);
}

void handleS3Status() {
  sendCorsHeaders();
  String json = "{";
  json += "\"ok\":true,";
  json += "\"cameraReady\":" + String(cameraReady ? "true" : "false") + ",";
  json += "\"feed\":" + String(feedEnabled ? "true" : "false") + ",";
  json += "\"fps\":" + String(currentFps, 1) + ",";
  json += "\"kbps\":" + String(currentKbps, 0) + ",";
  json += "\"jpegBytes\":" + String(lastFrameBytes) + ",";
  json += "\"res\":\"" + currentResName + "\",";
  json += "\"quality\":" + String(currentQuality) + ",";
  json += "\"micRms\":" + String(micRmsLevel) + ",";
  json += "\"wifiMode\":\"" + wifiModeStr + "\",";
  json += "\"ip\":\"" + activeIpStr + "\"";
  json += "}";
  webServer.send(200, "application/json", json);
}

void handleS3CamCfg() {
  sendCorsHeaders();
  sensor_t *s = esp_camera_sensor_get();
  if (s != nullptr) {
    if (webServer.hasArg("res")) {
      String r = webServer.arg("res");
      r.toLowerCase();
      if (r == "qvga") {
        s->set_framesize(s, FRAMESIZE_QVGA);
        currentResName = "QVGA";
      } else if (r == "vga") {
        s->set_framesize(s, FRAMESIZE_VGA);
        currentResName = "VGA";
      } else {
        s->set_framesize(s, FRAMESIZE_HVGA);
        currentResName = "HVGA";
      }
    }
    if (webServer.hasArg("q")) {
      int q = constrain(webServer.arg("q").toInt(), 8, 30);
      s->set_quality(s, q);
      currentQuality = q;
    }
  }
  webServer.send(200, "application/json", "{\"ok\":true,\"res\":\"" + currentResName + "\"}");
}

void handleS3Feed() {
  sendCorsHeaders();
  if (webServer.hasArg("on")) {
    feedEnabled = (webServer.arg("on").toInt() != 0);
  }
  webServer.send(200, "application/json", "{\"ok\":true,\"feed\":" + String(feedEnabled ? "true" : "false") + "}");
}

void handleS3Auto25() {
  sendCorsHeaders();
  sensor_t *s = esp_camera_sensor_get();
  if (s != nullptr) {
    s->set_framesize(s, FRAMESIZE_HVGA);
    s->set_quality(s, 13);
    currentResName = "HVGA";
    currentQuality = 13;
  }
  webServer.send(200, "application/json", "{\"ok\":true,\"mode\":\"HVGA_FAST\"}");
}

void handleS3Chime() {
  sendCorsHeaders();
  playChimeTone();
  webServer.send(200, "application/json", "{\"ok\":true,\"chime\":true}");
}

void handleS3DrvInject() {
  sendCorsHeaders();
  int ch1 = webServer.hasArg("ch1") ? constrain(webServer.arg("ch1").toInt(), 1000, 2000) : 1500;
  int ch2 = webServer.hasArg("ch2") ? constrain(webServer.arg("ch2").toInt(), 1000, 2000) : 1500;
  int ch5 = webServer.hasArg("ch5") ? constrain(webServer.arg("ch5").toInt(), 1000, 2000) : 1500;
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "DRV, CH1:%d, CH2:%d, CH5:%d\n", ch1, ch2, ch5);
  Serial1.print(cmd);
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleS3AudioRx() {
  sendCorsHeaders();
  if (!i2sReady || rxChan == nullptr) {
    webServer.send(200, "application/octet-stream", "");
    return;
  }
  int32_t rawBuf[320];
  int16_t pcmOut[160];
  size_t bytesRead = 0;
  if (i2s_channel_read(rxChan, rawBuf, sizeof(rawBuf), &bytesRead, 25) == ESP_OK && bytesRead > 0) {
    size_t frames = bytesRead / (sizeof(int32_t) * 2);
    double sumSq = 0.0;
    for (size_t i = 0; i < frames; i++) {
      int16_t sample = (int16_t)(rawBuf[i * 2] >> 16);
      pcmOut[i] = sample;
      sumSq += (double)sample * (double)sample;
    }
    if (frames > 0) {
      micRmsLevel = (int)sqrt(sumSq / (double)frames);
    }
    webServer.send_P(200, "application/octet-stream", (const char *)pcmOut, frames * sizeof(int16_t));
  } else {
    webServer.send(200, "application/octet-stream", "");
  }
}

void handleS3AudioTx() {
  sendCorsHeaders();
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleS3AudioUpload() {
  HTTPUpload &up = webServer.upload();
  if (up.status == UPLOAD_FILE_WRITE && i2sReady && txChan != nullptr && up.currentSize >= 2) {
    size_t samples = up.currentSize / 2;
    const int16_t *in16 = (const int16_t *)up.buf;
    int32_t out32[256];
    size_t idx = 0;
    while (idx < samples) {
      size_t chunk = min((size_t)128, samples - idx);
      for (size_t i = 0; i < chunk; i++) {
        int32_t s = ((int32_t)in16[idx + i]) << 16;
        out32[i * 2] = s;
        out32[i * 2 + 1] = s;
      }
      size_t written = 0;
      i2s_channel_write(txChan, out32, chunk * 2 * sizeof(int32_t), &written, 20);
      idx += chunk;
    }
  }
}

void handleS3WifiSave() {
  String s = webServer.arg("ssid");
  String p = webServer.arg("pass");
  s.trim();
  if (s.length() > 0) {
    prefs.begin("s3_wifi", false);
    prefs.putString("ssid", s);
    prefs.putString("pass", p);
    prefs.end();
    webServer.send(200, "text/plain", "Saved! Rebooting ESP32-S3-CAM...");
    delay(600);
    ESP.restart();
  } else {
    webServer.send(400, "text/plain", "SSID required");
  }
}

// ------------------------- UART1 PROTOCOL (250000 BAUD) -------------------------
void serviceBoardAUart1() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      uartRxLine.trim();
      if (uartRxLine.startsWith("TLM,")) {
        latestBoardATlm = uartRxLine;
      }
      uartRxLine = "";
    } else if (c != '\r') {
      if (uartRxLine.length() < 160) uartRxLine += c;
      else uartRxLine = "";
    }
  }

  uint32_t now = millis();
  if (now - lastHeartbeatTxMs >= 300) {
    lastHeartbeatTxMs = now;
    char pkt[128];
    snprintf(
      pkt, sizeof(pkt),
      "S3_OK, CAM:%uB, MIC_RMS:%d, IP:%s\n",
      (unsigned int)lastFrameBytes, micRmsLevel, activeIpStr.c_str()
    );
    Serial1.print(pkt);
  }
}

// ------------------------- WIFI & SETUP -------------------------
void connectS3WiFi() {
  prefs.begin("s3_wifi", true);
  String ssid = prefs.getString("ssid", DEFAULT_STA_SSID);
  String pass = prefs.getString("pass", DEFAULT_STA_PASS);
  prefs.end();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(ssid.c_str(), pass.c_str());

  Serial.print("Connecting S3-CAM to Wi-Fi: ");
  Serial.println(ssid);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs < 12000)) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiModeStr = "STA";
    activeIpStr = WiFi.localIP().toString();
    setStatusLed(0, 40, 0);
    MDNS.begin("esp32cam");
    Serial.print("S3-CAM STA Connected! IP: ");
    Serial.println(activeIpStr);
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_FALLBACK_SSID, AP_FALLBACK_PASS);
    wifiModeStr = "AP";
    activeIpStr = WiFi.softAPIP().toString();
    setStatusLed(0, 0, 50);
    Serial.print("S3-CAM AP Fallback Started! IP: ");
    Serial.println(activeIpStr);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32-S3-CAM Media Deck (Board B) Booting ===");

  Serial1.begin(UART1_BAUD, SERIAL_8N1, PIN_UART1_RX, PIN_UART1_TX);

  cameraReady = initS3Camera();
  if (!cameraReady) {
    setStatusLed(50, 0, 0);
  }

  i2sReady = initSharedClockI2S();
  connectS3WiFi();

  webServer.on("/", HTTP_GET, handleS3Root);
  webServer.on("/api/status", HTTP_GET, handleS3Status);
  webServer.on("/api/cam/cfg", HTTP_GET, handleS3CamCfg);
  webServer.on("/api/feed", HTTP_GET, handleS3Feed);
  webServer.on("/api/auto25", HTTP_GET, handleS3Auto25);
  webServer.on("/api/drv", HTTP_GET, handleS3DrvInject);
  webServer.on("/api/audio/chime", HTTP_GET, handleS3Chime);
  webServer.on("/api/audio/rx", HTTP_GET, handleS3AudioRx);
  webServer.on("/api/audio/tx", HTTP_POST, handleS3AudioTx, handleS3AudioUpload);
  webServer.on("/api/wifi", HTTP_POST, handleS3WifiSave);
  webServer.begin();

  // Pin MJPEG stream task to Core 0 with high priority for instant video delivery
  xTaskCreatePinnedToCore(mjpegStreamTask, "mjpeg_stream", 4096, nullptr, 3, nullptr, 0);
  Serial.println("Board B Media Deck Ready.");
}

void loop() {
  webServer.handleClient();
  sampleMicRmsBackground();
  serviceBoardAUart1();
  delay(1);
}