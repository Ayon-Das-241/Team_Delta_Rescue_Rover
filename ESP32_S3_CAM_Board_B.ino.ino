/*
  =============================================================================
  Rescue Rover - ESP32-S3-CAM (Board B) Complete Standalone Firmware
  Board: ESP32S3 Dev Module (PSRAM: OPI PSRAM)
  Features:
  - Self-contained MJPEG Stream Server (Port 81 /stream) - No external files!
  - Web API & CORS-enabled Status/Control Server (Port 80)
  - Two-Way 16kHz I2S Audio Intercom (/api/audio/rx, /api/audio/tx, chime)
  - UART1 Bridge on TX1=GPIO 1, RX1=GPIO 2 (PING, CHIME, STATUS & Auto-IP sync)
  - Onboard RGB LED (GPIO 48): Green=Wi-Fi Connected, Blue=AP Failover, Red=Error
  =============================================================================
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <math.h>
#include "driver/i2s_std.h"

// ------------------------- DEFAULT WIFI SETTINGS -------------------------
static const char *DEFAULT_STA_SSID = "OPPO_A5s";
static const char *DEFAULT_STA_PASS = "25802580";
static const char *AP_FALLBACK_SSID = "ESP32_S3_CAM_PRO";
static const char *AP_FALLBACK_PASS = "12345678";

// ------------------------- BOARD B PINOUT -------------------------
// Standard ESP32-S3-CAM (OV2640 / OV3660) Camera Pins
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

// UART1 to Main ESP32 (TX1=GPIO 1 -> Main RX 13, RX1=GPIO 2 <- Main TX 14)
static const int PIN_UART1_TX = 1;
static const int PIN_UART1_RX = 2;

// Onboard RGB LED
static const int PIN_RGB_LED = 48;

// I2S Audio Pins (Safe non-PSRAM GPIOs on ESP32-S3)
static const int PIN_I2S_BCLK = 42;
static const int PIN_I2S_WS   = 41;
static const int PIN_I2S_DOUT = 40;
static const int PIN_I2S_DIN  = 39;
static const uint32_t AUDIO_RATE = 16000;

// ------------------------- GLOBAL STATE -------------------------
WebServer webServer(80);
WiFiServer streamServer(81);
Preferences prefs;

bool cameraReady = false;
bool i2sReady = false;
volatile bool feedEnabled = true;
volatile float currentFps = 0.0f;
volatile float currentKbps = 0.0f;
int currentQuality = 12;
volatile int micPeakLevel = 0;

String wifiModeStr = "BOOTING";
String activeIpStr = "";
String uartRxBuffer = "";
uint32_t lastIpAnnounceMs = 0;

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

// ------------------------- I2S AUDIO & CHIME -------------------------
bool initS3Audio() {
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

void playChimeTone() {
  if (!i2sReady || txChan == nullptr) return;
  const float notes[] = {880.0f, 1174.66f}; // A5 -> D6 chime
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
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != nullptr) {
    currentQuality = s->status.quality;
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1);
      s->set_saturation(s, -1);
    }
  }
  return true;
}

// ------------------------- PORT 81 MJPEG STREAM TASK -------------------------
void mjpegStreamTask(void *param) {
  (void)param;
  streamServer.begin();

  for (;;) {
    WiFiClient client = streamServer.available();
    if (!client) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Read HTTP request headers
    uint32_t reqStart = millis();
    while (client.connected() && (millis() - reqStart < 1000)) {
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
      "Cache-Control: no-store\r\n\r\n"
    );

    uint32_t frameCounter = 0;
    size_t byteCounter = 0;
    uint32_t statWindowStart = millis();

    while (client.connected()) {
      if (!feedEnabled || !cameraReady) {
        currentFps = 0.0f;
        currentKbps = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(80));
        continue;
      }

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      char hdr[128];
      int hLen = snprintf(
        hdr, sizeof(hdr),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        (unsigned int)fb->len
      );

      if (client.write((const uint8_t *)hdr, hLen) == 0 ||
          client.write(fb->buf, fb->len) == 0 ||
          client.write((const uint8_t *)"\r\n", 2) == 0) {
        esp_camera_fb_return(fb);
        break;
      }

      frameCounter++;
      byteCounter += fb->len;
      esp_camera_fb_return(fb);

      uint32_t now = millis();
      if (now - statWindowStart >= 1000) {
        float sec = (now - statWindowStart) / 1000.0f;
        currentFps = frameCounter / sec;
        currentKbps = (byteCounter * 8.0f) / (sec * 1000.0f);
        frameCounter = 0;
        byteCounter = 0;
        statWindowStart = now;
      }
      vTaskDelay(1);
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
                "<title>ESP32-S3-CAM Board B</title>"
                "<style>body{background:#0f172a;color:#f8fafc;font-family:system-ui;padding:16px;max-width:640px;margin:auto}"
                "img{width:100%;border-radius:10px;border:1px solid #334155;background:#000}"
                "input,button{padding:10px;border-radius:8px;border:1px solid #334155;margin:4px 0;width:100%}"
                "button{background:#2563eb;color:#fff;font-weight:700;border:0;cursor:pointer}</style></head><body>"
                "<h2>ESP32-S3-CAM Live Studio</h2>"
                "<p>IP: <b>" + activeIpStr + "</b> (" + wifiModeStr + ")</p>"
                "<img id='v'>"
                "<script>document.getElementById('v').src='http://'+location.hostname+':81/stream';</script>"
                "<h3>Wi-Fi Manager</h3><form action='/api/wifi' method='POST'>"
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
  json += "\"quality\":" + String(currentQuality) + ",";
  json += "\"micPeak\":" + String(micPeakLevel) + ",";
  json += "\"wifiMode\":\"" + wifiModeStr + "\",";
  json += "\"ip\":\"" + activeIpStr + "\"";
  json += "}";
  webServer.send(200, "application/json", json);
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
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 14);
    currentQuality = 14;
  }
  webServer.send(200, "application/json", "{\"ok\":true,\"mode\":\"AUTO_25_PLUS\"}");
}

void handleS3Chime() {
  sendCorsHeaders();
  playChimeTone();
  webServer.send(200, "application/json", "{\"ok\":true,\"chime\":true}");
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
    int peak = 0;
    for (size_t i = 0; i < frames; i++) {
      int16_t sample = (int16_t)(rawBuf[i * 2] >> 16);
      pcmOut[i] = sample;
      int a = abs((int)sample);
      if (a > peak) peak = a;
    }
    micPeakLevel = peak;
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

// ------------------------- UART1 COMMANDS (TX=1, RX=2) -------------------------
void sendUartStatusPacket() {
  String pkt = "{\"ip\":\"" + activeIpStr + "\",\"wifiMode\":\"" + wifiModeStr + "\",\"fps\":" + String(currentFps, 1) + "}";
  Serial1.println(pkt);
}

void pollMainEspUart() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      uartRxBuffer.trim();
      if (uartRxBuffer == "PING") {
        Serial1.print("PONG\r\n");
        sendUartStatusPacket();
      } else if (uartRxBuffer == "CHIME") {
        playChimeTone();
        Serial1.print("OK: CHIME\r\n");
      } else if (uartRxBuffer == "STATUS") {
        sendUartStatusPacket();
      }
      uartRxBuffer = "";
    } else if (c != '\r') {
      if (uartRxBuffer.length() < 128) uartRxBuffer += c;
      else uartRxBuffer = "";
    }
  }

  // Periodically broadcast IP over UART1 so Main ESP32 auto-discovers S3-CAM IP
  if (millis() - lastIpAnnounceMs >= 3000) {
    lastIpAnnounceMs = millis();
    sendUartStatusPacket();
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
  WiFi.begin(ssid.c_str(), pass.c_str());

  Serial.print("Connecting S3-CAM to Wi-Fi: ");
  Serial.println(ssid);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs < 15000)) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiModeStr = "STA";
    activeIpStr = WiFi.localIP().toString();
    setStatusLed(0, 40, 0); // Green = Connected
    MDNS.begin("esp32cam");
    Serial.print("S3-CAM STA Connected! IP: ");
    Serial.println(activeIpStr);
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_FALLBACK_SSID, AP_FALLBACK_PASS);
    wifiModeStr = "AP";
    activeIpStr = WiFi.softAPIP().toString();
    setStatusLed(0, 0, 50); // Blue = AP Fallback Mode
    Serial.print("S3-CAM AP Fallback Started! IP: ");
    Serial.println(activeIpStr);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== ESP32-S3-CAM Board B Booting ===");

  // UART1 to Main ESP32 (TX=GPIO 1, RX=GPIO 2)
  Serial1.begin(115200, SERIAL_8N1, PIN_UART1_RX, PIN_UART1_TX);

  cameraReady = initS3Camera();
  if (!cameraReady) {
    setStatusLed(50, 0, 0); // Red = Camera Error
  }

  i2sReady = initS3Audio();
  connectS3WiFi();

  webServer.on("/", HTTP_GET, handleS3Root);
  webServer.on("/api/status", HTTP_GET, handleS3Status);
  webServer.on("/api/feed", HTTP_GET, handleS3Feed);
  webServer.on("/api/auto25", HTTP_GET, handleS3Auto25);
  webServer.on("/api/audio/chime", HTTP_GET, handleS3Chime);
  webServer.on("/api/audio/rx", HTTP_GET, handleS3AudioRx);
  webServer.on("/api/audio/tx", HTTP_POST, handleS3AudioTx, handleS3AudioUpload);
  webServer.on("/api/wifi", HTTP_POST, handleS3WifiSave);
  webServer.begin();

  // Dedicated FreeRTOS task on Core 0 for smooth Port 81 MJPEG streaming
  xTaskCreatePinnedToCore(mjpegStreamTask, "mjpeg_stream", 4096, nullptr, 2, nullptr, 0);

  sendUartStatusPacket();
  Serial.println("S3-CAM Studio Ready.");
}

void loop() {
  webServer.handleClient();
  pollMainEspUart();
  delay(2);
}