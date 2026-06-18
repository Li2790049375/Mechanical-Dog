#include <arduino.h>
#include "WebPage.h"
#include <Adafruit_NeoPixel.h>

#define BUZZER 21

// WiFi热点(AP)配置
#define AP_SSID "LH666"
#define AP_PWD  "LH123456"

// WiFi客户端(STA)配置
#define STA_SSID "iPhone"
#define STA_PWD  "1234567800"

// 默认WiFi模式: 1=AP热点模式(上位机控制), 2=STA客户端模式
#define DEFAULT_WIFI_MODE 1
extern IPAddress IP_ADDRESS;
extern byte WIFI_MODE;
extern int WIFI_RSSI;

#include "esp_wifi.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"

// ESP-EYE摄像头引脚定义
#define CAMERA_MODEL_ESP_EYE
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     4
#define SIOD_GPIO_NUM     18
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       36
#define Y8_GPIO_NUM       37
#define Y7_GPIO_NUM       38
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM       35
#define Y4_GPIO_NUM       14
#define Y3_GPIO_NUM       13
#define Y2_GPIO_NUM       34
#define VSYNC_GPIO_NUM    5
#define HREF_GPIO_NUM     27
#define PCLK_GPIO_NUM     25

extern int MiddlePosition;
extern int moveFB;
extern int moveLR;
extern int debugMode;
extern int funcMode;
extern void initPosAll();
extern void middlePosAll();
extern void servoDebug(byte servoID, int offset);
extern void servoConfigSave(byte activeServo);
extern int ServoMiddlePWM[16];
extern int CurrentPWM[16];
extern Adafruit_NeoPixel matrix;
extern void setSingleLED(uint16_t LEDnum, uint32_t c);
extern portMUX_TYPE ctrlMux;
extern float gestureUD;
extern float gestureLR;

void balanceRainbowLED(){
  static uint8_t balanceHue = 0;
  uint8_t r, g, b;
  uint8_t sector = balanceHue / 43;
  uint8_t pos = (balanceHue - sector * 43) * 6;
  switch(sector){
    case 0: r = 255; g = pos; b = 0; break;
    case 1: r = 255 - pos; g = 255; b = 0; break;
    case 2: r = 0; g = 255; b = pos; break;
    case 3: r = 0; g = 255 - pos; b = 255; break;
    case 4: r = pos; g = 0; b = 255; break;
    default: r = 255; g = 0; b = 255 - pos; break;
  }
  setSingleLED(0, matrix.Color(r, g, b));
  setSingleLED(1, matrix.Color(r, g, b));
  balanceHue += 3;
}


void getIP(){
  IP_ADDRESS = WiFi.localIP();
}


void setAP(){
  WiFi.softAP(AP_SSID, AP_PWD);
  IPAddress myIP = WiFi.softAPIP();
  IP_ADDRESS = myIP;
  WIFI_MODE = 1;
}


void setSTA(){
  WIFI_MODE = 3;
  WiFi.begin(STA_SSID, STA_PWD);
}


// WiFi重连间隔控制(避免频繁重连)
unsigned long lastWifiReconnect = 0;
#define WIFI_RECONNECT_INTERVAL 5000

void getWifiStatus(){
  if(WiFi.status() == WL_CONNECTED){
    WIFI_MODE = 2;
    getIP();
    WIFI_RSSI = WiFi.RSSI();
  }
  else if(DEFAULT_WIFI_MODE == 2){
    // STA模式下处理各种断连状态
    if(WiFi.status() == WL_CONNECTION_LOST ||
       WiFi.status() == WL_DISCONNECTED ||
       WiFi.status() == WL_IDLE_STATUS ||
       WiFi.status() == WL_CONNECT_FAILED){
      WIFI_MODE = 3;
      // 限制重连频率,避免阻塞主循环
      if(millis() - lastWifiReconnect > WIFI_RECONNECT_INTERVAL){
        Serial.println("WiFi断开,尝试重连...");
        WiFi.reconnect();
        lastWifiReconnect = millis();
      }
    }
  }
}


void wifiInit(){
  WIFI_MODE = DEFAULT_WIFI_MODE;
  if(WIFI_MODE == 1){setAP();}
  else if(WIFI_MODE == 2){setSTA();}
}


#define PART_BOUNDARY "L1234567890000000000987654321H"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
 
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
 
  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }
 
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
 
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
    } 
    else {
      {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;

    vTaskDelay(pdMS_TO_TICKS(10));
  }
 
  last_frame = 0;
  return res;
}

// 功能: 运动, 功能模式, 舵机调试, 灯光, 摄像头参数
static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  char value[32] = {0,};
  char cmd[32] = {0,};
 
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK &&
          httpd_query_key_value(buf, "cmd", cmd, sizeof(cmd)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
 
  int val = atoi(value);
  int cmdint = atoi(cmd);
  int res = 0;
 
  // 根据URL参数执行对应功能
  if (!strcmp(variable, "framesize")){
    Serial.println("framesize");
    sensor_t * s = esp_camera_sensor_get();
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  }

  // funcMode
  else if (!strcmp(variable, "funcMode")){
    int _funcMode;
    portENTER_CRITICAL(&ctrlMux);
    debugMode = 0;
    gestureUD = 0;
    gestureLR = 0;
    if (val == 1){
      if(funcMode == 1){funcMode = 0;}
      else if(funcMode == 0){funcMode = 1;}
    }
    else{
      funcMode = val;
    }
    _funcMode = funcMode;
    portEXIT_CRITICAL(&ctrlMux);
    if (val == 1){
      Serial.println(_funcMode == 1 ? "Balance ON" : "Balance OFF");
    }
    else{
      Serial.println(val);
    }
    // 每个动作对应的颜色
    if (val == 2) {
      setSingleLED(0, matrix.Color(255, 200, 0)); setSingleLED(1, matrix.Color(255, 200, 0));
    } else if (val == 3) {
      setSingleLED(0, matrix.Color(200, 0, 255)); setSingleLED(1, matrix.Color(200, 0, 255));
    } else if (val == 4) {
      setSingleLED(0, matrix.Color(255, 100, 0)); setSingleLED(1, matrix.Color(255, 100, 0));
    } else if (val == 5) {
      setSingleLED(0, matrix.Color(255, 0, 0)); setSingleLED(1, matrix.Color(255, 0, 0));
    } else if (val == 6) {
      setSingleLED(0, matrix.Color(255, 0, 64)); setSingleLED(1, matrix.Color(255, 0, 64));
    } else if (val == 7) {
      setSingleLED(0, matrix.Color(0, 0, 255)); setSingleLED(1, matrix.Color(0, 0, 255));
    } else if (val == 8) {
      setSingleLED(0, matrix.Color(255, 255, 255)); setSingleLED(1, matrix.Color(255, 255, 255));
    } else if (val == 9) {
      setSingleLED(0, matrix.Color(255, 180, 100)); setSingleLED(1, matrix.Color(255, 180, 100));
    } else if (val == 10) {
      setSingleLED(0, matrix.Color(0, 64, 255)); setSingleLED(1, matrix.Color(0, 64, 255));
    } else if (val == 11) {
      setSingleLED(0, matrix.Color(255, 64, 64)); setSingleLED(1, matrix.Color(255, 64, 64));
    }
  }

  // 舵机调试: val=舵机ID, cmdint=偏移量
  else if (!strcmp(variable, "sconfig")){
    portENTER_CRITICAL(&ctrlMux);
    debugMode = 1;
    funcMode = 0;
    portEXIT_CRITICAL(&ctrlMux);
    servoDebug(val, cmdint);
    Serial.print("servo:");Serial.print(val);Serial.print(" position:");Serial.print(CurrentPWM[val]);
    Serial.print(" MID:");Serial.print(ServoMiddlePWM[val]);Serial.print(" offset:");Serial.println(cmdint);
  }
  else if (!strcmp(variable, "sset")){
    int _debugMode;
    portENTER_CRITICAL(&ctrlMux);
    _debugMode = debugMode;
    portEXIT_CRITICAL(&ctrlMux);
    if(_debugMode && val >= 0 && val < 16){
    servoConfigSave(val);
    Serial.print("SET servo:");Serial.print(val);Serial.print(" position:");Serial.println(ServoMiddlePWM[val]);
    }
    else{
      Serial.print("DebugMode = 0, servo config could not be saved.");
    }
  }

  // 运动控制
  else if (!strcmp(variable, "move")){
    portENTER_CRITICAL(&ctrlMux);
    debugMode = 0;
    funcMode  = 0;
    if (val == 1) {
      moveFB = 1;
    }
    else if (val == 2) {
      moveLR = -1;
    }
    else if (val == 3) {
      moveFB = 0;
    }
    else if (val == 4) {
      moveLR = 1;
    }
    else if (val == 5) {
      moveFB = -1;
    }
    else if (val == 6){
      moveLR = 0;
    }
    portEXIT_CRITICAL(&ctrlMux);

    digitalWrite(BUZZER, HIGH);
    if (val == 1) {
      Serial.println("前进");
      setSingleLED(0, matrix.Color(0, 128, 255)); setSingleLED(1, matrix.Color(0, 128, 255));
    }
    else if (val == 2) {
      Serial.println("左转");
      setSingleLED(0, matrix.Color(255, 255, 0)); setSingleLED(1, matrix.Color(255, 255, 0));
    }
    else if (val == 3) {
      Serial.println("前后停止");
      setSingleLED(0, matrix.Color(128, 128, 128)); setSingleLED(1, matrix.Color(128, 128, 128));
    }
    else if (val == 4) {
      Serial.println("右转");
      setSingleLED(0, matrix.Color(0, 255, 255)); setSingleLED(1, matrix.Color(0, 255, 255));
    }
    else if (val == 5) {
      Serial.println("后退");
      setSingleLED(0, matrix.Color(255, 160, 0)); setSingleLED(1, matrix.Color(255, 160, 0));
    }
    else if (val == 6){
      Serial.println("左右停止");
      setSingleLED(0, matrix.Color(128, 128, 128)); setSingleLED(1, matrix.Color(128, 128, 128));
    }
  }

  else
  {
    Serial.println("variable");
    res = -1;
  }
 
  if (res) {
    return httpd_resp_send_500(req);
  }
 
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

 
// 页面服务: 返回WebPage.h中的HTML控制页面
static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, sizeof(INDEX_HTML) - 1);
}


// 注册HTTP路由
void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
 
    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };
 
    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };
 
   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
    }
 
    config.server_port += 1;
    config.ctrl_port += 1;
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}


// 摄像头与WiFi服务初始化
// 流程: 禁用欠压复位 -> 配置引脚 -> 初始化摄像头 -> 启动 WiFi -> 启动 HTTP
void webServerInit(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // 禁用欠压复位

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

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count = 2;

  // 摄像头初始化
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
  }
  else{
    sensor_t * s = esp_camera_sensor_get();
    s->set_saturation(s, 2);
    delay(1000); 
  }

  wifiInit();
  delay(1000); 
  startCameraServer();
}
