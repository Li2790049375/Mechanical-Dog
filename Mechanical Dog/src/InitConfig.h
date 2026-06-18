#include <Wire.h>

// 双核选择: OLED显示用一个核，步态控制用另一个核
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#define GAIT_RUNNING_CORE    1
#else
#define ARDUINO_RUNNING_CORE 1
#define GAIT_RUNNING_CORE    0
#endif

// GPIO引脚定义
#define S_SCL   33
#define S_SDA   32
#define RGB_LED 26
#define BUZZER  21
#define WIRE_DEBUG 12

// 舵机中间位置(脉宽计数)
int MiddlePosition = 300;

// 16路舵机当前位置
int CurrentPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                      MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                      MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                      MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

// 有线调试: IO12接HIGH时进入调试模式
void wireDebugInit(){
  pinMode(WIRE_DEBUG, INPUT_PULLDOWN);
}

// ICM20948 九轴IMU (地址0x68)
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

float ACC_X;
float ACC_Y;
float ACC_Z;

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

void InitICM20948(){
  myIMU.init();
  delay(200);
  myIMU.autoOffsets();

  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);
}

void accXYZUpdate(){
  myIMU.readSensor();
  xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();

  ACC_X = corrAccRaw.x;
  ACC_Y = corrAccRaw.y;
  ACC_Z = corrAccRaw.z;
}

// INA219 电流电压传感器 (地址0x42)
#include <INA219_WE.h>
#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
bool ina219_overflow = false;

void InitINA219(){
  ina219.init();
  ina219.setADCMode(BIT_MODE_9);
  ina219.setPGain(PG_320);
  ina219.setBusRange(BRNG_16);
  ina219.setShuntSizeInOhms(0.01);
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}

// SSD1306 OLED屏幕 (128x32, 软件I2C)
#include <U8g2lib.h>
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 33, /* data=*/ 32);

// 绘制火柴人打招呼动画
void drawStickMan(int frame) {
  u8g2.clearBuffer();
  int cx = 64;

  u8g2.drawCircle(cx, 8, 5, U8G2_DRAW_ALL);
  u8g2.drawLine(cx, 13, cx, 23);
  u8g2.drawLine(cx, 23, cx - 6, 31);
  u8g2.drawLine(cx, 23, cx + 6, 31);
  u8g2.drawLine(cx, 16, cx - 7, 21);

  if (frame == 0) {
    u8g2.drawLine(cx, 16, cx + 7, 21);
  } else {
    u8g2.drawLine(cx, 16, cx + 10, 12);
  }

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(85, 18, "Hi!");
  u8g2.sendBuffer();
}

void showStickManAnimation() {
  for (int i = 0; i < 3; i++) {
    drawStickMan(0);
    delay(250);
    drawStickMan(1);
    delay(250);
  }
}

void showNames() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_wqy13_t_gb2312a);
  u8g2.setCursor(2, 13);
  u8g2.print("李航");
  u8g2.setCursor(2, 28);
  u8g2.print("Mechanical Dog");
  u8g2.sendBuffer();
}

void showStatus() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);

  // 第一行: 当前状态
  u8g2.setCursor(0, 10);
  if(debugMode){
    u8g2.print("DEBUG");
  }
  else if(funcMode == 1){
    u8g2.print("BALANCE");
  }
  else if(funcMode == 2){
    u8g2.print("STAY LOW");
  }
  else if(funcMode == 3){
    u8g2.print("HANDSHAKE");
  }
  else if(funcMode == 4){
    u8g2.print("JUMP");
  }
  else if(funcMode == 5){
    u8g2.print("NOD");
  }
  else if(funcMode == 6){
    u8g2.print("HELLO");
  }
  else if(funcMode == 7){
    u8g2.print("SPIN");
  }
  else if(funcMode == 8){
    u8g2.print("INIT POS");
  }
  else if(funcMode == 9){
    u8g2.print("MID POS");
  }
  else if(funcMode == 10){
    u8g2.print("Crouch");
  }
  else if(funcMode == 11){
    u8g2.print("GET DOWN");
  }
  else if(moveFB == 1){
    u8g2.print("前进");
  }
  else if(moveFB == -1){
    u8g2.print("后退");
  }
  else if(moveLR == -1){
    u8g2.print("左转");
  }
  else if(moveLR == 1){
    u8g2.print("右转");
  }
  else{
    u8g2.print("待机");
  }

  // 第二行: 电压 + WiFi
  u8g2.setCursor(0, 22);
  u8g2.print(loadVoltage_V, 1);
  u8g2.print("V ");
  if(WIFI_MODE == 1){
    u8g2.print("AP");
  }
  else if(WIFI_MODE == 2){
    u8g2.print("STA:");
    u8g2.print(WIFI_RSSI);
  }
  else{
    u8g2.print("WiFi...");
  }

  u8g2.sendBuffer();
}

void InitScreen(){
  u8g2.begin();
  showStickManAnimation();
  showNames();
  delay(1000);
}

#define OLED_REFRESH_MS 500
#define INA219_REFRESH_MS 1000
unsigned long lastOledRefresh = 0;
unsigned long lastIna219Refresh = 0;

void allDataUpdate(){
  // INA219每秒读一次
  if(millis() - lastIna219Refresh > INA219_REFRESH_MS){
    InaDataUpdate();
    lastIna219Refresh = millis();
  }
  if(millis() - lastOledRefresh > OLED_REFRESH_MS){
    getWifiStatus();
    showStatus();
    lastOledRefresh = millis();
  }
}

// 蜂鸣器初始化
void InitBuzzer(){
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
}

// WS2812 RGB灯初始化
#include <Adafruit_NeoPixel.h>
#define NUMPIXELS   6
#define BRIGHTNESS  255
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUMPIXELS, RGB_LED, NEO_GRB + NEO_KHZ800);

void InitRGB(){
  matrix.setBrightness(BRIGHTNESS);
  matrix.begin();
  matrix.show();
}

void setSingleLED(uint16_t LEDnum, uint32_t c){
  matrix.setPixelColor(LEDnum, c);
  matrix.show();
}
