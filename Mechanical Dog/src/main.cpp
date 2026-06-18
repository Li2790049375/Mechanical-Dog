IPAddress IP_ADDRESS(0, 0, 0, 0);
byte WIFI_MODE = 0;                   // WiFi模式: 0=未初始化, 1=AP热点, 2=STA已连接, 3=STA连接中
void getWifiStatus();
int WIFI_RSSI = 0;                    // WiFi信号强度

// 步态类型: 0=对角步态, 1=三角步态
int GAIT_TYPE = 0;

int moveFB = 0;                       // 前后: 1=前进, -1=后退, 0=停止
int moveLR = 0;                       // 左右: 1=右转, -1=左转, 0=停止
int debugMode = 0;                    // 调试模式开关
int funcMode  = 0;                    // 功能模式: 0=无, 1=平衡, 2=趴下, 3=握手, 4=跳跃, 5=点头, 6=打招呼, 7=旋转, 8=初始位置, 9=中间位置, 10=蹲下, 11=趴平
float gestureUD = 0;                  // 俯仰偏移量
float gestureLR = 0;                  // 偏航偏移量
float gestureOffSetMax = 15;
float gestureSpeed = 2;
int STAND_STILL = 0;

// 双核共享变量互斥锁
// 串口路径(serialCtrl) 和主循环(robotCtrl) 都使用此锁
portMUX_TYPE ctrlMux = portMUX_INITIALIZER_UNLOCKED;
unsigned long LAST_JSON_SEND;
int JSON_SEND_INTERVAL = 1000;

#include "InitConfig.h"
#include "robot_motion.h"
#include "PreferencesConfig.h"
#include <ArduinoJson.h>

#include "WiFi.h"
#include "WebServer.h"

StaticJsonDocument<200> docReceive;
StaticJsonDocument<100> docSend;
TaskHandle_t threadings;


void webServerInit();

// 串口 JSON 控制指令解析（运行在 Core 0 robotThreadings 任务中）
// 支持: funcMode(功能模式), move(运动), ges(姿态), light(灯光), buzzer(蜂鸣器)
void serialCtrl(){
  if (Serial.available()){
    DeserializationError err = deserializeJson(docReceive, Serial);

    if (err == DeserializationError::Ok){
      int val = docReceive["val"];

      if(docReceive["var"] == "funcMode"){
        portENTER_CRITICAL(&ctrlMux);
        debugMode = 0;
        gestureUD = 0;
        gestureLR = 0;
        if(val == 1){
          if(funcMode == 1){funcMode = 0;}
          else if(funcMode == 0){funcMode = 1;}
        }
        else{
          funcMode = val;
        }
        portEXIT_CRITICAL(&ctrlMux);
        Serial.println(val);
      }

      else if(docReceive["var"] == "move"){
        portENTER_CRITICAL(&ctrlMux);
        debugMode = 0;
        funcMode  = 0;
        switch(val){
          case 1: moveFB = 1; break;
          case 2: moveLR =-1; break;
          case 3: moveFB = 0; break;
          case 4: moveLR = 1; break;
          case 5: moveFB =-1; break;
          case 6: moveLR = 0; break;
        }
        portEXIT_CRITICAL(&ctrlMux);
        digitalWrite(BUZZER, HIGH);
        switch(val){
          case 1: Serial.println("前进");break;
          case 2: Serial.println("左转");break;
          case 3: Serial.println("前后停止");break;
          case 4: Serial.println("右转");break;
          case 5: Serial.println("后退");break;
          case 6: Serial.println("左右停止");break;
        }
      }

      else if(docReceive["var"] == "ges"){
        float _ud, _lr;
        portENTER_CRITICAL(&ctrlMux);
        debugMode = 0;
        funcMode  = 0;
        switch(val){
          case 1: gestureUD += gestureSpeed;if(gestureUD > gestureOffSetMax){gestureUD = gestureOffSetMax;}break;
          case 2: gestureUD -= gestureSpeed;if(gestureUD <-gestureOffSetMax){gestureUD =-gestureOffSetMax;}break;
          case 3: gestureUD = 0; break;
          case 4: gestureLR -= gestureSpeed;if(gestureLR <-gestureOffSetMax){gestureLR =-gestureOffSetMax;}break;
          case 5: gestureLR += gestureSpeed;if(gestureLR > gestureOffSetMax){gestureLR = gestureOffSetMax;}break;
          case 6: gestureLR = 0; break;
        }
        _ud = gestureUD;
        _lr = gestureLR;
        portEXIT_CRITICAL(&ctrlMux);
        pitchYawRollHeightCtrl(_ud, _lr, 0, 0);
      }

      else if(docReceive["var"] == "light"){
        switch(val){
          case 0: setSingleLED(0,matrix.Color(0, 0, 0));setSingleLED(1,matrix.Color(0, 0, 0));break;
          case 1: setSingleLED(0,matrix.Color(0, 32, 255));setSingleLED(1,matrix.Color(0, 32, 255));break;
          case 2: setSingleLED(0,matrix.Color(255, 32, 0));setSingleLED(1,matrix.Color(255, 32, 0));break;
          case 3: setSingleLED(0,matrix.Color(32, 255, 0));setSingleLED(1,matrix.Color(32, 255, 0));break;
          case 4: setSingleLED(0,matrix.Color(255, 255, 0));setSingleLED(1,matrix.Color(255, 255, 0));break;
          case 5: setSingleLED(0,matrix.Color(0, 255, 255));setSingleLED(1,matrix.Color(0, 255, 255));break;
          case 6: setSingleLED(0,matrix.Color(255, 0, 255));setSingleLED(1,matrix.Color(255, 0, 255));break;
          case 7: setSingleLED(0,matrix.Color(255, 64, 32));setSingleLED(1,matrix.Color(32, 64, 255));break;
        }
      }

      else if(docReceive["var"] == "buzzer"){
        switch(val){
          case 0: digitalWrite(BUZZER, HIGH);break;
          case 1: digitalWrite(BUZZER, LOW);break;
        }
      }
    }

    else {
      while (Serial.available() > 0)
        Serial.read();
    }
  }
}

// 串口定时发送电压数据
void jsonSend(){
  if(millis() - LAST_JSON_SEND > JSON_SEND_INTERVAL || millis() < LAST_JSON_SEND){
    docSend["vol"] = loadVoltage_V;
    serializeJson(docSend, Serial);
    LAST_JSON_SEND = millis();
  }
}

// FreeRTOS 任务: 串口控制监听（Core 0）
// 启动后延迟 3s 等待硬件稳定，然后以 25ms 周期轮询串口
void robotThreadings(void *pvParameter){
  delay(3000);
  while(1){
    serialCtrl();
    jsonSend();
    delay(25);
  }
}

void threadingsInit(){
  xTaskCreate(&robotThreadings, "RobotThreadings", 4000, NULL, 5, &threadings);
}

void setup() {
  // 上电初始化流程: I2C → 串口 → 外设检测 → 舵机 → 屏幕 → NVS → 站立 → IMU → WiFi/HTTP → 主任务
  Wire.begin(S_SDA, S_SCL);
  Serial.begin(115200);

  wireDebugInit();
  InitINA219();
  InitBuzzer();
  InitRGB();
  ServoSetup();
  InitScreen();
  preferencesSetup();

  // 舵机初始状态: 站立并校准IMU
  delay(100);
  setSingleLED(0,matrix.Color(0, 128, 255));
  setSingleLED(1,matrix.Color(0, 128, 255));
  standMassCenter(0, 0);GoalPosAll();delay(1000);
  setSingleLED(0,matrix.Color(255, 128, 0));
  setSingleLED(1,matrix.Color(255, 128, 0));
  delay(500);

  InitICM20948();
  webServerInit();

  delay(500);
  setSingleLED(0,matrix.Color(0, 255, 0));
  setSingleLED(1,matrix.Color(0, 255, 0));

  allDataUpdate();
  threadingsInit();
}

void loop() {
  // 主循环 (Core 1): 步态调度 + 外设数据刷新 + 有线调试检测
  robotCtrl();
  allDataUpdate();
  wireDebugDetect();
}
