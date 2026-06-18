#include <Preferences.h>

Preferences preferences;

// 从NVS读取所有舵机校准中间值, key格式: "PWM0"~"PWM15"
void middleUpdate(){
  for(int i = 0; i < 16; i++){
    String key = "PWM" + String(i);
    ServoMiddlePWM[i] = preferences.getInt(key.c_str(), ServoMiddlePWM[i]);
  }
}

// 保存指定舵机的当前脉宽作为中间值
void servoConfigSave(byte activeServo){
  ServoMiddlePWM[activeServo] = CurrentPWM[activeServo];
  String key = "PWM" + String(activeServo);
  preferences.putInt(key.c_str(), CurrentPWM[activeServo]);
}

void preferencesSetup(){
  preferences.begin("ServoConfig", false);
  middleUpdate();
  delay(500);
}
