#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVOMIN  263 // 舵机最小脉宽计数(4096步)
#define SERVOMAX  463 // 舵机最大脉宽计数(4096步)
#define SERVO_FREQ 50 // 模拟舵机频率50Hz
#define SERVO_RANGE 90

#ifndef M_PI
#define M_PI 3.1415926
#endif

// 连杆机构尺寸参数(mm)
double Linkage_W = 19.15;   // 摆动舵机到腿部连杆平面的距离

double Linkage_S = 12.2;    // 两个舵机之间的距离
double Linkage_A = 40.0;    // 连杆A: 与舵机连接的连杆
double Linkage_B = 40.0;    // 连杆B: 限向连杆
double Linkage_C = 39.8153; // 连杆C: 腿部上段
double Linkage_D = 31.7750; // 连杆D: 腿部下段
double Linkage_E = 30.8076; // 连杆E: 足部

double WALK_HEIGHT_MAX  = 110;
double WALK_HEIGHT_MIN  = 75;
double WALK_HEIGHT      = 95;
double WALK_LIFT        = 9; // 抬腿高度, 需满足: 行走高度+抬腿高度≤最大高度
double WALK_RANGE       = 40;
double WALK_ACC         = 5;
double WALK_EXTENDED_X  = 16;
double WALK_EXTENDED_Z  = 25;
double WALK_SIDE_MAX    = 30;
double WALK_MASS_ADJUST = 21;
double STAND_HEIGHT     = 95;

float WALK_LIFT_PROP    = 0.25; // 抬腿占比(需<1)

float BALANCE_PITCH_BUFFER;
float BALANCE_ROLL_BUFFER;
float BALANCE_P = 0.00018; // 平衡PID比例系数
float BALANCE_D = 0.00002; // 平衡PID微分系数
float BALANCE_PREV_ACC_X = 0; // 上一次X轴加速度
float BALANCE_PREV_ACC_Y = 0; // 上一次Y轴加速度

float GLOBAL_STEP  = 0;    // 当前步态周期进度
int   STEP_DELAY   = 4;    // 步态更新延迟(ms)
float STEP_ITERATE = 0.04; // 每次步态迭代步进量

int SERVO_MOVE_EVERY = 0;

#define LEG_A_FORE 8
#define LEG_A_BACK 9
#define LEG_A_WAVE 10

#define LEG_B_WAVE 13
#define LEG_B_FORE 14
#define LEG_B_BACK 15

#define LEG_C_FORE 7
#define LEG_C_BACK 6
#define LEG_C_WAVE 5

#define LEG_D_WAVE 2
#define LEG_D_FORE 1
#define LEG_D_BACK 0


int ServoMiddlePWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                          MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                          MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                          MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int legPosBuffer[12] = { (int) WALK_EXTENDED_X,  (int) STAND_HEIGHT, (int) WALK_EXTENDED_Z,
                        (int)-WALK_EXTENDED_X,  (int) STAND_HEIGHT, (int) WALK_EXTENDED_Z,
                        (int) WALK_EXTENDED_X,  (int) STAND_HEIGHT, (int) WALK_EXTENDED_Z,
                        (int)-WALK_EXTENDED_X,  (int) STAND_HEIGHT, (int) WALK_EXTENDED_Z};

int GoalPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int ServoDirection[16] = {-1,  1,  1,  1,
                           1, -1, -1,  1,
                          -1,  1,  1,  1,
                           1, -1, -1,  1};

double linkageBuffer[32] = {0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0};


// 预计算常量(编译时优化)
float LAxLA = Linkage_A*Linkage_A;
float LBxLB = Linkage_B*Linkage_B;
float LWxLW = Linkage_W*Linkage_W;
float LExLE = Linkage_E*Linkage_E;
float LAxLA_LBxLB = LAxLA - LBxLB;
float LBxLB_LAxLA = LBxLB - LAxLA;
float L_CD = (Linkage_C+Linkage_D)*(Linkage_C+Linkage_D);
float LAx2  = 2 * Linkage_A;
float LBx2  = 2 * Linkage_B;
float E_PI  = 180 / M_PI;
float LSs2  = Linkage_S/2;
float aLCDE = atan((Linkage_C + Linkage_D)/Linkage_E);
float sLEDC = sqrt(Linkage_E*Linkage_E + (Linkage_D+Linkage_C)*(Linkage_D+Linkage_C));
float O_WLP = 1 - WALK_LIFT_PROP;
float WALK_ACCx2 = WALK_ACC*2;
float WALK_H_L = WALK_HEIGHT - WALK_LIFT;


void ServoSetup(){
  Wire.setClock(400000);
  pwm.begin();
  pwm.setOscillatorFrequency(26000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
}


void initPosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, MiddlePosition);
    CurrentPWM[i] = MiddlePosition;
    Serial.print(CurrentPWM[i]);
    Serial.print(" ");
    delay(SERVO_MOVE_EVERY);
  }
  Serial.println(" ");
}


void middlePosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, ServoMiddlePWM[i]);
    CurrentPWM[i] = ServoMiddlePWM[i];
    Serial.print(CurrentPWM[i]);
    Serial.print(" ");
    delay(SERVO_MOVE_EVERY);
  }
  Serial.println(" ");
}


void servoDebug(byte servoID, int offset){
  if(servoID >= 16) return;
  CurrentPWM[servoID] += offset;
  if(CurrentPWM[servoID] < SERVOMIN) CurrentPWM[servoID] = SERVOMIN;
  if(CurrentPWM[servoID] > SERVOMAX) CurrentPWM[servoID] = SERVOMAX;
  pwm.setPWM(servoID, 0, CurrentPWM[servoID]);  
}


void GoalPosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, GoalPWM[i]);
  }
}


void goalPWMSet(uint8_t servoNum, double angleInput){
  int pwmGet;
  if (angleInput == 0){pwmGet = 0;}
  else{pwmGet = round((SERVOMAX - SERVOMIN) * angleInput / SERVO_RANGE);}
  pwmGet = pwmGet * ServoDirection[servoNum] + ServoMiddlePWM[servoNum];
  GoalPWM[servoNum] = pwmGet;
}



void simpleLinkageIK(double LA, double LB, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta){
  double psi;
  double alpha;
  double omega;
  double beta;
  double L2C;
  double LC;
  double lambda;
  double delta;
  if(bIn == 0){
    double arg1 = (LAxLA_LBxLB + aIn * aIn)/(LAx2 * aIn);
    if(arg1 > 1) arg1 = 1; if(arg1 < -1) arg1 = -1;
    psi   = acos(arg1) * E_PI;
    alpha = 90 - psi;
    double arg2 = (aIn * aIn + LBxLB_LAxLA)/(LBx2 * aIn);
    if(arg2 > 1) arg2 = 1; if(arg2 < -1) arg2 = -1;
    omega = acos(arg2) * E_PI;
    beta  = psi + omega;
  }
  else{
    L2C = aIn * aIn + bIn * bIn;
    LC  = sqrt(L2C);
    lambda = atan(bIn/aIn) * 180 / M_PI;
    double arg3 = (LAxLA_LBxLB + L2C)/(2 * LA * LC);
    if(arg3 > 1) arg3 = 1; if(arg3 < -1) arg3 = -1;
    psi    = acos(arg3) * E_PI;
    alpha  = 90 - lambda - psi;
    double arg4 = (LBxLB_LAxLA + L2C)/(2 * LC * LB);
    if(arg4 > 1) arg4 = 1; if(arg4 < -1) arg4 = -1;
    omega  = acos(arg4) * E_PI;
    beta   = psi + omega;
  }
  delta = 90 - alpha - beta;
  linkageBuffer[outputAlpha] = alpha;
  linkageBuffer[outputBeta]  = beta;
  linkageBuffer[outputDelta] = delta;
}


void wigglePlaneIK(double LA, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputLen){
  double LB;
  double L2C;
  double LC;
  double alpha;
  double lambda;
  double psi;
  if(bIn > 0){
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * E_PI;
    double argW = LA/LC;
    if(argW > 1) argW = 1; if(argW < -1) argW = -1;
    psi = acos(argW) * E_PI;
    LB = sqrt(L2C - LWxLW);
    alpha = psi + lambda - 90;
  }
  else if(bIn == 0){
    if(aIn == 0){ linkageBuffer[outputAlpha] = 0; linkageBuffer[outputLen] = 0; return; }
    alpha = asin(LA/aIn) * E_PI;
    L2C = aIn * aIn + bIn * bIn;
    LB = sqrt(L2C);
  }
  else{
    bIn = -bIn;
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * E_PI;
    double argW2 = LA/LC;
    if(argW2 > 1) argW2 = 1; if(argW2 < -1) argW2 = -1;
    psi = acos(argW2) * E_PI;
    LB = sqrt(L2C - LWxLW);
    alpha = 90 - lambda + psi;
  }
  linkageBuffer[outputAlpha] = alpha;
  linkageBuffer[outputLen]  = LB;
}


void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, double xIn, double yIn, uint8_t outputBeta, uint8_t outputX, uint8_t outputY){
  double bufferS = sqrt((xIn + LSs2)*(xIn + LSs2) + yIn*yIn);
  double argS = ((xIn + LSs2)*(xIn + LSs2) + yIn*yIn + LAxLA - L_CD - LExLE)/(2*bufferS*LA);
  if(argS > 1) argS = 1; if(argS < -1) argS = -1;
  double lambda = acos(argS);
  double delta = atan((xIn + LSs2)/yIn);
  double beta = lambda - delta;
  double betaAngle = beta * E_PI;

  double theta = aLCDE;
  double omega = asin((yIn - cos(beta)*LA)/sLEDC);
  double nu = M_PI - theta - omega;
  double dFX = cos(nu)*LE;
  double dFY = sin(nu)*LE;

  double mu = M_PI/2 - nu;
  double dEX = cos(mu)*LD;
  double dEY = sin(mu)*LD;

  double positionX = xIn + dFX - dEX;
  double positionY = yIn - dFY - dEY;
  
  linkageBuffer[outputBeta] = betaAngle;
  linkageBuffer[outputX]  = positionX;
  linkageBuffer[outputY]  = positionY;
}


void singleLegCtrl(uint8_t LegNum, double xPos, double yPos, double zPos){
  uint8_t alphaOut;
  uint8_t xPosBuffer;
  uint8_t yPosBuffer;
  uint8_t betaOut;
  uint8_t betaB;
  uint8_t betaC;
  uint8_t NumF;
  uint8_t NumB;
  uint8_t wiggleAlpha;
  uint8_t wiggleLen;
  uint8_t NumW;
  if(LegNum < 1 || LegNum > 4) return;
  if(LegNum == 1){
    NumF = LEG_A_FORE;
    NumB = LEG_A_BACK;
    NumW = LEG_A_WAVE;
    alphaOut   = 0;
    xPosBuffer = 1;
    yPosBuffer = 2;
    betaOut = 3;
    betaB   = 4;
    betaC   = 5;
    wiggleAlpha = 6;
    wiggleLen   = 7;
  }
  else if(LegNum == 2){
    NumF = LEG_B_FORE;
    NumB = LEG_B_BACK;
    NumW = LEG_B_WAVE;
    alphaOut   = 8;
    xPosBuffer = 9;
    yPosBuffer = 10;
    betaOut = 11;
    betaB   = 12;
    betaC   = 13;
    wiggleAlpha = 14;
    wiggleLen   = 15;
  }
  else if(LegNum == 3){
    NumF = LEG_C_FORE;
    NumB = LEG_C_BACK;
    NumW = LEG_C_WAVE;
    alphaOut   = 16;
    xPosBuffer = 17;
    yPosBuffer = 18;
    betaOut = 19;
    betaB   = 20;
    betaC   = 21;
    wiggleAlpha = 22;
    wiggleLen   = 23;
  }
  else if(LegNum == 4){
    NumF = LEG_D_FORE;
    NumB = LEG_D_BACK;
    NumW = LEG_D_WAVE;
    alphaOut   = 24;
    xPosBuffer = 25;
    yPosBuffer = 26;
    betaOut = 27;
    betaB   = 28;
    betaC   = 29;
    wiggleAlpha = 30;
    wiggleLen   = 31;
  }

  wigglePlaneIK(Linkage_W, zPos, yPos, wiggleAlpha, wiggleLen);
  singleLegPlaneIK(Linkage_S, Linkage_A, Linkage_C, Linkage_D, Linkage_E, xPos, linkageBuffer[wiggleLen], alphaOut, xPosBuffer, yPosBuffer);
  simpleLinkageIK(Linkage_A, Linkage_B, linkageBuffer[yPosBuffer], (linkageBuffer[xPosBuffer]-Linkage_S/2), betaOut, betaB, betaC);

  goalPWMSet(NumW, linkageBuffer[wiggleAlpha]);
  goalPWMSet(NumF, (90 - linkageBuffer[betaOut]));
  goalPWMSet(NumB, linkageBuffer[alphaOut]);
}


void standUp(double cmdInput){
  singleLegCtrl(1, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(2, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(3, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(4, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
}


// 单腿步态控制
// cycleInput: 步态周期(0~1)
// directionInput: 运动方向角度(>0时远离中线)
// extendedX/extendedZ: 摆动周期中点偏移量
// statusInput: 用于缩减步幅
void singleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, float directionInput, double extendedX, double extendedZ){
  double rDist = 0;
  double xGait = 0;
  double yGait = 0;
  double zGait = 0;
  double rDiection = directionInput * M_PI / 180;

  if(cycleInput < O_WLP){
    if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP){
      yGait = WALK_H_L + cycleInput/(O_WLP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP)*WALK_LIFT;
    }
    else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP){
      yGait = WALK_HEIGHT;
    }
    else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP){
      yGait = WALK_HEIGHT - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP)/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*O_WLP))*WALK_LIFT;
    }

    rDist = (WALK_RANGE*statusInput/2 + WALK_ACC) - (cycleInput/O_WLP)*(WALK_RANGE*statusInput + WALK_ACC*2);
  }
  else if(cycleInput >= O_WLP){
    yGait = WALK_H_L;
    rDist = - (WALK_RANGE*statusInput/2 + WALK_ACC) + ((cycleInput-O_WLP)/WALK_LIFT_PROP)*(WALK_RANGE*statusInput + WALK_ACC*2);
  }

  xGait = cos(rDiection) * rDist;
  zGait = sin(rDiection) * rDist;
  singleLegCtrl(LegNum, (xGait + extendedX), yGait, (zGait + extendedZ));
}


// 对角步态控制
// GlobalInput: 全局步态周期(0~1)
// directionAngle: 运动方向角度
// turnCmd: 转弯指令(-1=左转, 0=直行, 1=右转)
void simpleGait(float GlobalInput, float directionAngle, int turnCmd){
  float Group_A;
  float Group_B;

  Group_A = GlobalInput;
  Group_B = GlobalInput+0.5;
  if(Group_B>1){Group_B--;}

  if(!turnCmd){
    singleGaitCtrl(1, 1, Group_A, directionAngle,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1, Group_A, -directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1, Group_B, directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1, Group_B, -directionAngle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
  else if(turnCmd == -1){
    singleGaitCtrl(1, 1.5, Group_A, 90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1.5, Group_A, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1.5, Group_B, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1.5, Group_B, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
  else if(turnCmd == 1){
    singleGaitCtrl(1, 1.5, Group_A, -90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1.5, Group_A, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1.5, Group_B, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1.5, Group_B, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
}


// 三角步态生成器
void triangularGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;

  float aInput = 0;
  float bInput = 0;
  float adProp;

  StepB = GlobalInput;
  StepC = GlobalInput + 0.25;
  StepD = GlobalInput + 0.5;
  StepA = GlobalInput + 0.75;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  if(GlobalInput <= 0.25){
    adProp = GlobalInput;
    aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
    bInput = -WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.25 && GlobalInput <= 0.5){
    adProp = GlobalInput-0.25;
    aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
    bInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.5 && GlobalInput <= 0.75){
    adProp = GlobalInput-0.5;
    aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
    bInput =  WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.75 && GlobalInput <= 1){
    adProp = GlobalInput-0.75;
    aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
    bInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
  }

  if(!turnCmd){
    singleGaitCtrl(1, 1, StepA,  directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1, StepD, -directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1, StepB,  directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1, StepC, -directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
  else if(turnCmd == -1){
    singleGaitCtrl(1, 1.5, StepA,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
  else if(turnCmd == 1){
    singleGaitCtrl(1, 1.5, StepA, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
}


// 步态选择
void gaitTypeCtrl(float GlobalStepInput, float directionCmd, int turnCmd){
  if(GAIT_TYPE == 0){
    simpleGait(GlobalStepInput, directionCmd, turnCmd);
  }
  else if(GAIT_TYPE == 1){
    triangularGait(GlobalStepInput, directionCmd, turnCmd);
  }
}

void standMassCenter(float aInput, float bInput){
  singleLegCtrl(1, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));
  singleLegCtrl(2, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));

  singleLegCtrl(3, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
  singleLegCtrl(4, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
}


// 俯仰/偏航/横滚控制
// pitchInput: >0抬头, <0低头
// yawInput:   >0右转, <0左转
// rollInput:  >0右倾, <0左倾
void pitchYawRoll(float pitchInput, float yawInput, float rollInput){
  portENTER_CRITICAL(&ctrlMux);
  legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput;
  legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput;
  legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput;
  legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput;

  if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}

  if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}

  legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
  legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
  legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
  legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

  if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}

  if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}

  singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
  singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
  singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
  singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
  portEXIT_CRITICAL(&ctrlMux);
}


void pitchYawRollHeightCtrl(float pitchInput, float yawInput, float rollInput, float heightInput){
  portENTER_CRITICAL(&ctrlMux);
  legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput + heightInput;
  legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput + heightInput;
  legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput + heightInput;
  legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput + heightInput;

  if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}

  if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}

  legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
  legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
  legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
  legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

  if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}

  if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}

  singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
  singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
  singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
  singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
  portEXIT_CRITICAL(&ctrlMux);
}


// 平衡控制
void balancing(){
  // D项: 计算加速度变化率,抑制振荡
  float dPitch = (ACC_Y - BALANCE_PREV_ACC_Y) * BALANCE_D;
  float dRoll  = (ACC_X - BALANCE_PREV_ACC_X) * BALANCE_D;
  BALANCE_PREV_ACC_Y = ACC_Y;
  BALANCE_PREV_ACC_X = ACC_X;

  BALANCE_PITCH_BUFFER += ACC_Y * BALANCE_P - dPitch;
  BALANCE_ROLL_BUFFER  -= ACC_X * BALANCE_P + dRoll;

  if(BALANCE_PITCH_BUFFER > 21){BALANCE_PITCH_BUFFER = 21;}
  if(BALANCE_PITCH_BUFFER < -21){BALANCE_PITCH_BUFFER = -21;}

  if(BALANCE_ROLL_BUFFER > 21){BALANCE_ROLL_BUFFER = 21;}
  if(BALANCE_ROLL_BUFFER < -21){BALANCE_ROLL_BUFFER = -21;}

  pitchYawRoll(BALANCE_PITCH_BUFFER, 0, BALANCE_ROLL_BUFFER);
}


// 重心调整测试
void massCenerAdjustTestLoop(){
  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(-20+i, i);
    GoalPosAll();
    delay(5);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(i, 20-i);
    GoalPosAll();
    delay(5);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(20-i, -i);
    GoalPosAll();
    delay(5);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(-i, -20+i);
    GoalPosAll();
    delay(5);
  }
}


// 俯仰/偏航/横滚测试
void pitchYawRollTestLoop(){
  for(int i = -23; i<23; i++){
    pitchYawRoll(0, 0, i);
    GoalPosAll();
    delay(10);
  }
  for(int i = 23; i>-23; i--){
    pitchYawRoll(0, 0, i);
    GoalPosAll();
    delay(10);
  }

  for(int i = -23; i<23; i++){
    pitchYawRoll(i, 0, 0);
    GoalPosAll();
    delay(10);
  }
  for(int i = 23; i>-23; i--){
    pitchYawRoll(i, 0, 0);
    GoalPosAll();
    delay(10);
  }
}


// 关键帧插值函数
// rateInput: 0~1的插值比例
// 将numStart线性变化到numEnd
float linearCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*rateInput + numStart;
  return numOut;
}


// 贝塞尔插值: 余弦平滑过渡
float besselCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*((cos(rateInput*M_PI-M_PI)+1)/2) + numStart;
  return numOut;
}


// 动作函数
void functionStayLow(){
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i));
    GoalPosAll();
    delay(1);
  }
  delay(300);
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MAX, i));
    GoalPosAll();
    delay(1);
  }
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i));
    GoalPosAll();
    delay(1);
  }
}


void functionHandshake(){
  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), besselCtrl(WALK_EXTENDED_Z, -15, i));
    singleLegCtrl(3,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), WALK_EXTENDED_Z);

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));

    GoalPosAll();
    delay(1);
  }


  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(3,  besselCtrl(0, WALK_RANGE/2+WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT_MIN, i), besselCtrl(WALK_EXTENDED_Z, 0, i));

    GoalPosAll();
    delay(1);
  }

  for(int shakeTimes = 0; shakeTimes < 3; shakeTimes++){
    for(float i = 0; i<=1; i+=0.03){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MIN+30, i), 0);

      GoalPosAll();
      delay(1);
    }
    for(float i = 0; i<=1; i+=0.03){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN+30, WALK_HEIGHT_MIN, i), 0);

      GoalPosAll();
      delay(1);
    }
  }

  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1,  besselCtrl(0, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i), besselCtrl(-15, WALK_EXTENDED_Z, i));
    singleLegCtrl(3,  besselCtrl(WALK_RANGE/2+WALK_EXTENDED_X, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), besselCtrl(0, WALK_EXTENDED_Z, i));

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));

    GoalPosAll();
    delay(1);
  }
}

// 跳跃
void functionJump(){
  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(1);
  }

  singleLegCtrl(1, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(2,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(3, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(4,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  GoalPosAll();
  delay(70);

  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(1);
  }
}


// nod: 点头三次
void functionNod(){
  for(int n = 0; n < 3; n++){
    for(int step = 0; step <= 40; step++){
      float t = (float)step / 40.0;
      float pitch = sin(t * M_PI) * 20;
      singleLegCtrl(1,  WALK_EXTENDED_X, WALK_HEIGHT - pitch, WALK_EXTENDED_Z);
      singleLegCtrl(2, -WALK_EXTENDED_X, WALK_HEIGHT, WALK_EXTENDED_Z);
      singleLegCtrl(3,  WALK_EXTENDED_X, WALK_HEIGHT - pitch, WALK_EXTENDED_Z);
      singleLegCtrl(4, -WALK_EXTENDED_X, WALK_HEIGHT, WALK_EXTENDED_Z);
      GoalPosAll();
      delay(5);
    }
    delay(100);
  }
  standUp(WALK_HEIGHT);
  GoalPosAll();
  funcMode = 0;
}


// Hello: 右前腿抬起上下摆三次打招呼
void functionHello(){
  for(float i = 0; i <= 1; i += 0.03){
    singleLegCtrl(1,  WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(2, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));
    singleLegCtrl(3,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), besselCtrl(WALK_EXTENDED_Z, -15, i));
    singleLegCtrl(4, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));
    GoalPosAll();
    delay(1);
  }

  for(int wave = 0; wave < 3; wave++){
    for(float i = 0; i <= 1; i += 0.04){
      singleLegCtrl(3, WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MIN+30, i), 0);
      GoalPosAll();
      delay(1);
    }
    for(float i = 0; i <= 1; i += 0.04){
      singleLegCtrl(3, WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN+30, WALK_HEIGHT_MIN, i), 0);
      GoalPosAll();
      delay(1);
    }
  }

  for(float i = 0; i <= 1; i += 0.03){
    singleLegCtrl(1,  WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(2, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));
    singleLegCtrl(3,  besselCtrl(WALK_RANGE/2+WALK_EXTENDED_X, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), besselCtrl(0, WALK_EXTENDED_Z, i));
    singleLegCtrl(4, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));
    GoalPosAll();
    delay(1);
  }
  funcMode = 0;
}


// spin: 原地旋转360度
void functionSpin(){
  float spinStep = 0;
  for(int cycles = 0; cycles < 10; cycles++){
    for(spinStep = 0; spinStep < 1; spinStep += STEP_ITERATE){
      simpleGait(spinStep, 0, 1);
      GoalPosAll();
      delay(STEP_DELAY);
    }
  }
  standMassCenter(0, 0);
  GoalPosAll();
  funcMode = 0;
}


// crouch: 前腿蹲下后腿撑起
void functionCrouch(){
  for(float i = 0; i <= 1; i += 0.02){
    singleLegCtrl(1,  WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(3,  WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(2, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX-10, i), WALK_EXTENDED_Z);
    singleLegCtrl(4, -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX-10, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(2);
  }
  funcMode = 0;
}


// sit: 趴下(前腿向前伸后腿向后伸)
void functionSit(){
  for(float i = 0; i <= 1; i += 0.02){
    singleLegCtrl(1, besselCtrl(WALK_EXTENDED_X, 50, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(2, besselCtrl(-WALK_EXTENDED_X, -50, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, besselCtrl(WALK_EXTENDED_X, 50, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(4, besselCtrl(-WALK_EXTENDED_X, -50, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(2);
  }
  funcMode = 0;
}


// 主控制函数
void robotCtrl(){
  // 读取共享变量到局部副本(避免双核竞态)
  portENTER_CRITICAL(&ctrlMux);
  int _moveFB = moveFB;
  int _moveLR = moveLR;
  int _funcMode = funcMode;
  int _debugMode = debugMode;
  portEXIT_CRITICAL(&ctrlMux);

  // 运动控制
  if(!_debugMode && !_funcMode){
    if(_moveFB == 0 && _moveLR == 0 && STAND_STILL == 0){
      setSingleLED(0, matrix.Color(0, 255, 0));
      setSingleLED(1, matrix.Color(0, 255, 0));
      standMassCenter(0, 0);
      GoalPosAll();
      STAND_STILL = 1;
      GLOBAL_STEP = 0;
      delay(STEP_DELAY);
    }
    else if(_moveFB == 0 && _moveLR == 0 && STAND_STILL == 1){
      GoalPosAll();
      delay(STEP_DELAY);
    }
    else{
      STAND_STILL = 0;
      portENTER_CRITICAL(&ctrlMux);
      gestureUD = 0;
      gestureLR = 0;
      BALANCE_PITCH_BUFFER = 0;
      BALANCE_ROLL_BUFFER = 0;
      portEXIT_CRITICAL(&ctrlMux);
      if(_moveFB == 1 && _moveLR == 0){
        setSingleLED(0, matrix.Color(0, 128, 255)); setSingleLED(1, matrix.Color(0, 128, 255));
      }
      else if(_moveFB == -1 && _moveLR == 0){
        setSingleLED(0, matrix.Color(255, 160, 0)); setSingleLED(1, matrix.Color(255, 160, 0));
      }
      else if(_moveFB == 0 && _moveLR == -1){
        setSingleLED(0, matrix.Color(255, 255, 0)); setSingleLED(1, matrix.Color(255, 255, 0));
      }
      else if(_moveFB == 0 && _moveLR == 1){
        setSingleLED(0, matrix.Color(0, 255, 255)); setSingleLED(1, matrix.Color(0, 255, 255));
      }
      else if(_moveFB == 1 && _moveLR == -1){
        setSingleLED(0, matrix.Color(0, 200, 255)); setSingleLED(1, matrix.Color(0, 200, 255));
      }
      else if(_moveFB == 1 && _moveLR == 1){
        setSingleLED(0, matrix.Color(0, 255, 200)); setSingleLED(1, matrix.Color(0, 255, 200));
      }
      else if(_moveFB == -1 && _moveLR == 1){
        setSingleLED(0, matrix.Color(200, 128, 0)); setSingleLED(1, matrix.Color(200, 128, 0));
      }
      else if(_moveFB == -1 && _moveLR == -1){
        setSingleLED(0, matrix.Color(128, 200, 0)); setSingleLED(1, matrix.Color(128, 200, 0));
      }
      if(GLOBAL_STEP > 1){GLOBAL_STEP = 0;}
      if(_moveFB == 1 && _moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 0, 0);}
      else if(_moveFB == -1 && _moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 180, 0);}
      else if(_moveFB == 1 && _moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 30, 0);}
      else if(_moveFB == 1 && _moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -30, 0);}
      else if(_moveFB == -1 && _moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -120, 0);}
      else if(_moveFB == -1 && _moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 120, 0);}
      else if(_moveFB == 0 && _moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 0, -1);}
      else if(_moveFB == 0 && _moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, 0, 1);}
      GoalPosAll();
      GLOBAL_STEP += STEP_ITERATE;
      delay(STEP_DELAY);
    }
  }

  // 功能控制
  else if(!_debugMode && _funcMode){
    if(_funcMode == 1){
      extern void balanceRainbowLED();
      balanceRainbowLED();
      accXYZUpdate();
      balancing();
      GoalPosAll();
    }
    else if (_funcMode == 2){
      setSingleLED(0, matrix.Color(255, 200, 0)); setSingleLED(1, matrix.Color(255, 200, 0));
      Serial.println("stayLow");
      functionStayLow();
      funcMode = 0;
    }
    else if (_funcMode == 3){
      setSingleLED(0, matrix.Color(200, 0, 255)); setSingleLED(1, matrix.Color(200, 0, 255));
      Serial.println("handshake");
      functionHandshake();
      funcMode = 0;
    }
    else if (_funcMode == 4){
      setSingleLED(0, matrix.Color(255, 100, 0)); setSingleLED(1, matrix.Color(255, 100, 0));
      Serial.println("Jump");
      functionJump();
      funcMode = 0;
    }
    else if (_funcMode == 5){
      setSingleLED(0, matrix.Color(255, 0, 0)); setSingleLED(1, matrix.Color(255, 0, 0));
      Serial.println("nod");
      functionNod();
    }
    else if (_funcMode == 6){
      setSingleLED(0, matrix.Color(255, 0, 64)); setSingleLED(1, matrix.Color(255, 0, 64));
      Serial.println("Hello");
      functionHello();
    }
    else if (_funcMode == 7){
      setSingleLED(0, matrix.Color(0, 0, 255)); setSingleLED(1, matrix.Color(0, 0, 255));
      Serial.println("spin");
      functionSpin();
    }
    else if (_funcMode == 10){
      setSingleLED(0, matrix.Color(0, 64, 255)); setSingleLED(1, matrix.Color(0, 64, 255));
      Serial.println("Crouch");
      functionCrouch();
    }
    else if (_funcMode == 11){
      setSingleLED(0, matrix.Color(255, 64, 64)); setSingleLED(1, matrix.Color(255, 64, 64));
      Serial.println("getdown");
      functionSit();
    }
    else if (_funcMode == 8){
      setSingleLED(0, matrix.Color(255, 255, 255)); setSingleLED(1, matrix.Color(255, 255, 255));
      Serial.println("InitPos");
      initPosAll();
      funcMode = 0;
    }
    else if (_funcMode == 9){
      setSingleLED(0, matrix.Color(255, 180, 100)); setSingleLED(1, matrix.Color(255, 180, 100));
      Serial.println("MiddlePos");
      middlePosAll();
      funcMode = 0;
    }
  }
  else if(_debugMode){
    setSingleLED(0,matrix.Color(255, 64, 0));
    setSingleLED(1,matrix.Color(255, 64, 0));
    delay(100);
  }
}

// IO12为HIGH时进入调试模式
void wireDebugDetect(){
  if(digitalRead(WIRE_DEBUG) == HIGH){
    initPosAll();
    portENTER_CRITICAL(&ctrlMux);
    debugMode = 1;
    portEXIT_CRITICAL(&ctrlMux);

    setSingleLED(0,matrix.Color(255, 64, 0));
    setSingleLED(1,matrix.Color(255, 64, 0));

    while(digitalRead(WIRE_DEBUG) == HIGH){
      delay(100);
    }
    delay(1000);
  }
}
