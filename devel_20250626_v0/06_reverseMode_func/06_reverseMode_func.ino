#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib.h"

// 使用するDynamixel ID（3つ）
const uint8_t DXL_ID_LIST[] = {1, 2, 3};
const uint8_t DXL_COUNT = sizeof(DXL_ID_LIST) / sizeof(DXL_ID_LIST[0]);
int32_t positions[DXL_COUNT],velocities[DXL_COUNT];
int16_t currents[DXL_COUNT];

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //※必ず初期化
  dynamixelInitialize();
     
  //※Dynamixelの認識(ping)
  for(int i=0; i<DXL_COUNT; i++)  recognitionDynamixel(DXL_ID_LIST[i]);

  //※パラメータ込みで各モードに設定
 //configurePositionMode(DXL_ID_LIST[0], 20, 100);   //(id, 加速度プロファイル、速度プロファイル）1がリバース
 //configurePositionMode(DXL_ID_LIST[1], 20, 100);
 //configurePositionMode(DXL_ID_LIST[2], 20, 100);

 configureVelocityMode(DXL_ID_LIST[0], 20);   //(id, 加速度プロファイル、速度プロファイル）1がリバース
 configureVelocityMode(DXL_ID_LIST[1], 20);
 configureVelocityMode(DXL_ID_LIST[2], 20);

 //回転方向変更  CCW: モータを見て反時計方向を＋、CW：モータをみて時計方向を＋
 setReverse(DXL_ID_LIST[0],Rev_CCW);         
 setReverse(DXL_ID_LIST[1],Rev_CW);
 setReverse(DXL_ID_LIST[2],Rev_CCW);       

 //Time_Based_Profieの設定   Time Based =1, Velocity Profile=0. 
 //position, Extended Position, Current-based PositionのみTime_based が使える.  
 //setTimeBasedProfile(DXL_ID_LIST[0], false);
 //setTimeBasedProfile(DXL_ID_LIST[1], false);
// setTimeBasedProfile(DXL_ID_LIST[2], false);
           
}


void loop() {

 int32_t goals[DXL_COUNT] = {20,20,20};
 
  if (!writeSyncGoalVelocityCount(DXL_ID_LIST, DXL_COUNT,goals)) {
    Serial.println("Failed to write goal positions");
  }
  
/*
  if (!writeSyncGoalPositionsCount(DXL_ID_LIST, DXL_COUNT,goals)) {
    Serial.println("Failed to write goal positions");
  }
  */
  
  if (readBulkPosVelCur(DXL_ID_LIST, DXL_COUNT, positions, velocities, currents)) {
    for (int i = 0; i < DXL_COUNT; i++) {
      Serial.print("ID "); Serial.println(DXL_ID_LIST[i]);
      Serial.print("  Pos[deg]: ");
      Serial.println(rawToDegree(positions[i]), 2);
      Serial.print("  Vel[deg/s]: ");
      Serial.println(rawToVelocityDegPerSec(velocities[i]), 2);
      Serial.print("  Curr[mA]: ");
      Serial.println(rawToCurrentMilliAmp(currents[i]), 2);
    }
  } else {
   Serial.println("Bulk read failed");
  }
  
  
  delay(50);
  
  
}
