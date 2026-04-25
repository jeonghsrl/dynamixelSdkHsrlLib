#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib.h"

// 使用するDynamixel ID（3つ）
const uint8_t DXL_ID_LIST[] = {1, 2, 3};
const uint8_t DXL_COUNT = 3;   //sizeof(DXL_ID_LIST) / sizeof(DXL_ID_LIST[0]);
int32_t positions[DXL_COUNT];


void setup() {
  Serial.begin(115200);
  while (!Serial);

  //必ず初期化
  dynamixelInitialize();
     
  //Dynamixelの認識(ping)
  for(int i=0; i<DXL_COUNT; i++)  recognitionDynamixel(DXL_ID_LIST[i]);

 //パラメータ込みで各モードに設定
 configurePositionMode(DXL_ID_LIST[0], 20, 100);   //(id, 加速度プロファイル、速度プロファイル）
 configurePositionMode(DXL_ID_LIST[1], 20, 100);
 configurePositionMode(DXL_ID_LIST[2], 20, 100);
                   
}

void loop() {

  if (readSyncPositionsCount(DXL_ID_LIST, DXL_COUNT, positions)) {
    Serial.println("Current Positions:");
    for (uint8_t i = 0; i < DXL_COUNT; i++) {
      Serial.print("ID ");
      Serial.print(DXL_ID_LIST[i]);
      Serial.print(": ");
      Serial.println(positions[i]);
    }
  } else {
    Serial.println("Failed to read positions");
  }

  int32_t goals[DXL_COUNT] = {500, 1000, 1500};
  if (!writeSyncGoalPositionsCount(DXL_ID_LIST, DXL_COUNT,goals)) {
    Serial.println("Failed to write goal positions");
  }

  delay(1000);
  
}
