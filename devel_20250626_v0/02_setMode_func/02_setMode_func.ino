#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib.h"

// 使用するDynamixel ID（3つ）
const uint8_t DXL_ID_LIST[] = {1, 2, 3};
const uint8_t DXL_COUNT = 3;   //sizeof(DXL_ID_LIST) / sizeof(DXL_ID_LIST[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //必ず初期化
  dynamixelInitialize();
     
  //Dynamixelの認識(ping)
  for(int i=0; i<DXL_COUNT; i++)  recognitionDynamixel(DXL_ID_LIST[i]);

 //パラメータ込みで各モードに設定
 configurePositionMode(DXL_ID_LIST[0], 20, 100);
 configurePositionMode(DXL_ID_LIST[1], 20, 100);
 configurePositionMode(DXL_ID_LIST[2], 20, 100);
  
}

void loop() {
  // 何もしない
}
