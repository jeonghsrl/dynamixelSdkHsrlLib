#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib.h"


// 使用するDynamixel ID（3つ）
const uint8_t DXL_ID_LIST[] = {1, 2, 3};
const uint8_t DXL_COUNT = 3;   //sizeof(DXL_ID_LIST) / sizeof(DXL_ID_LIST[0]);

using namespace dynamixel;        //dynamixel classを利用
// ハンドラ
PortHandler *portHandler;
PacketHandler *packetHandler;



void recognitionDynamixel(){
  // ハンドラ初期化
  portHandler = PortHandler::getPortHandler(DEVICENAME);  // "" でOpenCR自動認識
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // 通信初期化
  if (!portHandler->openPort()) {
    Serial.println("Failed to open port");
    while (1);
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Failed to set baudrate");
    while (1);
  }

  Serial.println("Starting Ping...");

  // 各IDにPingして接続確認
  for (int i = 0; i < DXL_COUNT; i++) {
    uint8_t dxl_id = DXL_ID_LIST[i];
    uint8_t dxl_error = 0;
    uint16_t model_number = 0;

    int dxl_comm_result = packetHandler->ping(portHandler, dxl_id, &model_number, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
      Serial.print("ID "); Serial.print(dxl_id); Serial.print(" found. Model Number: "); Serial.println(model_number);
      tone(BDPIN_BUZZER, 700, 150); 
    } else {
      Serial.print("ID "); Serial.print(dxl_id); Serial.println(" not found.");  tone(BDPIN_BUZZER, 100, 500);
    }
    delay(500);
  }

  
}
