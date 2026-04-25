#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib.h"

using namespace dynamixel;        //dynamixel classを利用
// ハンドラ
PortHandler *portHandler;
PacketHandler *packetHandler;

int dxl_comm_result;    //communication error 処理用
uint8_t dxl_error;
  
void dynamixelInitialize(){

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

}

void recognitionDynamixel(uint8_t id){
  
    Serial.println("Starting Ping...");
    uint16_t model_number = 0;

    int dxl_comm_result = packetHandler->ping(portHandler, id, &model_number, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
      Serial.print("ID "); Serial.print(id); Serial.print(" found. Model Number: "); Serial.println(model_number);
      tone(BDPIN_BUZZER, 700, 150); 
    } else {
      Serial.print("ID "); Serial.print(id); Serial.println(" not found.");  tone(BDPIN_BUZZER, 100, 500);
    }
    delay(500);  
}

// トルクOFF → モード設定 → トルクON
bool setOperatingModeOnly(uint8_t id, uint8_t  mode) {
  // トルクOFF
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) return false;

  // モード設定
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, mode, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS){Serial.println("Failed to set mode."); return false;}

  // トルクON
//  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
 // if (dxl_comm_result != COMM_SUCCESS) return false;

  Serial.print("Secceed to set mode:"); Serial.println(mode);  tone(BDPIN_BUZZER, 500, 150); 
  delay(500);  
  return true;
}

bool configureCurrentMode(uint8_t id, uint16_t goal_current, uint16_t current_limit) {
  
  if (!setOperatingModeOnly(id, 0)) return false;
  packetHandler->write2ByteTxRx(portHandler, id, ADDR_CURRENT_LIMIT, current_limit, nullptr);
  packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_CURRENT, goal_current, nullptr);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, nullptr);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) return false;
  return true;
}

bool configureVelocityMode(uint8_t id, uint32_t accel, uint32_t velocity) {
  if (!setOperatingModeOnly(id, 1)) return false;
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION, accel, nullptr);
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, velocity, nullptr);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, nullptr);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) return false;
  return true;
}

bool configurePositionMode(uint8_t id, uint32_t accel, uint32_t velocity) {
  if (!setOperatingModeOnly(id, 3)) return false;
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION, accel, nullptr);
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, velocity, nullptr);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, nullptr);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) return false;
  return true;
}

bool configureExtendedPositionMode(uint8_t id, uint32_t accel, uint32_t velocity) {
  if (!setOperatingModeOnly(id, 4)) return false;
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION, accel, nullptr);
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, velocity, nullptr);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, nullptr);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) return false;
  return true;
}

bool configureCurrentBasedPositionMode(uint8_t id, uint32_t accel, uint32_t velocity, uint16_t goal_current, uint16_t current_limit) {
  if (!setOperatingModeOnly(id, 5)) return false;
  packetHandler->write2ByteTxRx(portHandler, id, ADDR_CURRENT_LIMIT, current_limit, nullptr);
  packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_CURRENT, goal_current, nullptr);
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION, accel, nullptr);
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, velocity, nullptr);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, nullptr);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) return false;
  return true;
}
