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


//-------------Torque ON/OFF
bool torqueON(uint8_t id){
    // トルクON
   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) return false;
}

bool torqueOFF(uint8_t id){
    //トルクOFF
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
   if (dxl_comm_result != COMM_SUCCESS) return false;
}

//------ Mode設定
bool setOperatingModeOnly(uint8_t id, uint8_t  mode) {
  // トルクOFF
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) return false;

  // モード設定
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, mode, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS){Serial.println("Failed to set mode."); return false;}

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

//----------------------------------------------------

bool readSyncPositionsCount(const uint8_t* ids, uint8_t id_count, int32_t* positions_out) {

  GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_POSITION);

  for (uint8_t i = 0; i < id_count; i++) {
    if (!groupSyncRead.addParam(ids[i])) return false;
  }

  if (groupSyncRead.txRxPacket() != COMM_SUCCESS) return false;

  for (uint8_t i = 0; i < id_count; i++) {
    if (!groupSyncRead.isAvailable(ids[i], ADDR_PRESENT_POSITION, LEN_POSITION)) return false;
    positions_out[i] = groupSyncRead.getData(ids[i], ADDR_PRESENT_POSITION, LEN_POSITION);
  }

  return true;
}

bool writeSyncGoalPositionsCount(const uint8_t* ids, uint8_t id_count, const int32_t* goal_positions) {
  
  GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_POSITION);
  
  uint8_t param[LEN_POSITION];
  
  for (uint8_t i = 0; i < id_count; i++) {
    param[0] = goal_positions[i] & 0xFF;
    param[1] = (goal_positions[i] >> 8) & 0xFF;
    param[2] = (goal_positions[i] >> 16) & 0xFF;
    param[3] = (goal_positions[i] >> 24) & 0xFF;
    if (!groupSyncWrite.addParam(ids[i], param)) return false;
  }
  
  if (groupSyncWrite.txPacket() != COMM_SUCCESS) return false;
  groupSyncWrite.clearParam();

  return true;
}

float countToDegree(int32_t pos_raw) {
  return (pos_raw - 2048) * 360.0 / 4096.0;
}

int32_t degreeToCount(float degree) {
  return round(degree * 4096.0 / 360.0) + 2048;
}
