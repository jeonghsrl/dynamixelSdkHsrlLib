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

bool configureVelocityMode(uint8_t id, uint32_t accel) {
  if (!setOperatingModeOnly(id, 1)) return false;
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION, accel, nullptr);
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

//--set parameter
bool setReverse(uint8_t id, bool enable_reverse) {
  uint8_t dxl_error = 0;
  uint8_t mode;

  // Torque OFF
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  // Drive Mode 読み取り
  packetHandler->read1ByteTxRx(portHandler, id, ADDR_DRIVE_MODE, &mode, &dxl_error);
  // Bit0 を書き換え
  mode = enable_reverse ? (mode | 0x01) : (mode & ~0x01);
  // Drive Mode 書き込み
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_DRIVE_MODE, mode, &dxl_error);
 // Torque ON
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  
  return true; // ※簡略化のため常に true

}

bool setTimeBasedProfile(uint8_t id, bool enable_time_based) {
  uint8_t dxl_error = 0;
  uint8_t mode;

  // Torque OFF
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  // 現在の Drive Mode を読み込み
  packetHandler->read1ByteTxRx(portHandler, id, ADDR_DRIVE_MODE, &mode, &dxl_error);
  // Bit2 を設定：1 = Time-based, 0 = Velocity-based
  mode = enable_time_based ? (mode | 0x04) : (mode & ~0x04);
  // Drive Mode を書き込み
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_DRIVE_MODE, mode, &dxl_error);
   // Torque ON
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  
  return true;  // ※簡略化のため常に true
}

//--------ReadWrite HommingOffset----------------------------
bool readHomingOffset(uint8_t id, int32_t* offset_out) {
  uint8_t dxl_error = 0;
  uint32_t value = 0;
  int result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_HOMING_OFFSET, &value, &dxl_error);

  if (result != COMM_SUCCESS) {
    Serial.print("Comm error (read): ");
    Serial.println(packetHandler->getTxRxResult(result));
    return false;
  }
  if (dxl_error != 0) {
    Serial.print("Dynamixel error (read): ");
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    return false;
  }

  *offset_out = (int32_t)value;
  return true;
}

bool writeHomingOffset(uint8_t id, int32_t offset_value) {
  uint8_t dxl_error = 0;

  torqueOFF(id);
  int result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_HOMING_OFFSET, offset_value, &dxl_error);

  if (result != COMM_SUCCESS) {
    Serial.print("Comm error (write): ");
    Serial.println(packetHandler->getTxRxResult(result));
    return false;
  }
  if (dxl_error != 0) {
    Serial.print("Dynamixel error (write): ");
    Serial.println(packetHandler->getRxPacketError(dxl_error));
    return false;
  }
  torqueON(id);

  return true;
}

bool readHomingOffsets(const uint8_t* ids, uint8_t id_count, int32_t* offsets_out) {
  for (uint8_t i = 0; i < id_count; i++) {
    uint8_t dxl_error = 0;
    uint32_t value = 0;

    int result = packetHandler->read4ByteTxRx(portHandler, ids[i], ADDR_HOMING_OFFSET, &value, &dxl_error);
    if (result != COMM_SUCCESS || dxl_error != 0) {
      Serial.print("Homing Offset read failed for ID "); Serial.println(ids[i]);
      return false;
    }

    offsets_out[i] = (int32_t)value;
  }

  return true;
}

bool writeHomingOffsets(const uint8_t* ids, uint8_t id_count, const int32_t* offsets) {
  for (uint8_t i = 0; i < id_count; i++) {
    uint8_t dxl_error = 0;

    // Torque OFF（必要）
    int result = packetHandler->write1ByteTxRx(portHandler, ids[i], ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (result != COMM_SUCCESS || dxl_error != 0) {
      Serial.print("Torque OFF failed for ID "); Serial.println(ids[i]);
      return false;
    }

    // Homing Offset 書き込み
    result = packetHandler->write4ByteTxRx(portHandler, ids[i], ADDR_HOMING_OFFSET, offsets[i], &dxl_error);
    if (result != COMM_SUCCESS || dxl_error != 0) {
      Serial.print("Homing Offset write failed for ID "); Serial.println(ids[i]);
      return false;
    }


     // Torque ON（必要）
     result = packetHandler->write1ByteTxRx(portHandler, ids[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (result != COMM_SUCCESS || dxl_error != 0) {
      Serial.print("Torque ON failed for ID "); Serial.println(ids[i]);
      return false;
    }
  }

   
  

  return true;
}












//---------Sync&Buld Read Write--------------------------------------
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

bool writeSyncGoalVelocityCount(const uint8_t* ids, uint8_t id_count, const int32_t* goal_velocites) {
  
  GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_VELOCITY);
  
  uint8_t param[LEN_VELOCITY];
  
  for (uint8_t i = 0; i < id_count; i++) {
    param[0] = goal_velocites[i] & 0xFF;
    param[1] = (goal_velocites[i] >> 8) & 0xFF;
    param[2] = (goal_velocites[i] >> 16) & 0xFF;
    param[3] = (goal_velocites[i] >> 24) & 0xFF;
    if (!groupSyncWrite.addParam(ids[i], param)) return false;
  }
  
  if (groupSyncWrite.txPacket() != COMM_SUCCESS) return false;
  groupSyncWrite.clearParam();

  return true;
}

bool writeSyncGoalCurrentCount(const uint8_t* ids, uint8_t id_count, const int32_t* goal_currents) {
  
  GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_CURRENT);
  
  uint8_t param[LEN_CURRENT];
  
  for (uint8_t i = 0; i < id_count; i++) {
    param[0] = goal_currents[i] & 0xFF;
    param[1] = (goal_currents[i] >> 8) & 0xFF;
    param[2] = (goal_currents[i] >> 16) & 0xFF;
    param[3] = (goal_currents[i] >> 24) & 0xFF;
    if (!groupSyncWrite.addParam(ids[i], param)) return false;
  }
  
  if (groupSyncWrite.txPacket() != COMM_SUCCESS) return false;
  groupSyncWrite.clearParam();

  return true;
}


bool readBulkPosVelCur(const uint8_t* ids, uint8_t id_count, int32_t* positions, int32_t* velocities, int16_t* currents) {

  GroupBulkRead groupBulkRead(portHandler, packetHandler);
  
  for (uint8_t i = 0; i < id_count; i++) {
    uint8_t id = ids[i]; 
    // readするパラメータの登録。 groupBulkでは、同じidに対しては複数のaddPramが使えない。
    if (!groupBulkRead.addParam(id, ADDR_PRESENT_CURRENT, 10)) return false;   //現在電流ADDR(2)+速度(4)+位置(4)=10
  }
  
   //登録したパラメータでbulk通信
  if (groupBulkRead.txRxPacket() != COMM_SUCCESS) return false; 

  //アドレスにデータがあれば、状態を取得
  for (uint8_t i = 0; i < id_count; i++) {
    uint8_t id = ids[i];
    if (!groupBulkRead.isAvailable(id, ADDR_PRESENT_CURRENT, 10)) return false;
    currents[i]  = groupBulkRead.getData(id, ADDR_PRESENT_CURRENT, LEN_CURRENT);
    velocities[i]= groupBulkRead.getData(id, ADDR_PRESENT_VELOCITY, LEN_VELOCITY);
    positions[i] = groupBulkRead.getData(id, ADDR_PRESENT_POSITION, LEN_POSITION);
  }

  //bulkread通信終了 ※必ずclearすること
  groupBulkRead.clearParam(); 

  return true;
}


float rawToDegree(int32_t pos_raw) {
  return (pos_raw * 360.0) / 4096.0;
}

int32_t degreeToRaw(float degree) {
  return round(degree * 4096.0 / 360.0);
}

float rawToVelocityDegPerSec(int32_t vel_raw) {
  return vel_raw * 0.229 * 6.0; // 0.229 RPM → deg/s
}

float rawToCurrentMilliAmp(int16_t curr_raw) {
  return curr_raw * 2.69;
}
