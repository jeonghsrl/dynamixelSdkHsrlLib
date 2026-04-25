#define PROTOCOL_VERSION 2.0
#define DEVICENAME       ""      // PCからopencrを使う場合は空欄で大丈夫 
#define BAUDRATE         1000000
//設定関連アドレス
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE  64
#define ADDR_CURRENT_LIMIT        38
#define ADDR_GOAL_CURRENT         102
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY     112
#define TORQUE_ENABLE       1
#define TORQUE_DISABLE      0
//状態取得、出力関連アドレス
#define ADDR_PRESENT_POSITION   132
#define ADDR_GOAL_POSITION      116
#define LEN_POSITION            4

// モードの列挙型（わかりやすく）
enum OperatingMode {
  CURRENT_MODE = 0,
  VELOCITY_MODE = 1,           
  POSITION_MODE = 3,
  EXTENDED_POSITION_MODE = 4,
  CURRENT_BASED_POSITION_MODE = 5
};

void dynamixelInitialize();
void recognitionDynamixel(uint8_t id);
//Mode設定
bool setOperatingModeOnly(uint8_t id, uint8_t mode);
bool configureCurrentMode(uint8_t id, uint16_t goal_current, uint16_t current_limit);
bool configureVelocityMode(uint8_t id, uint32_t accel, uint32_t velocity);
bool configurePositionMode(uint8_t id, uint32_t accel, uint32_t velocity);
bool configureExtendedPositionMode(uint8_t id, uint32_t accel, uint32_t velocity);
bool configureCurrentBasedPositionMode(uint8_t id, uint32_t accel, uint32_t velocity, uint16_t goal_current, uint16_t current_limit);
//状態取得&書き込み
bool readSyncPositionsCount(const uint8_t* ids, uint8_t id_count, int32_t* positions_out);
bool writeSyncGoalPositionsCount(const uint8_t* ids, uint8_t id_count, const int32_t* goal_positions);
float countToDegree(int32_t pos_raw);
int32_t degreeToCount(float degree);
