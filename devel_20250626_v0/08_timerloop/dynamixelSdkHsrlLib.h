#define PROTOCOL_VERSION 2.0
#define DEVICENAME       ""      // PCからopencrを使う場合は空欄で大丈夫 
#define BAUDRATE         1000000
//設定関連アドレス
#define ADDR_DRIVE_MODE     10    //reverse mode設定など
#define ADDR_OPERATING_MODE 11    //position, velocidty ...mode設定など
#define ADDR_TORQUE_ENABLE  64
#define ADDR_CURRENT_LIMIT        38
#define ADDR_HOMING_OFFSET    20
#define ADDR_GOAL_CURRENT         102
#define ADDR_GOAL_VELOCITY        104
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY     112
#define TORQUE_ENABLE       1
#define TORQUE_DISABLE      0
//状態取得、出力関連アドレス
#define ADDR_PRESENT_POSITION   132
#define ADDR_PRESENT_CURRENT    126
#define ADDR_PRESENT_VELOCITY   128
#define ADDR_GOAL_POSITION      116
#define LEN_VELOCITY    4
#define LEN_POSITION    4
#define LEN_CURRENT     2
#define LEN_HOMING_OFFSET     4

#define Rev_CCW      0
#define Rev_CW       1

#define TIMER_RATE   20000    //timerloop rate microseconds

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
bool configureVelocityMode(uint8_t id, uint32_t accel);
bool configurePositionMode(uint8_t id, uint32_t accel, uint32_t velocity);
bool configureExtendedPositionMode(uint8_t id, uint32_t accel, uint32_t velocity);
bool configureCurrentBasedPositionMode(uint8_t id, uint32_t accel, uint32_t velocity, uint16_t goal_current, uint16_t current_limit);

//set parameter
bool setReverse(uint8_t id, bool enable_reverse);             //回転方向変更  CCW: モータを見て反時計方向を＋、CW：モータをみて時計方向を＋
bool setTimeBasedProfile(uint8_t id, bool enable_time_based); // Time Based =1, Velocity Profile:0 


//ReadWrite HomingOffset
bool readHomingOffset(uint8_t id, int32_t* offset_out);    //read HomingOffset
bool writeHomingOffset(uint8_t id, int32_t offset_value);  //write HomingOffset
bool readHomingOffsets(const uint8_t* ids, uint8_t id_count, int32_t* offsets_out); //複数
bool writeHomingOffsets(const uint8_t* ids, uint8_t id_count, const int32_t* offsets); 

//状態取得&書き込み
bool readSyncPositionsCount(const uint8_t* ids, uint8_t id_count, int32_t* positions_out);
bool writeSyncGoalPositionsCount(const uint8_t* ids, uint8_t id_count, const int32_t* goal_positions);
bool writeSyncGoalVelocityCount(const uint8_t* ids, uint8_t id_count, const int32_t* goal_velocites);
bool readBulkPosVelCur(const uint8_t* ids, uint8_t id_count, int32_t* positions, int32_t* velocities, int16_t* currents);

float rawToDegree(int32_t pos_raw);
int32_t degreeToRaw(float degree);
float rawToVelocityDegPerSec(int32_t vel_raw);
float rawToCurrentMilliAmp(int16_t curr_raw);
