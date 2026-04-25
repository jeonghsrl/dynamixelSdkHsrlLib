//-- dxlrobotSetup.h 
#define TIMER_RATE   20000    //timerloop rate microseconds
#define __TONE_ON
#define Deg2Rad      0.0174   


#define Jnum 3       //全体ジョイント数
#define Jnum_S  1     //実行したい関節の開始番号
#define Jnum_F  3     //実行したい関節の終了番号


// Joint Number
#define J1   1     //Joint 1 (XH540)
#define J2   2     //Joint 2 (XH540)
#define J3   3     //Joint 3 (XH540)  
//#define J4   4     //Joint 4 (XH540)  
//#define J5   5     //Joint 5 (XH540)
//#define J6   6     //Joint 6 (XH540)  

// Dynamixel Joint ID
#define J1_id     1        //XH540-W270-R ( rpm, Nm) 
#define J2_id     2        //XH540-W150-R ( rpm, Nm) 
#define J3_id     3        //XH540-W150-R ( rpm, Nm) 
//#define J4_id     4        //XH540-W270-R ( rpm, Nm) 
//#define J5_id     5        //XH540-W150-R ( rpm, Nm) 
//#define J6_id     6        //XH540-W150-R ( rpm, Nm) 


/*----homingoffsetを使ってoffset角度を任意の角度に設定する場合----------------*/
//関節を手動で望ましい姿勢にした状態で、プログラムを実行。その時の角度を記入）
#define OffsetDxlAng_J1 180
#define OffsetDxlAng_J2 180
#define OffsetDxlAng_J3 180
//#define OffsetDxlAng_J4 0
//#define OffsetDxlAng_J5 0
//#define OffsetDxlAng_J6 0


//----初期姿勢角度 (上の姿勢になったときに、関節角度を何度にしたいかを記入)
#define InitAng_J1 0       
#define InitAng_J2 0
#define InitAng_J3 0
//#define InitAng_J4 0 
//#define InitAng_J5 0 
//#define InitAng_J6 90


//Reverse mode設定:-1  設定してない：1
#define ReverseMode_J1 1       
#define ReverseMode_J2 -1 
#define ReverseMode_J3 1
//#define ReverseMode_J4 1 
//#define ReverseMode_J5 1 
//#define ReverseMode_J6 1  


void SetupParameter();

/* state valuable struct */
typedef struct __state{
  double statetime;           //time

  double offset;               //オフセット値
  double lim_max,lim_min;      //limitValue value
  double ori;
  double init;
  double i,i_buf;              //initial value
  double n,n_buf;              //now value
  double d,d_buf;              //desired value       
  
  double s,s_buf;              // start value
  double f,f_buf;              // final value
  double g,g_buf;              // goal value
  
}STATE;


/* Dynamixel Joint Struct */
typedef struct __dynaJoint_type{

  uint8_t jn;
  uint8_t id;           
  int8_t  dir;

  int32_t dxlHomingoffset;     //dynamixel homming offset
  STATE posraw,velraw,accRaw;         // dynamixel raw 
  STATE dxlAng,dxlVel,dxlAcc;         // dynamixel angle
  STATE curraw;                    // dynamixel current
 
  double dynPul2Deg, dynDeg2Pul;      // dynamixel pulse angle 変換係数 

  STATE curmA; 
  STATE th,dth,ddth;                  // system coord. angle [rad]        
  STATE ang,dang,ddang;               // system coord. angle [deg] 
  STATE x,dx,ddx;                     // system coord. dist  [mm]                                                    

  double gearRatio;                  // dxl to system 状態変換係数（ギア比など）    
  uint8_t revmode;                   // reverse mode 状態変数
 
  //pid 
  double kp_j, ki_j, kd_j;           /* PID gain for joint control */ 

}DYNAMIXEL_JOINT;


//----------------------------------------
void rb_setupParameter();
void rb_calBulkPosVelCur(const uint8_t* ids, uint8_t id_count);  //全関節の関節状態計算
bool rb_calOneJointAngle(uint8_t id);     //１関節のみの関節位置・速度計算 
void rb_moveToGoalJointAngleWithPause(uint8_t id, float ang_g, float ang_thres ); //目標関節位置になるまで待つ
void rb_moveToGoalJointAngle(uint8_t id, float ang_g); //目標関節位置degまで移動(※動作未確認)
void rb_moveGoalJointAngVel(uint8_t id, float angVel); //目標角速度deg/sで移動 (※動作未確認）
void rb_moveGoalCurrent_mA(uint8_t id, float current_mA);// 目標電流mAで移動  (※動作未確認)
void rb_setExtendedPositonModeWithHomingOffset(uint8_t jn, uint32_t accProfile, uint32_t velProfile, float OffsetDxlAng, float InitAng);
