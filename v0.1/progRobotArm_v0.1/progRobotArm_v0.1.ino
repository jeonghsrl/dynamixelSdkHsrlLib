#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib_v0.1.h"
#include "dxlrobotSetup_v0.1.h"

HardwareTimer Timer(TIMER_CH1); //Periodic timerloop

extern DYNAMIXEL_JOINT J[Jnum+1];

// 使用するDynamixel ID（3つ）
const uint8_t dxlIds[] = {J1_id, J2_id, J3_id};
const uint8_t dxlCnt = sizeof(dxlIds) / sizeof(dxlIds[0]);

int32_t goals[dxlCnt] = {0,0,0};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //ジョイントセットアップ
  rb_setupParameter();
  
  //※dynamixel必ず初期化
  dynamixelInitialize();
  
  //※Dynamixelの認識(ping)
  for(int i=0; i<dxlCnt; i++)  recognitionDynamixel(dxlIds[i]);

 /* //※パラメータ込みでposition mode設定
  configurePositionMode(J[J1].id, 20, 50);   //(id, 加速度プロファイル、速度プロファイル）1がリバース
  configurePositionMode(J[J2].id, 20, 50);
  configurePositionMode(J[J3].id, 20, 50);
 */

  rb_calBulkPosVelCur(dxlIds, dxlCnt);     //全状態計算

  rb_setExtendedPositonModeWithHomingOffset(J[J1].jn, 40, 50, OffsetDxlAng_J1, InitAng_J1);
  rb_setExtendedPositonModeWithHomingOffset(J[J2].jn, 40, 50, OffsetDxlAng_J2, InitAng_J2);
  rb_setExtendedPositonModeWithHomingOffset(J[J3].jn, 40, 50, OffsetDxlAng_J3, InitAng_J3);
 
  rb_calBulkPosVelCur(dxlIds, dxlCnt);     //全状態計算
   
  rb_moveToGoalJointAngleWithPause(J[J1].id, 0, 0.1 );  //１関節移動
  rb_moveToGoalJointAngleWithPause(J[J2].id, 0, 0.1 ); 
  rb_moveToGoalJointAngleWithPause(J[J3].id, 0, 0.1 ); 
  
  Serial.println("Initilizing Finish");

  rb_calBulkPosVelCur(dxlIds, dxlCnt);     //全状態取得

  for(uint8_t i=1;i<4;i++){
   Serial.print("posraw.n: "); Serial.print(J[i].posraw.n);
   Serial.print("ang.n: ");Serial.print(J[i].ang.n);
   Serial.print("th.n: ");Serial.println(J[i].th.n);
   }

  delay(3000);

//-TimerLoop----------------
  Timer.stop();
  Timer.setPeriod(TIMER_RATE);           // in microseconds
  Timer.attachInterrupt(timerloop);
  Timer.start(); 
//-----------------------

           
}


void loop() {

// setGoalAngle2Raw();
  goals[0]=degreeToRaw(45);
  goals[1]=degreeToRaw(45);
  goals[2]=degreeToRaw(45);

 delay(3000);

  goals[0]=degreeToRaw(-45);
  goals[1]=degreeToRaw(-45);
  goals[2]=degreeToRaw(-45);

 delay(3000);

}


void timerloop(void) {
  
  //すべての関節の角度、角速度、電流値を計算
  rb_calBulkPosVelCur(dxlIds, dxlCnt);        
     
  /* //　Wheel modeでの目標回転速度指令
  if (!writeSyncGoalVelocityCount(dxlIds, dxlCnt,goals)) {
    Serial.println("Failed to write goal positions");
  }
  */

 /* // Current modeでの目標電流指令
  if (!writeSyncGoalCurrentCount(dxlIds, dxlCnt,goals)) {
    Serial.println("Failed to write goal positions");
  }
  */ 
   // Position modeでの目標位置指令
  if (!writeSyncGoalPositionsCount(dxlIds, dxlCnt,goals)) {
    Serial.println("Failed to write goal positions");
  }
   
   for(uint8_t i=1;i<4;i++){
   Serial.print("posraw.n: "); Serial.print(J[i].posraw.n);
   Serial.print("ang.n: ");Serial.print(J[i].ang.n);
   Serial.print("th.n: ");Serial.println(J[i].th.n);
   //  Serial.print("velraw.n: "); Serial.print(J[J1].velraw.n);
   //   Serial.print("dang.n: ");Serial.print(J[J1].dang.n);
   //  Serial.print("dth.n: ");Serial.print(J[J1].dth.n);
   //  Serial.print("curraw.n: "); Serial.print(J[J1].curraw.n);
   //  Serial.print("curmA.n: ");Serial.println(J[J1].curmA.n);
 }

}
