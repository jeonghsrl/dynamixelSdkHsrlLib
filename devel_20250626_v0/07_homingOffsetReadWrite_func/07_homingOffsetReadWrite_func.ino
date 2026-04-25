#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib.h"

// 使用するDynamixel ID（3つ）
const uint8_t DXL_ID_LIST[] = {1, 2, 3};
const uint8_t DXL_COUNT = sizeof(DXL_ID_LIST) / sizeof(DXL_ID_LIST[0]);
int32_t positions[DXL_COUNT],velocities[DXL_COUNT];
int16_t currents[DXL_COUNT];
int32_t homingoffsets[DXL_COUNT]={0,0,0};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  //※必ず初期化
  dynamixelInitialize();
     
  //※Dynamixelの認識(ping)
  for(int i=0; i<DXL_COUNT; i++)  recognitionDynamixel(DXL_ID_LIST[i]);

  //※パラメータ込みで各モードに設定
 configurePositionMode(DXL_ID_LIST[0], 20, 100);   //(id, 加速度プロファイル、速度プロファイル）1がリバース
 configurePositionMode(DXL_ID_LIST[1], 20, 100);
 configurePositionMode(DXL_ID_LIST[2], 20, 100);

 //configureVelocityMode(DXL_ID_LIST[0], 20);   //(id, 加速度プロファイル、速度プロファイル）1がリバース
 //configureVelocityMode(DXL_ID_LIST[1], 20);
 //configureVelocityMode(DXL_ID_LIST[2], 20);

/* 1つずつの場合
  for (int i = 0; i < DXL_COUNT; i++) {            //現在位置が「0度」の状態で、homingを1023(90度)に設定すると、
   writeHomingOffset(DXL_ID_LIST[i], 1023);        //現在位置が（0+90度）=90度に変わる。
   Serial.print("ID "); Serial.println(DXL_ID_LIST[i]); //したがって、目標角度「0」を与えると、-90度回転する。
   Serial.print("  HomingOffset set "); 
  } 
*/

  writeHomingOffsets(DXL_ID_LIST, DXL_COUNT, homingoffsets);
  Serial.print("  HomingOffsets set ");
  delay(1000);
           
}


void loop() {

 int32_t goals[DXL_COUNT] = {0,0,0};
 /*
  if (!writeSyncGoalVelocityCount(DXL_ID_LIST, DXL_COUNT,goals)) {
    Serial.println("Failed to write goal positions");
  }
  */

  if (!writeSyncGoalPositionsCount(DXL_ID_LIST, DXL_COUNT,goals)) {
    Serial.println("Failed to write goal positions");
  }
  
  writeHomingOffsets(DXL_ID_LIST, DXL_COUNT, homingoffsets);
  
  if (readBulkPosVelCur(DXL_ID_LIST, DXL_COUNT, positions, velocities, currents)) {
    for (int i = 0; i < DXL_COUNT; i++) {
      Serial.print("ID "); Serial.println(DXL_ID_LIST[i]);
      Serial.print("  Pos[deg]: ");
      Serial.println(rawToDegree(positions[i]), 2);
      Serial.print("  Vel[deg/s]: ");
      Serial.println(rawToVelocityDegPerSec(velocities[i]), 2);
      Serial.print("  Curr[mA]: ");
      Serial.println(rawToCurrentMilliAmp(currents[i]), 2);
      
      //readHomingOffset(DXL_ID_LIST[i], &homingoffsets[i]); 
      Serial.print("  HomingOffset: "); 
      Serial.println(homingoffsets[i]);
      
  } 
  } else {
   Serial.println("Bulk read failed");
  }




  delay(50);
  
  
}
