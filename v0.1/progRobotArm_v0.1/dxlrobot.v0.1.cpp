#include <DynamixelWorkbench.h>
#include "dynamixelSdkHsrlLib_v0.1.h"
#include "dxlrobotSetup_v0.1.h"

DYNAMIXEL_JOINT J[Jnum+1];

void rb_setupParameter(){

 J[J1].jn   = J1;
 J[J1].id   = J1_id;
 J[J1].dir = ReverseMode_J1; 

 J[J2].jn   = J2;
 J[J2].id   = J2_id;
 J[J2].dir = ReverseMode_J2; 

 J[J3].jn   = J3;
 J[J3].id   = J3_id;
 J[J3].dir = ReverseMode_J3; 

}


//---   Homing Offset設定込みのExtendedPositionMode設定
void rb_setExtendedPositonModeWithHomingOffset(uint8_t jn, uint32_t accProfile, uint32_t velProfile, float OffsetDxlAng, float InitAng){
    writeHomingOffset(J[jn].id, 0);
    configureExtendedPositionMode(J[jn].id, accProfile, velProfile);
    J[jn].dxlHomingoffset= -1*J[jn].dir*degreeToRaw(OffsetDxlAng)+degreeToRaw(InitAng);  //dynamixelの90度を、関節角度180度に設定
    writeHomingOffset(J[jn].id, J[jn].dxlHomingoffset);
}

//------------- Interface 
void rb_calBulkPosVelCur(const uint8_t* ids, uint8_t id_count){

  int32_t positions[id_count],velocities[id_count];
  int16_t currents[id_count];

  readBulkPosVelCur(ids, id_count, positions, velocities, currents);

  for (uint8_t i = 0; i < id_count; i++){
   J[i+1].posraw.n = positions[i];
   J[i+1].ang.n = rawToDegree(positions[i]); 
   J[i+1].th.n = J[i+1].ang.n*Deg2Rad; 
   J[i+1].velraw.n = velocities[i];
   J[i+1].dang.n = rawToDegree(velocities[i]); 
   J[i+1].dth.n = J[i+1].dang.n*Deg2Rad; 
   J[i+1].curraw.n = currents[i];
   J[i+1].curmA.n = rawToCurrentMilliAmp(currents[i]); 
  }
}


void rb_moveToGoalJointAngleWithPause(uint8_t id, float degree, float ang_thres ){

  J[id].ang.g = degree;
  
  moveRawPosition(id,degreeToRaw(J[id].ang.g));  

  while(fabs(J[id].ang.g-J[id].ang.n) > ang_thres){
      rb_calOneJointAngle(id);    //DXLパルスから現在角度取得      
    }
    #ifdef __TONE_ON 
    tone(BDPIN_BUZZER, 700, 150);
   #endif 
   
}

void rb_moveToGoalJointAngle(uint8_t id, float degree){

  J[id].ang.g = degree;
  
  moveRawPosition(id,degreeToRaw(J[id].ang.g));  

    #ifdef __TONE_ON 
    tone(BDPIN_BUZZER, 700, 150);
   #endif 
   
}


void rb_moveGoalJointAngVel(uint8_t id, float angVel){

  J[id].dang.g = angVel;
  
  moveRawAngVel(id,degreeToRaw(J[id].dang.g));  

    #ifdef __TONE_ON 
    tone(BDPIN_BUZZER, 700, 150);
   #endif 
   
}

void rb_moveGoalCurrent_mA(uint8_t id, float current_mA){

  J[id].curmA.g = current_mA;
  
  moveRawAngVel(id,currentMilliAmpToRaw(J[id].curmA.g));  

    #ifdef __TONE_ON 
    tone(BDPIN_BUZZER, 700, 150);
   #endif 
   
}




bool rb_calOneJointAngle(uint8_t id) {
  uint8_t dxl_error = 0;
  int32_t value = 0;
  
  readRawPosition(id,&value);

  J[id].posraw.n = value;
  J[id].ang.n = rawToDegree(value); 
  J[id].th.n = J[id].ang.n*Deg2Rad; 
  Serial.print(J[id].ang.n);   Serial.print("ang.g"); Serial.println(J[id].ang.g);
}


 
