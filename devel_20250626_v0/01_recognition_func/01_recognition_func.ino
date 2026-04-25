#include <DynamixelSDK.h>
#include "dynamixelSdkHsrlLib.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);

  recognitionDynamixel();
  
}

void loop() {
  // 何もしない
}
