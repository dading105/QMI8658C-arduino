/*
   @Description: QMI8658C
   @Author: ELEGOO
   @Date: 2019-07-10 16:46:17
   @LastEditTime: 2021-04-26 16:25:22
   @LastEditors: Changhua
*/

#include "QMI8658C.h"
#include "I2Cdev.h"
QMI8658C _QMI8658C;

void setup()
{
  Serial.begin(115200);
  _QMI8658C.QMI8658C_dveInit();
}

void loop()
{
  float pitch, roll, yaw;
  uint8_t cmd[] = {AccX_L, AccY_L, AccZ_L, TEMP_L,GyrX_L, GyrY_L, GyrZ_L};
  char cmdStr[][10] = {"  AcX:", "  AcY:", "  AcZ:","  Tmp:", "  GyX:", "  GyY:", "  GyZ:"};
  static unsigned long Test_time; //获取时间戳 timestamp
  if (millis() - Test_time > 50)
  {
    // for (int i = 0; i < 7; i++)
    // {
    //   Serial.print(cmdStr[i]);
    //   if(i==3)
    //   {
    //     Serial.print(((QMI8658C_readBytes(cmd[i]))/256)-5.2);
    //   }
    //   else
    //   {
    //     Serial.print(QMI8658C_readBytes(cmd[i]));
    //   }   
    //   Serial.print("  |");
    // }
    // Serial.println();
    if (_QMI8658C.getAttitude(&pitch, &roll, &yaw)) {
        Serial.print("Pitch: "); Serial.print(pitch);
        Serial.print(" Roll: "); Serial.print(roll);
        Serial.print(" Yaw: "); Serial.println(yaw);
    }
    Test_time = millis();
  }
}
