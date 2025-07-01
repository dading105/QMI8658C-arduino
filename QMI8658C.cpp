/*
 * @Description: QMI8658C
 * @Author: ELEGOO
 * @Date: 2019-07-10 16:46:17
 * @LastEditTime: 2021-04-23 15:08:40
 * @LastEditors: Changhua
 */
#include "QMI8658C.h"
#include "I2Cdev.h"



/*
  QMI8658C UI Sensor Configuration Settings and Output Data
----------------------------------------------------------------------------------------------*/
int16_t QMI8658C_readBytes(unsigned char tmp)
{
  uint8_t buffer[14];
  I2Cdev::readBytes(address, tmp, 2, buffer);
  return ((buffer[1] << 8) | buffer[0]);
}
bool QMI8658C::QMI8658C_dveInit(void)
{
  Wire.begin();
  delay(1000);
  uint8_t chip_id = 0x00;
  do
  {
    I2Cdev::readBytes(address, WHO_AM_I, 1, &chip_id);
    Serial.print("WHO_AM_I: 0X");
    Serial.println(chip_id, HEX);
    delay(10);
  } while (chip_id == 0); //确保从机设备在线（强行等待 获取 ID ）

  I2Cdev::writeByte(address, CTRL1, 0x40); //Serial Interface and Sensor Enable<串行接口（SPI或I 2 C）地址自动递增>
  I2Cdev::writeByte(address, CTRL7, 0x03); //Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer>

  //I2Cdev::writeByte(address, CTRL2, 0x04); //Accelerometer Settings<±2g  500Hz>
  I2Cdev::writeByte(address, CTRL2, 0x0c); //Accelerometer Settings<±2g  128Hz>
  //I2Cdev::writeByte(address, CTRL3, 0x64); //Gyroscope Settings< ±2048dps 500Hz>
  I2Cdev::writeByte(address, CTRL3, 0x36); //Gyroscope Settings< ±128dps 117.5Hz>
  I2Cdev::writeByte(address, CTRL5, 0x11); //Sensor Data Processing Settings<Enable Gyroscope Accelerometer 低通滤波>

  delay(2000);
  //test
  //I2Cdev::writeByte(address, CTRL7, 0x01); //Power-down Mode

  unsigned short times = 1000; //采样次数
  for (int i = 0; i < times; i++)
  {
    gz = QMI8658C_readBytes(GyrX_L);
    gzo += gz;
  }
  gzo /= times; //计算陀螺仪偏移
  return false;
}

bool QMI8658C::QMI8658C_dveGetEulerAngles(float *gyro, float *yaw)
{
  unsigned long now = millis();   //当前时间(ms)
  dt = (now - lastTime) / 1000.0; //微分时间(s)
  lastTime = now;                 //上一次采样时间(ms)
  gz = QMI8658C_readBytes(GyrX_L);      //读取六轴原始数值
  float gyroz = -(gz - gzo) / 32.00 * dt;
  if (fabs(gyroz) <= 0.05)
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z轴角速度积分

  *gyro = gyroz;

  *yaw = agz;
  return false;
}
bool QMI8658C::QMI8658C_dveGetEulerAngles(float *Yaw)
{
  unsigned long now = millis();   //当前时间(ms)
  dt = (now - lastTime) / 1000.0; //微分时间(s)
  lastTime = now;                 //上一次采样时间(ms)
  gz = QMI8658C_readBytes(GyrX_L);
  float gyroz = -(gz - gzo) / 32.00 * dt; //z轴角速度< 131.0 为传感器比例系数常量，详细信息请查阅MPU-6050_DataSheet>
  if (fabs(gyroz) <= 0.05)
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z轴角速度积分
  *Yaw = agz;
  return false;
}
void HDSC_IIC_Test(void)
{
  float gyro, is_yaw;
  uint8_t IIC_buff[10];
  uint8_t a = 0;
  Wire.requestFrom(0x51, 6); // request 6 bytes from slave device #2
  while (Wire.available())   // slave may send less than requested
  {
    IIC_buff[a++] = Wire.read(); // receive a byte as character
  }

  if ((IIC_buff[0] == 0XA1) && (IIC_buff[5] == 0XB1))
  {

    gyro = ((IIC_buff[1] << 8) | (IIC_buff[2])) / 1000.00;
    is_yaw = ((IIC_buff[3] << 8) | (IIC_buff[4])) / 100.00;

    Serial.print(gyro);
    Serial.print("\t");
    Serial.print(is_yaw);
  }
  else
  {
    /* code */
    Serial.println("Contact Changhua :STM8S003F3_MPU6050 data error"); // print the character
    return;
  }
  // for (int i = 0; i < a; i++)
  // {
  //   Serial.print(IIC_buff[i], HEX);
  //   Serial.print("\t");
  // }
  Serial.println("");
}
void QMI8658C::QMI8658C_Check(void)
{
  Wire.begin();
  uint8_t address_Test = 0;
  int error = 1;
  delay(1000);
  // do
  // {
  //   Wire.beginTransmission(address_Test);
  //   error = Wire.endTransmission();

  //   Serial.print("address_Test: 0X");
  //   Serial.println(address_Test, HEX);
  //   address_Test++;
  //   delay(100);
  // } while (error); //扫描从机设备
  // Serial.println("address_Test: OK");

  Wire.beginTransmission(0x51);
  Wire.write(110);
  Wire.endTransmission();

  for (;;)
  {

    HDSC_IIC_Test();
    //delay(1);
  }
}

// 获取三轴姿态（核心函数）
bool QMI8658C::getAttitude(float* pitch, float* roll, float* yaw) {
    // 1. 读取6轴原始数据
    ax = QMI8658C_readBytes(AccX_L);
    ay = QMI8658C_readBytes(AccY_L);
    az = QMI8658C_readBytes(AccZ_L);
    gx = QMI8658C_readBytes(GyrX_L);
    gy = QMI8658C_readBytes(GyrY_L);
    gz = QMI8658C_readBytes(GyrZ_L);

    Serial.print("Raw: ax ay az gx gy gz: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); Serial.print(", ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);

    // 2. 转换为物理单位 (防止数据溢出)
    const float acc_scale = 16384.0f; // ±2g量程的灵敏度
    const float gyr_scale = 256.0f;   // ±128dps量程的灵敏度
    
    float ax_g = ax / acc_scale;
    float ay_g = ay / acc_scale;
    float az_g = az / acc_scale;

    // 3. 计算时间差
    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    // 4. 加速度计计算姿态（带溢出保护）
    float denom = ay_g * ay_g + az_g * az_g;
    
    // 防护1：确保分母为正
    if (denom <= 0) {
        *pitch = this->pitch; // 使用上一次有效值
    } else {
        *pitch = atan2(-ax_g, sqrt(denom)) * RAD_TO_DEG;
    }
    
    // 防护2：roll计算同样需要保护
    if (fabs(az_g) < 0.001f) {
        *roll = this->roll;
    } else {
        *roll = atan2(ay_g, az_g) * RAD_TO_DEG;
    }

    // 5. 陀螺仪计算yaw
    float gyroz = -(gz - gzo) / gyr_scale * dt;
    if (fabs(gyroz) <= 0.05f) gyroz = 0;
    agz += gyroz;
    *yaw = agz;

    // 6. 更新类成员
    this->pitch = *pitch;
    this->roll = *roll;
    this->yaw = *yaw;

    return true;
}