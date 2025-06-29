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
int16_t QMI8658C_readBytes(unsigned char tmp) {
  uint8_t buffer[14];
  I2Cdev::readBytes(address, tmp, 2, buffer);
  return ((buffer[1] << 8) | buffer[0]);
}
bool QMI8658C::QMI8658C_dveInit(void) {
  static unsigned long lastTime=0;
  Wire.begin();
  delay(1000);
  uint8_t chip_id = 0x00;
  do {
    I2Cdev::readBytes(address, WHO_AM_I, 1, &chip_id);
    Serial.print("WHO_AM_I: 0X");
    Serial.println(chip_id, HEX);
    delay(10);
  } while (chip_id == 0);  //确保从机设备在线（强行等待 获取 ID ）

  I2Cdev::writeByte(address, CTRL1, 0x40);  //Serial Interface and Sensor Enable<串行接口（SPI或I 2 C）地址自动递增>
  I2Cdev::writeByte(address, CTRL7, 0x03);  //Enable Sensors and Configure Data Reads<Enable Gyroscope Accelerometer>

  //I2Cdev::writeByte(address, CTRL2, 0x04); //Accelerometer Settings<±2g  500Hz>
  I2Cdev::writeByte(address, CTRL2, 0x0c);  //Accelerometer Settings<±2g  128Hz>
  //I2Cdev::writeByte(address, CTRL3, 0x64); //Gyroscope Settings< ±2048dps 500Hz>
  I2Cdev::writeByte(address, CTRL3, 0x36);  //Gyroscope Settings< ±128dps 117.5Hz>
  I2Cdev::writeByte(address, CTRL5, 0x11);  //Sensor Data Processing Settings<Enable Gyroscope Accelerometer 低通滤波>

  delay(2000);
  //test
  //I2Cdev::writeByte(address, CTRL7, 0x01); //Power-down Mode

  unsigned short times = 1000;  //采样次数
  for (int i = 0; i < times; i++) {
    gx = QMI8658C_readBytes(GyrX_L);
    gy = QMI8658C_readBytes(GyrY_L);
    gz = QMI8658C_readBytes(GyrZ_L);
    
    gxo += gx;
    gyo += gy;
    gzo += gz;
  }
  gxo /= times;
  gyo /= times;
  gzo /= times;  //计算陀螺仪偏移
  // After calibration
  Serial.print("Offsets X:");
  Serial.print(gxo);
  Serial.print(" Y:");
  Serial.print(gyo);
  Serial.print(" Z:");
  Serial.println(gzo);

  initFusion();
  return false;
}

bool QMI8658C::QMI8658C_dveGetEulerAngles(float *gyro, float *yaw) {
  unsigned long now = millis();     //当前时间(ms)
  dt = (now - lastTime) / 1000.0;   //微分时间(s)
  lastTime = now;                   //上一次采样时间(ms)
  gz = QMI8658C_readBytes(GyrX_L);  //读取六轴原始数值
  float gyroz = -(gz - gzo) / 32.00 * dt;
  if (fabs(gyroz) <= 0.05) {
    gyroz = 0.00;
  }
  agz += gyroz;  //z轴角速度积分

  *gyro = gyroz;

  *yaw = agz;
  return false;
}
bool QMI8658C::QMI8658C_dveGetEulerAngles(float *Yaw) {
  unsigned long now = millis();    //当前时间(ms)
  dt = (now - lastTime) / 1000.0;  //微分时间(s)
  lastTime = now;                  //上一次采样时间(ms)
  gz = QMI8658C_readBytes(GyrX_L);
  float gyroz = -(gz - gzo) / 32.00 * dt;  //z轴角速度< 131.0 为传感器比例系数常量，详细信息请查阅MPU-6050_DataSheet>
  if (fabs(gyroz) <= 0.05) {
    gyroz = 0.00;
  }
  agz += gyroz;  //z轴角速度积分
  *Yaw = agz;
  return false;
}
void HDSC_IIC_Test(void) {
  float gyro, is_yaw;
  uint8_t IIC_buff[10];
  uint8_t a = 0;
  Wire.requestFrom(0x51, 6);  // request 6 bytes from slave device #2
  while (Wire.available())    // slave may send less than requested
  {
    IIC_buff[a++] = Wire.read();  // receive a byte as character
  }

  if ((IIC_buff[0] == 0XA1) && (IIC_buff[5] == 0XB1)) {

    gyro = ((IIC_buff[1] << 8) | (IIC_buff[2])) / 1000.00;
    is_yaw = ((IIC_buff[3] << 8) | (IIC_buff[4])) / 100.00;

    Serial.print(gyro);
    Serial.print("\t");
    Serial.print(is_yaw);
  } else {
    /* code */
    Serial.println("Contact Changhua :STM8S003F3_MPU6050 data error");  // print the character
    return;
  }
  // for (int i = 0; i < a; i++)
  // {
  //   Serial.print(IIC_buff[i], HEX);
  //   Serial.print("\t");
  // }
  Serial.println("");
}
void QMI8658C::QMI8658C_Check(void) {
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

  for (;;) {

    HDSC_IIC_Test();
    //delay(1);
  }
}

// Initialize quaternion and filter parameters
void QMI8658C::initFusion() {
  q0 = 1.0f;
  q1 = q2 = q3 = 0.0f;
  // Increase beta for higher accelerometer trust
  beta = 0.2f;  // Was 0.1f, better for ±128dps
  // Add zeta for drift compensation
  zeta = 0.02f;  // Gyro drift compensation
  integralFBx = integralFBy = integralFBz = 0.0f;
}

// Sensor fusion update function
void QMI8658C::updateFusedData(float *pitch, float *roll, float *yaw) {
  // 添加时间基准检查
  static unsigned long lastCallTime = 0;  // 静态变量保持状态
  unsigned long now = millis();
  
  // 计算实际时间差（毫秒）
  unsigned long elapsed = now - lastCallTime;
  
  // 处理首次调用和超高频调用
  if (elapsed == 0) {
    dt = 0.001f;  // 最小时间步长 (1ms)
  } else {
    dt = elapsed / 1000.0f;  // 转换为秒
  }
  
  lastCallTime = now;  // 更新调用时间
  
  // 保持原lastTime用于其他函数
  // Read raw sensor data
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

  // Serial.print("dt: ");
  // Serial.println(dt);

  // Convert to proper units
  float ax_g = ax * (2.0f / 32768.0f);
  float ay_g = ay * (2.0f / 32768.0f);
  float az_g = az * (2.0f / 32768.0f);
  float gx_rad = (gx - gxo) * (128.0f / 32768.0f) * (M_PI / 180.0f);
  float gy_rad = (gy - gyo) * (128.0f / 32768.0f) * (M_PI / 180.0f);
  float gz_rad = (gz - gzo) * (128.0f / 32768.0f) * (M_PI / 180.0f);
  // float gx_rad = (gx) * (128.0f / 32768.0f) * (M_PI / 180.0f);
  // float gy_rad = (gy) * (128.0f / 32768.0f) * (M_PI / 180.0f);
  // float gz_rad = (gz) * (128.0f / 32768.0f) * (M_PI / 180.0f);

  // Normalize accelerometer measurement
  float norm_a = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
  if (norm_a < 1e-7f) { // 检查加速度计模长
      return; // 跳过此次更新
  }
  float recipNorm = 1.0f / sqrt(norm_a);
  //float recipNorm = 1.0f / sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  ax_g *= recipNorm;
  ay_g *= recipNorm;
  az_g *= recipNorm;

  // Estimated gravity direction
  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // Error is cross product between estimated and measured gravity
  float ex = (ay_g * vz - az_g * vy);
  float ey = (az_g * vx - ax_g * vz);
  float ez = (ax_g * vy - ay_g * vx);

  // Apply feedback
  integralFBx += ex * beta * dt;
  integralFBy += ey * beta * dt;
  integralFBz += ez * beta * dt;

  gx_rad += beta * ex + integralFBx;
  gy_rad += beta * ey + integralFBy;
  gz_rad += beta * ez + integralFBz;

  // Integrate quaternion
  gx_rad *= (0.5f * dt);
  gy_rad *= (0.5f * dt);
  gz_rad *= (0.5f * dt);

  float qa = q0;
  float qb = q1;
  float qc = q2;
  q0 += (-qb * gx_rad - qc * gy_rad - q3 * gz_rad);
  q1 += (qa * gx_rad + qc * gz_rad - q3 * gy_rad);
  q2 += (qa * gy_rad - qb * gz_rad + q3 * gx_rad);
  q3 += (qa * gz_rad + qb * gy_rad - qc * gx_rad);

  // Normalize quaternion
  // 在四元数更新后添加：
  float norm_q = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  if (norm_q < 1e-7f) {
      initFusion(); // 重新初始化四元数
      return;
  }
  recipNorm = 1.0f / sqrt(norm_q);
  //recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Convert to Euler angles
  // *roll = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
  // *pitch = asin(-2.0f * (q1 * q3 - q0 * q2));
  // *yaw = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

  *roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 180.0f / M_PI;
  
  float pitch_val = -2.0f * (q1*q3 - q0*q2);
  pitch_val = fmax(-1.0f, fmin(1.0f, pitch_val));
  *pitch = asin(pitch_val) * 180.0f / M_PI;  // 转为角度
  
  *yaw = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 180.0f / M_PI;

  // 低通滤波 (0.2为滤波系数，值越小越平滑)
  const float filterFactor = 0.2f;
  
  filtered_roll = filtered_roll*(1-filterFactor) + *roll*filterFactor;
  filtered_pitch = filtered_pitch*(1-filterFactor) + *pitch*filterFactor;
  filtered_yaw = filtered_yaw*(1-filterFactor) + *yaw*filterFactor;
  
  *roll = filtered_roll;
  *pitch = filtered_pitch;
  *yaw = filtered_yaw;
}