// Author : takara.kasai@gmail.com

#include<Wire.h>
#include<ros.h>
#include<ros/time.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/MagneticField.h>

// #define DP_BMX055_DEBUG

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
#if !defined(DP_BMX055_DEBUG)
// ros::NodeHandle nh;
ros::NodeHandle_<ArduinoHardware, 0, 2, 0, 480, ros::FlashReadOutBuffer_> nh;
ros::Publisher pub_imu("/imu/data_raw", &imu_msg);
ros::Publisher pub_mag("/imu/mag", &mag_msg);
constexpr char frameid[] = "/imu";
#endif

//  Jamper |  Sensor Addr
// =========|===============
//   1 2 3  | Accl Gyro Mag
// ---------|---------------
//   O O O  | 0x19 0x69 0x13
//   S O O  | 0x19 0x69 0x11
//   O S O  | 0x18 0x69 0x12
//   S S O  | 0x18 0x69 0x10
// * O O S  | 0x19 0x68 0x13
//   S O S  | 0x19 0x68 0x11
//   O S S  | 0x18 0x68 0x12
//   S S S  | 0x18 0x68 0x10
constexpr uint8_t kAddrAccel = 0x19;
constexpr uint8_t kAddrGyro = 0x68;
constexpr uint8_t kAddrMag = 0x13;
constexpr float kPI = 3.1415;
constexpr float kDeg2Rad = kPI / 180.0;

void Send(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delay(100);
}

void InitAccel() {
  // [-2,+2] [g]
  Send(kAddrAccel, 0x0F, 0x03);

  // 01000    7.81 64.0[ms] (default?)
  // 01001   15.63 32.0[ms]
  // 01010   31.25 16.0[ms]
  // 01011   62.50  8.0[ms]
  // 01100  125.00  4.0[ms]
  // 01101  250.00  2.0[ms]
  // 01110  500.00  1.0[ms]
  // 01111 1000.00  0.5[ms]
  Send(kAddrAccel, 0x10, 0x08);

  // Normal Mode
  // sleep 0.5[msec]
  Send(kAddrAccel, 0x11, 0x00);
}

void InitGyro() {
  // [-125,+125] [deg/s]
  Send(kAddrGyro, 0x0F, 0x04);

  // 100 [Hz]
  // 0b0111 : 100
  // 0b0110 : 200
  // ...
  // 0b0000 : 2000
  Send(kAddrGyro, 0x10, 0x07);

  // Normal Mode
  // sleep 2.0[msec]
  Send(kAddrGyro, 0x11, 0x00);
}

void InitMag() {
  // soft reset
  Send(kAddrGyro, 0x4B, 0x83);
  Send(kAddrGyro, 0x4B, 0x01);

  // 10 [Hz]
  // 0b0000 : 10 (default)
  // 0b0001 :  2
  // 0b0010 :  6
  // 0b0011 :  8
  // 0b0100 : 15
  // 0b0101 : 20
  // 0b0110 : 25
  // 0b0111 : 30
  Send(kAddrGyro, 0x4C, 0x00);

  // enable all axis (x,y,z)
  Send(kAddrGyro, 0x4E, 0x84);

  // X-Y Repetition :  9
  //   Z Repetition : 15
  Send(kAddrGyro, 0x51, 0x04);
  Send(kAddrGyro, 0x52, 0x16);
}

void SetupDevice() {
  InitAccel();
  InitGyro();
  InitMag();
}

void setup()
{
#if defined(DP_BMX055_DEBUG)
  Serial.begin(9600);
#endif

  Wire.begin();
  SetupDevice();  
  delay(300);


#if !defined(DP_BMX055_DEBUG)
  //          official try
  // 115200 :    OK     OK  1?Hz
  // 230400 :    ??     OK  35Hz
  // 460800 :    ??     OK  56Hz
  // 921600 :    ??     NG  --Hz
  nh.getHardware()->setBaud(230400);
  nh.initNode();
  nh.advertise(pub_imu);
  nh.advertise(pub_mag);
  imu_msg.header.frame_id = frameid;
  imu_msg.header.stamp = nh.now();
  mag_msg.header.frame_id = frameid;
  mag_msg.header.stamp = nh.now();

  imu_msg.orientation_covariance[0] = -1;
  imu_msg.angular_velocity_covariance[0] = -1;
  imu_msg.linear_acceleration_covariance[0] = -1;
#endif
}

void Read(int addr, uint8_t start_reg, uint8_t data[6]) {
  // Read 6[Byte]
  // |   X   |   Y   |   Z   |
  // |=======|=======|=======|
  // | 1 | 2 | 3 | 4 | 5 | 6 |
  // |LSB|MSB|LSB|MSB|LSB|MSB|

  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(addr);
    Wire.write((start_reg + i));
    Wire.endTransmission();
    Wire.requestFrom(addr, 1);

    if (Wire.available() == 1) {
      data[i] = Wire.read();
    }
  }
}

float AccelToFloat(uint8_t data[2]) {
  int val = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (val > 2047) {
    val -= 4096;
  }
  // [-2G, +2G] [mm/s^2] --> [m/s^2]
  return val * 0.001; 
}

float GyroToFloat(uint8_t data[2]) {
  int val = ((data[1] * 256) + data[0]);
  if (val > 32767) {
    val -= 65536;
  }
  // [-125, +125] [degree/s] --> [rad/s]
  return (val * 0.0038) * kDeg2Rad;
}

float MagXYToFloat(uint8_t data[2]) {
  int value = ((data[1] * 256) + (data[0] & 0xF8)) / 8;
  if (value > 4095) {
    value -= 8192;
  }
  return value;
}

float MagZToFloat(uint8_t data[2]) {
  int value = ((data[1] * 256) + (data[0] & 0xFE)) / 2;
  if (value > 16383) {
    value -= 32768;
  }
  return value;
}

void ConvertAccel(uint8_t data[6]) {
  imu_msg.linear_acceleration.x = AccelToFloat(&data[0]);
  imu_msg.linear_acceleration.y = AccelToFloat(&data[2]);
  imu_msg.linear_acceleration.z = AccelToFloat(&data[4]);
}

void ConvertGyro(uint8_t data[6]) {
  imu_msg.angular_velocity.x = GyroToFloat(&data[0]);
  imu_msg.angular_velocity.y = GyroToFloat(&data[2]);
  imu_msg.angular_velocity.z = GyroToFloat(&data[4]);
}

void ConvertMag(uint8_t data[6]) {
  mag_msg.magnetic_field.x = MagXYToFloat(&data[0]);
  mag_msg.magnetic_field.y = MagXYToFloat(&data[2]);
  mag_msg.magnetic_field.z = MagZToFloat(&data[4]);
}

#if defined(DP_BMX055_DEBUG)
void PrintSensorValues(const char* header, const geometry_msgs::Vector3& val) {
  Serial.print(header);
  Serial.print(val.x);
  Serial.print(", ");
  Serial.print(val.y);
  Serial.print(", ");
  Serial.println(val.z);
}
#endif

void loop()
{
  uint8_t data[6];

  Read(kAddrAccel, 2, data);
  ConvertAccel(data);
  Read(kAddrGyro, 2, data);
  ConvertGyro(data);
  
  Read(kAddrMag, 66, data);
  ConvertMag(data);

#if !defined(DP_BMX055_DEBUG)
  pub_imu.publish(&imu_msg);
  pub_mag.publish(&mag_msg);
  nh.spinOnce();
#endif

#if defined(DP_BMX055_DEBUG)
  PrintSensorValues("Accel : ", imu_msg.linear_acceleration);
  PrintSensorValues("Gyro  : ", imu_msg.angular_velocity);
  PrintSensorValues("Mag   : ", mag_msg.magnetic_field);
#endif
}
