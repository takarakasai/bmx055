// Author : takara.kasai@gmail.com

#include<Wire.h>

#define DP_BMX055_DEBUG

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

void Send(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void InitAccel() {
  // [-2,+2] [g]
  Send(kAddrAccel, 0x0F, 0x03);

  // 7.81 [Hz]
  Send(kAddrAccel, 0x10, 0x08);

  // Normal Mode
  // sleep 0.5[msec]
  Send(kAddrAccel, 0x11, 0x00);
}

void InitGyro() {
  // [-125,+125] [deg/s]
  Send(kAddrGyro, 0x0F, 0x04);

  // 100 [Hz]
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
  Serial.begin(9600);

  Wire.begin();
  SetupDevice();
  delay(300);
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
  return val * 0.0098;
}

float GyroToFloat(uint8_t data[2]) {
  int val = ((data[1] * 256) + data[0]);
  if (val > 32767) {
    val -= 65536;
  }
  return val * 0.0038;
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

float accl[3] = {0.0, 0.0, 0.0};
float gyro[3] = {0.0, 0.0, 0.0};
float mag[3]  = {0.0, 0.0, 0.0};

void ConvertAccel(uint8_t data[6]) {
  for (int i = 0; i < 3; i++) {
    accl[i] = AccelToFloat(&data[i * 2]);
  }
}

void ConvertGyro(uint8_t data[6]) {
  for (int i = 0; i < 3; i++) {
    gyro[i] = GyroToFloat(&data[i * 2]);
  }
}

void ConvertMag(uint8_t data[6]) {
  mag[0] = MagXYToFloat(&data[0]);
  mag[1] = MagXYToFloat(&data[2]);
  mag[2] = MagZToFloat(&data[4]);
}

#if defined(DP_BMX055_DEBUG)
void PrintSensorValues(const char* header, float val[3]) {
  Serial.print(header);
  Serial.print(val[0]);
  Serial.print(", ");
  Serial.print(val[1]);
  Serial.print(", ");
  Serial.println(val[2]);
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

#if defined(DP_BMX055_DEBUG)
  PrintSensorValues("Accel : ", accl);
  PrintSensorValues("Gyro  : ", gyro);
  PrintSensorValues("Mag   : ", mag);
#endif

  delay(100);
}
