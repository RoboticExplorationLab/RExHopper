#include "hopper_mpc/wt901.h"
#include "hopper_mpc/utils.hpp"
// for sleep_for
#include <chrono>
#include <thread>

Wt901::Wt901() {
  mraa::init();
  if (geteuid()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  uint8_t ucDevAddr = 0x50;  // from the manual
  // how we know which bus to use:
  // https://github.com/up-board/up-community/wiki/Pinout_Xtreme
  // i2c_designware.3 -> I2C channel on hat (pin 3,5 on HAT)
  // ls /sys/bus/pci/devices/0000\:00\:19.0/i2c_designware*/ | grep i2c
  i2cPtr.reset(new mraa::I2c(0));
  i2cPtr->address(ucDevAddr);

  double imu_mount_angle = 8.9592122 * M_PI / 180;
  R1 = Utils::EulerToQuat(imu_mount_angle, 0.0, 0.0).matrix();
  R2 = Utils::EulerToQuat(0, 0, 0.5 * M_PI).matrix();
  T << -35.3, 0.0, 14.8;
}

void Wt901::ReadRegisters(unsigned char addressToRead, unsigned char bytesToRead, uint8_t* dest) {
  i2cPtr->readBytesReg(addressToRead, dest, bytesToRead);
}

void Wt901::GetTime() {
  ReadRegisters(0x30, 8, (uint8_t*)&stcTime);
}
void Wt901::GetAcc() {
  ReadRegisters(AX, 6, (uint8_t*)&stcAcc);
}
void Wt901::GetGyro() {
  ReadRegisters(GX, 6, (uint8_t*)&stcGyro);
}
void Wt901::GetAngle() {
  ReadRegisters(Roll, 6, (uint8_t*)&stcAngle);
}
void Wt901::GetMag() {
  ReadRegisters(HX, 6, (uint8_t*)&stcMag);
}
void Wt901::GetPress() {
  ReadRegisters(PressureL, 8, (uint8_t*)&stcPress);
}
void Wt901::GetDStatus() {
  ReadRegisters(D0Status, 8, (uint8_t*)&stcDStatus);
}
void Wt901::GetLonLat() {
  ReadRegisters(LonL, 8, (uint8_t*)&stcLonLat);
}
void Wt901::GetGPSV() {
  ReadRegisters(GPSHeight, 8, (uint8_t*)&stcGPSV);
}

Eigen::Vector3d Wt901::CollectAcc() {
  // collecting data
  GetAcc();
  GetGyro();
  // GetTime();

  // int time_ms =
  //     static_cast<int>(stcTime.ucMinute) * 60 * 1000 + static_cast<int>(stcTime.ucSecond) * 1000 +
  //     static_cast<int>(stcTime.usMilliSecond);
  // float time_s = time_ms / 1000;

  float acc_x = (float)stcAcc.a[0] / 32768 * 16 * 9.8;
  float acc_y = (float)stcAcc.a[1] / 32768 * 16 * 9.8;
  float acc_z = (float)stcAcc.a[2] / 32768 * 16 * 9.8;
  alpha << acc_x, acc_y, acc_z;  // acc in imu frame

  float omega_x = (float)stcGyro.w[0] / 32768 * 2000;
  float omega_y = (float)stcGyro.w[1] / 32768 * 2000;
  float omega_z = (float)stcGyro.w[2] / 32768 * 2000;
  omega << omega_x, omega_y, omega_z;  // angular vel in imu frame

  omega_f = R2 * (R1 * omega);                                                // omega at foot in link 3 frame
  alpha_f = R2 * (R1 * alpha) - Utils::Skew(omega) * Utils::Skew(omega) * T;  // acc at foot in link 3 frame

  return alpha_f;
}