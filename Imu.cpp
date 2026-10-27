#include "Imu.hpp"

Imu* Imu::instance_ = nullptr;

Imu::Imu(LibXR::SPI& spi, LibXR::GPIO& acc_cs, LibXR::GPIO& gyro_cs, LibXR::GPIO* acc_int)
    : bmi088(spi, acc_cs, gyro_cs, acc_int), vqf(Imu_N::sample_dt)
{
  init_ans = static_cast<int32_t>(bmi088.Init());
  acc_id = bmi088.acc_id_read;
  gyro_id = bmi088.gyro_id_read;
  (void)bmi088.CalibrateOffset();
  instance_ = this;
}

Imu* Imu::Instance()
{
  return instance_;
}
