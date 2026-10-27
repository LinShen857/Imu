#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: BMI088 六轴 IMU 与 VQF 姿态解算
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "bmi088.hpp"
#include "gpio.hpp"
#include "libxr.hpp"
#include "spi.hpp"
#include "vqf.hpp"

namespace Imu_N
{
inline constexpr uint32_t sample_freq_hz = 800;
inline constexpr float sample_dt = 0.00125f;
inline constexpr uint32_t drdy_timeout_ms = 5;
inline constexpr uint32_t dt_min_us = 400;
inline constexpr uint32_t dt_max_us = 5000;
inline constexpr float gyro_avg_tau = 5.0f;
inline constexpr float pitch_sign = -1.0f;
}  // namespace Imu_N

class Imu
{
 public:
  Imu(LibXR::SPI& spi, LibXR::GPIO& acc_cs, LibXR::GPIO& gyro_cs,
      LibXR::GPIO* acc_int);

  Bmi088 bmi088;
  Vqf vqf;
  int32_t init_ans = 0;
  uint8_t acc_id = 0;
  uint8_t gyro_id = 0;

  static Imu* Instance();

 private:
  static Imu* instance_;
};
