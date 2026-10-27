#pragma once

#include <cstdint>

namespace Bmi088Reg_N
{
inline constexpr uint8_t acc_chip_id = 0x00;
inline constexpr uint8_t acc_chip_id_value = 0x1E;

inline constexpr uint8_t acc_err_reg = 0x02;
inline constexpr uint8_t accel_config_error = (1 << 2);
inline constexpr uint8_t accel_fatal_error = (1 << 0);

inline constexpr uint8_t acc_status = 0x03;
inline constexpr uint8_t accel_drdy = (1 << 7);

inline constexpr uint8_t accel_xout_l = 0x12;
inline constexpr uint8_t accel_yout_l = 0x14;
inline constexpr uint8_t accel_zout_l = 0x16;

inline constexpr uint8_t temp_m = 0x22;
inline constexpr uint8_t temp_l = 0x23;

inline constexpr uint8_t acc_conf = 0x40;
inline constexpr uint8_t acc_conf_must_set = 0x80;
inline constexpr uint8_t acc_osr4 = (0x0 << 4);
inline constexpr uint8_t acc_osr2 = (0x1 << 4);
inline constexpr uint8_t acc_normal = (0x2 << 4);
inline constexpr uint8_t acc_12_5_hz = (0x5 << 0);
inline constexpr uint8_t acc_25_hz = (0x6 << 0);
inline constexpr uint8_t acc_50_hz = (0x7 << 0);
inline constexpr uint8_t acc_100_hz = (0x8 << 0);
inline constexpr uint8_t acc_200_hz = (0x9 << 0);
inline constexpr uint8_t acc_400_hz = (0xA << 0);
inline constexpr uint8_t acc_800_hz = (0xB << 0);
inline constexpr uint8_t acc_1600_hz = (0xC << 0);

inline constexpr uint8_t acc_range = 0x41;
inline constexpr uint8_t acc_range_3g = (0x0 << 0);
inline constexpr uint8_t acc_range_6g = (0x1 << 0);
inline constexpr uint8_t acc_range_12g = (0x2 << 0);
inline constexpr uint8_t acc_range_24g = (0x3 << 0);

inline constexpr uint8_t int1_io_ctrl = 0x53;
inline constexpr uint8_t acc_int1_io_disable = (0x0 << 3);
inline constexpr uint8_t acc_int1_io_enable = (0x1 << 3);
inline constexpr uint8_t acc_int1_gpio_pp = (0x0 << 2);
inline constexpr uint8_t acc_int1_gpio_low = (0x0 << 1);
inline constexpr uint8_t acc_int1_gpio_high = (0x1 << 1);

inline constexpr uint8_t int_map_data = 0x58;
inline constexpr uint8_t int_map_off = 0x00;
inline constexpr uint8_t acc_int2_drdy_interrupt = (0x1 << 6);
inline constexpr uint8_t acc_int1_drdy_interrupt = (0x1 << 2);

inline constexpr uint8_t acc_self_test = 0x6D;
inline constexpr uint8_t acc_self_test_off = 0x00;
inline constexpr uint8_t acc_self_test_positive_signal = 0x0D;
inline constexpr uint8_t acc_self_test_negative_signal = 0x09;

inline constexpr uint8_t acc_pwr_conf = 0x7C;
inline constexpr uint8_t acc_pwr_suspend_mode = 0x03;
inline constexpr uint8_t acc_pwr_active_mode = 0x00;

inline constexpr uint8_t acc_pwr_ctrl = 0x7D;
inline constexpr uint8_t acc_enable_acc_off = 0x00;
inline constexpr uint8_t acc_enable_acc_on = 0x04;

inline constexpr uint8_t acc_softreset = 0x7E;
inline constexpr uint8_t acc_softreset_value = 0xB6;

inline constexpr uint8_t gyro_chip_id = 0x00;
inline constexpr uint8_t gyro_chip_id_value = 0x0F;

inline constexpr uint8_t gyro_x_l = 0x02;
inline constexpr uint8_t gyro_y_l = 0x04;
inline constexpr uint8_t gyro_z_l = 0x06;

inline constexpr uint8_t gyro_int_stat_1 = 0x0A;
inline constexpr uint8_t gyro_dydr = (0x1 << 7);

inline constexpr uint8_t gyro_range = 0x0F;
inline constexpr uint8_t gyro_2000 = (0x0 << 0);
inline constexpr uint8_t gyro_1000 = (0x1 << 0);
inline constexpr uint8_t gyro_500 = (0x2 << 0);
inline constexpr uint8_t gyro_250 = (0x3 << 0);
inline constexpr uint8_t gyro_125 = (0x4 << 0);

inline constexpr uint8_t gyro_bandwidth = 0x10;
inline constexpr uint8_t gyro_bandwidth_must_set = 0x80;
inline constexpr uint8_t gyro_2000_532_hz = 0x00;
inline constexpr uint8_t gyro_2000_230_hz = 0x01;
inline constexpr uint8_t gyro_1000_116_hz = 0x02;
inline constexpr uint8_t gyro_400_47_hz = 0x03;
inline constexpr uint8_t gyro_200_23_hz = 0x04;
inline constexpr uint8_t gyro_100_12_hz = 0x05;
inline constexpr uint8_t gyro_200_64_hz = 0x06;
inline constexpr uint8_t gyro_100_32_hz = 0x07;

inline constexpr uint8_t gyro_lpm1 = 0x11;
inline constexpr uint8_t gyro_normal_mode = 0x00;
inline constexpr uint8_t gyro_suspend_mode = 0x80;
inline constexpr uint8_t gyro_deep_suspend_mode = 0x20;

inline constexpr uint8_t gyro_softreset = 0x14;
inline constexpr uint8_t gyro_softreset_value = 0xB6;

inline constexpr uint8_t gyro_ctrl = 0x15;
inline constexpr uint8_t drdy_off = 0x00;
inline constexpr uint8_t drdy_on = 0x80;

inline constexpr uint8_t gyro_int3_int4_io_conf = 0x16;
inline constexpr uint8_t gyro_int4_gpio_pp = (0x0 << 3);
inline constexpr uint8_t gyro_int4_gpio_low = (0x0 << 2);
inline constexpr uint8_t gyro_int3_gpio_pp = (0x0 << 1);
inline constexpr uint8_t gyro_int3_gpio_low = (0x0 << 0);

inline constexpr uint8_t gyro_int3_int4_io_map = 0x18;
inline constexpr uint8_t gyro_drdy_io_off = 0x00;
inline constexpr uint8_t gyro_drdy_io_int3 = 0x01;
inline constexpr uint8_t gyro_drdy_io_int4 = 0x80;
inline constexpr uint8_t gyro_drdy_io_both = 0x81;

inline constexpr uint8_t gyro_self_test = 0x3C;
}
