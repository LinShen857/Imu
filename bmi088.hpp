#pragma once

#include <cstdint>

#include "gpio.hpp"
#include "libxr_def.hpp"
#include "semaphore.hpp"
#include "spi.hpp"

namespace Bmi088_N
{
inline constexpr float temp_factor = 0.125f;
inline constexpr float temp_offset = 23.0f;

inline constexpr float accel_sen_3g = 0.0008974358974f;
inline constexpr float accel_sen_6g = 0.00179443359375f;
inline constexpr float accel_sen_12g = 0.0035888671875f;
inline constexpr float accel_sen_24g = 0.007177734375f;

inline constexpr float gyro_sen_2000 = 0.00106526443603169529841533860381f;
inline constexpr float gyro_sen_1000 = 0.00053263221801584764920766930190693f;
inline constexpr float gyro_sen_500 = 0.00026631610900792382460383465095346f;
inline constexpr float gyro_sen_250 = 0.00013315805450396191230191732547673f;
inline constexpr float gyro_sen_125 = 0.000066579027251980956150958662738366f;

inline constexpr uint16_t long_delay_time = 80;
inline constexpr uint16_t com_wait_sensor_time = 150;
inline constexpr uint16_t default_calibrate_num = 2000;
inline constexpr uint32_t spi_timeout_ms = 100;
}

struct Bmi088Data
{
	float accel[3] {};
	float gyro[3] {};
	float temperature = 0.f;
	float gyro_offset[3] {};
	float accel_scale = 1.f;
	float g_norm = 0.f;
};

class Bmi088
{
	public:
	Bmi088(LibXR::SPI& spi_, LibXR::GPIO& accel_cs_, LibXR::GPIO& gyro_cs_,
		LibXR::GPIO* accel_int_ = nullptr);

	LibXR::ErrorCode Init();
	void Read();
	LibXR::ErrorCode CalibrateOffset(uint16_t sample_num_ = Bmi088_N::default_calibrate_num);
	LibXR::ErrorCode WaitDataReady(uint32_t timeout_ms_);

	Bmi088Data data {};
	uint8_t acc_id_read = 0;
	uint8_t gyro_id_read = 0;

	private:
	static void DataReadyIsr(bool in_isr_, Bmi088* self_);
	LibXR::ErrorCode ReadRegs(LibXR::GPIO& cs_, uint8_t reg_, uint8_t* buf_, uint8_t len_,
		uint8_t dummy_);
	LibXR::ErrorCode WriteReg(LibXR::GPIO& cs_, uint8_t reg_, uint8_t val_);
	void ReadAccelRaw(int16_t out_[3]);
	bool ReadGyroRaw(int16_t out_[3]);
	float ReadTemperature();
	LibXR::ErrorCode AccelInit();
	LibXR::ErrorCode GyroInit();

	LibXR::SPI& spi;
	LibXR::GPIO& accel_cs;
	LibXR::GPIO& gyro_cs;
	LibXR::GPIO* accel_int;
	LibXR::Semaphore spi_sem;
	LibXR::Semaphore drdy_sem;
	uint32_t spi_timeout = Bmi088_N::spi_timeout_ms;
	float accel_sen = Bmi088_N::accel_sen_6g;
	float gyro_sen = Bmi088_N::gyro_sen_2000;
};
