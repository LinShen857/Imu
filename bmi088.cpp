#include "bmi088.hpp"

#include <cmath>

#include "bmi088_reg.hpp"
#include "libxr.hpp"

using namespace Bmi088Reg_N;

Bmi088::Bmi088(LibXR::SPI& spi_, LibXR::GPIO& accel_cs_, LibXR::GPIO& gyro_cs_,
	LibXR::GPIO* accel_int_)
	: spi(spi_), accel_cs(accel_cs_), gyro_cs(gyro_cs_), accel_int(accel_int_)
{
	accel_cs.Write(true);
	gyro_cs.Write(true);
	if (accel_int != nullptr)
	{
		accel_int->RegisterCallback(LibXR::GPIO::Callback::Create(DataReadyIsr, this));
	}
}

void Bmi088::DataReadyIsr(bool in_isr, Bmi088* self)
{
	self->drdy_sem.PostFromCallback(in_isr);
}

LibXR::ErrorCode Bmi088::WaitDataReady(uint32_t timeout_ms)
{
	return drdy_sem.Wait(timeout_ms);
}

LibXR::ErrorCode Bmi088::ReadRegs(LibXR::GPIO& cs, uint8_t reg, uint8_t* buf, uint8_t len,
	uint8_t dummy)
{
	uint8_t rx[12] {};
	const uint8_t addr = static_cast<uint8_t>(reg | 0x80);
	const uint8_t tx[2] = {addr, addr};
	const size_t tx_len = (dummy > 0) ? 2u : 1u;
	const size_t read_len = static_cast<size_t>(len) + tx_len;

	LibXR::SPI::OperationRW op(spi_sem, spi_timeout);
	cs.Write(false);
	const LibXR::ErrorCode ans =
		spi.ReadAndWrite(LibXR::RawData(rx, read_len), LibXR::ConstRawData(tx, tx_len), op);
	cs.Write(true);
	if (ans != LibXR::ErrorCode::OK)
	{
		return ans;
	}

	for (uint8_t i = 0; i < len; ++i)
	{
		buf[i] = rx[i + tx_len];
	}
	return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode Bmi088::WriteReg(LibXR::GPIO& cs, uint8_t reg, uint8_t val)
{
	const uint8_t tx[2] = {reg, val};

	LibXR::SPI::OperationRW op(spi_sem, spi_timeout);
	cs.Write(false);
	const LibXR::ErrorCode ans = spi.Write(LibXR::ConstRawData(tx, 2u), op);
	cs.Write(true);
	return ans;
}

void Bmi088::ReadAccelRaw(int16_t out[3])
{
	uint8_t buf[6] {};
	ReadRegs(accel_cs, accel_xout_l, buf, 6, 1);
	out[0] = static_cast<int16_t>((static_cast<uint16_t>(buf[1]) << 8) | buf[0]);
	out[1] = static_cast<int16_t>((static_cast<uint16_t>(buf[3]) << 8) | buf[2]);
	out[2] = static_cast<int16_t>((static_cast<uint16_t>(buf[5]) << 8) | buf[4]);
}

bool Bmi088::ReadGyroRaw(int16_t out[3])
{
	uint8_t buf[8] {};
	ReadRegs(gyro_cs, gyro_chip_id, buf, 8, 0);
	gyro_id_read = buf[0];
	if (buf[0] != gyro_chip_id_value)
	{
		return false;
	}
	out[0] = static_cast<int16_t>((static_cast<uint16_t>(buf[3]) << 8) | buf[2]);
	out[1] = static_cast<int16_t>((static_cast<uint16_t>(buf[5]) << 8) | buf[4]);
	out[2] = static_cast<int16_t>((static_cast<uint16_t>(buf[7]) << 8) | buf[6]);
	return true;
}

float Bmi088::ReadTemperature()
{
	uint8_t buf[2] {};
	ReadRegs(accel_cs, temp_m, buf, 2, 1);
	int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 3) | (buf[1] >> 5));
	if (raw > 1023)
	{
		raw -= 2048;
	}
	return static_cast<float>(raw) * Bmi088_N::temp_factor + Bmi088_N::temp_offset;
}

LibXR::ErrorCode Bmi088::AccelInit()
{
	uint8_t res = 0;
	ReadRegs(accel_cs, acc_chip_id, &res, 1, 1);
	LibXR::Thread::Sleep(1);
	WriteReg(accel_cs, acc_softreset, acc_softreset_value);
	LibXR::Thread::Sleep(Bmi088_N::long_delay_time);

	for (uint8_t i = 0; i < 3; ++i)
	{
		res = 0;
		ReadRegs(accel_cs, acc_chip_id, &res, 1, 1);
		acc_id_read = res;
		if (res == acc_chip_id_value)
		{
			break;
		}
		LibXR::Thread::Sleep(Bmi088_N::com_wait_sensor_time);
	}

	uint8_t cfg[6][2] = {
		{acc_pwr_ctrl, acc_enable_acc_on},
		{acc_pwr_conf, acc_pwr_active_mode},
		{acc_conf, static_cast<uint8_t>(acc_normal | acc_800_hz | acc_conf_must_set)},
		{acc_range, acc_range_6g},
		{int1_io_ctrl, acc_int1_io_disable},
		{int_map_data, int_map_off},
	};

	if (accel_int != nullptr)
	{
		cfg[4][1] = static_cast<uint8_t>(acc_int1_io_enable | acc_int1_gpio_pp | acc_int1_gpio_high);
		cfg[5][1] = acc_int1_drdy_interrupt;
	}

	LibXR::ErrorCode ans =
		(acc_id_read == acc_chip_id_value) ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
	for (uint8_t i = 0; i < 6; ++i)
	{
		WriteReg(accel_cs, cfg[i][0], cfg[i][1]);
		LibXR::Thread::Sleep(1);
		res = 0;
		ReadRegs(accel_cs, cfg[i][0], &res, 1, 1);
		if (res != cfg[i][1])
		{
			ans = LibXR::ErrorCode::FAILED;
		}
	}
	return ans;
}

LibXR::ErrorCode Bmi088::GyroInit()
{
	uint8_t res = 0;
	ReadRegs(gyro_cs, gyro_chip_id, &res, 1, 0);
	LibXR::Thread::Sleep(1);
	WriteReg(gyro_cs, gyro_softreset, gyro_softreset_value);
	LibXR::Thread::Sleep(Bmi088_N::long_delay_time);

	for (uint8_t i = 0; i < 3; ++i)
	{
		res = 0;
		ReadRegs(gyro_cs, gyro_chip_id, &res, 1, 0);
		gyro_id_read = res;
		if (res == gyro_chip_id_value)
		{
			break;
		}
		LibXR::Thread::Sleep(Bmi088_N::com_wait_sensor_time);
	}

	static const uint8_t cfg[6][2] = {
		{gyro_range, gyro_2000},
		{gyro_bandwidth, static_cast<uint8_t>(gyro_1000_116_hz | gyro_bandwidth_must_set)},
		{gyro_lpm1, gyro_normal_mode},
		{gyro_ctrl, drdy_on},
		{gyro_int3_int4_io_conf, static_cast<uint8_t>(gyro_int3_gpio_pp | gyro_int3_gpio_low)},
		{gyro_int3_int4_io_map, gyro_drdy_io_off},
	};

	LibXR::ErrorCode ans =
		(gyro_id_read == gyro_chip_id_value) ? LibXR::ErrorCode::OK : LibXR::ErrorCode::FAILED;
	for (uint8_t i = 0; i < 6; ++i)
	{
		WriteReg(gyro_cs, cfg[i][0], cfg[i][1]);
		LibXR::Thread::Sleep(1);
		res = 0;
		ReadRegs(gyro_cs, cfg[i][0], &res, 1, 0);
		if (res != cfg[i][1])
		{
			ans = LibXR::ErrorCode::FAILED;
		}
	}
	return ans;
}

LibXR::ErrorCode Bmi088::Init()
{
	accel_sen = Bmi088_N::accel_sen_6g;
	gyro_sen = Bmi088_N::gyro_sen_2000;

	const LibXR::ErrorCode ans_accel = AccelInit();
	const LibXR::ErrorCode ans_gyro = GyroInit();
	if (ans_accel != LibXR::ErrorCode::OK || ans_gyro != LibXR::ErrorCode::OK)
	{
		return LibXR::ErrorCode::FAILED;
	}

	for (uint8_t i = 0; i < 3; ++i)
	{
		data.gyro_offset[i] = 0.f;
	}
	data.accel_scale = 1.f;
	data.g_norm = 0.f;
	return LibXR::ErrorCode::OK;
}

void Bmi088::Read()
{
	int16_t raw[3] {};

	ReadAccelRaw(raw);
	for (uint8_t i = 0; i < 3; ++i)
	{
		data.accel[i] = static_cast<float>(raw[i]) * accel_sen * data.accel_scale;
	}

	if (ReadGyroRaw(raw))
	{
		for (uint8_t i = 0; i < 3; ++i)
		{
			data.gyro[i] = static_cast<float>(raw[i]) * gyro_sen - data.gyro_offset[i];
		}
	}

	data.temperature = ReadTemperature();
}

LibXR::ErrorCode Bmi088::CalibrateOffset(uint16_t sample_num)
{
	if (sample_num == 0)
	{
		return LibXR::ErrorCode::FAILED;
	}

	uint8_t id = 0;
	ReadRegs(accel_cs, acc_chip_id, &id, 1, 1);
	acc_id_read = id;

	float gyro_sum[3] {0.f, 0.f, 0.f};
	float gyro_max[3] {0.f, 0.f, 0.f};
	float gyro_min[3] {0.f, 0.f, 0.f};
	float norm_sum = 0.f;
	float norm_max = 0.f;
	float norm_min = 0.f;

	for (uint16_t i = 0; i < sample_num; ++i)
	{
		int16_t accel_raw[3] {};
		int16_t gyro_raw[3] {};
		ReadAccelRaw(accel_raw);
		ReadGyroRaw(gyro_raw);

		const float a0 = static_cast<float>(accel_raw[0]) * accel_sen;
		const float a1 = static_cast<float>(accel_raw[1]) * accel_sen;
		const float a2 = static_cast<float>(accel_raw[2]) * accel_sen;
		const float norm = sqrtf(a0 * a0 + a1 * a1 + a2 * a2);

		for (uint8_t j = 0; j < 3; ++j)
		{
			const float g = static_cast<float>(gyro_raw[j]) * gyro_sen;
			gyro_sum[j] += g;
			if (i == 0)
			{
				gyro_max[j] = g;
				gyro_min[j] = g;
			}
			else
			{
				if (g > gyro_max[j]) gyro_max[j] = g;
				if (g < gyro_min[j]) gyro_min[j] = g;
			}
		}

		norm_sum += norm;
		if (i == 0)
		{
			norm_max = norm;
			norm_min = norm;
		}
		else
		{
			if (norm > norm_max) norm_max = norm;
			if (norm < norm_min) norm_min = norm;
		}

		LibXR::Thread::Sleep(1);
	}

	float gyro_spread = 0.f;
	for (uint8_t j = 0; j < 3; ++j)
	{
		const float spread = gyro_max[j] - gyro_min[j];
		if (spread > gyro_spread) gyro_spread = spread;
	}
	const float norm_spread = norm_max - norm_min;
	if (gyro_spread > 0.15f || norm_spread > 0.5f)
	{
		return LibXR::ErrorCode::FAILED;
	}

	const float inv = 1.f / static_cast<float>(sample_num);
	for (uint8_t j = 0; j < 3; ++j)
	{
		data.gyro_offset[j] = gyro_sum[j] * inv;
	}
	data.g_norm = norm_sum * inv;
	if (data.g_norm > 8.f && data.g_norm < 12.f)
	{
		data.accel_scale = 9.81f / data.g_norm;
		return LibXR::ErrorCode::OK;
	}
	data.accel_scale = 1.f;
	return LibXR::ErrorCode::FAILED;
}
