#pragma once

#include <cstdint>

namespace Vqf_N
{
inline constexpr float deg_to_rad = 0.017453292519943295f;

inline constexpr float tau_acc = 3.0f;
inline constexpr float rest_filter_tau = 0.5f;
inline constexpr float rest_th_gyr_deg = 4.0f;
inline constexpr float rest_th_acc = 0.5f;
inline constexpr float rest_min_t = 1.0f;
inline constexpr float bias_clip_deg = 2.0f;

inline constexpr bool rest_bias_est_enabled = true;
inline constexpr bool motion_bias_est_enabled = true;
inline constexpr float bias_sigma_init_deg = 0.5f;
inline constexpr float bias_forgetting_time = 100.0f;
inline constexpr float bias_sigma_motion_deg = 0.1f;
inline constexpr float bias_vertical_forgetting_factor = 0.0001f;
inline constexpr float bias_sigma_rest_deg = 0.03f;
}

class Vqf
{
	public:
	explicit Vqf(float ts_);

	void UpdateGyr(const float gyr_[3]);
	void UpdateAcc(const float acc_[3]);
	void Update(const float gyr_[3], const float acc_[3], float ts_ = 0.f);
	void GetQuat(float q_[4]) const;
	void GetEuler(float rpy_[3]) const;
	void GetBias(float bias_[3]) const;
	void SetBias(const float bias_[3], float sigma_ = -1.f);
	float GetBiasSigma() const;
	void Reset();

	bool RestDetected() const { return rest_detected; }
	float RestDevGyr() const { return rest_last_sq_dev[0]; }
	float RestDevAcc() const { return rest_last_sq_dev[1]; }

	private:
	static float GainFromTau(float tau_, float ts_);
	static void FilterCoeffs(float tau_, float ts_, float b_[3], float a_[2]);
	static void FilterInitialState(float x0_, const float b_[3], const float a_[2], float out_[2]);
	static float FilterStep(float x_, const float b_[3], const float a_[2], float state_[2]);
	static void FilterVec(const float x_[], uint8_t n_, float tau_, float ts_, const float b_[3],
		const float a_[2], float state_[], float out_[]);
	static void QuatMultiply(const float a_[4], const float b_[4], float out_[4]);
	static void QuatRotate(const float q_[4], const float v_[3], float out_[3]);
	static float Norm(const float v_[3]);
	static void Normalize(float v_[], uint8_t n_);
	static void Clip(float v_[], uint8_t n_, float lo_, float hi_);
	static void Mat3SetScaledIdentity(float scale_, float out_[9]);
	static void Mat3Multiply(const float a_[9], const float b_[9], float out_[9]);
	static void Mat3MultiplyTpsFirst(const float a_[9], const float b_[9], float out_[9]);
	static void Mat3MultiplyTpsSecond(const float a_[9], const float b_[9], float out_[9]);
	static bool Mat3Inv(const float in_[9], float out_[9]);

	float gyr_ts;
	float acc_ts;
	float acc_lp_b[3] {};
	float acc_lp_a[2] {};
	float rest_gyr_lp_b[3] {};
	float rest_gyr_lp_a[2] {};
	float rest_acc_lp_b[3] {};
	float rest_acc_lp_a[2] {};

	float gyr_quat[4] {};
	float acc_quat[4] {};
	float acc_lp_state[6] {};
	float rest_gyr_lp_state[6] {};
	float rest_acc_lp_state[6] {};
	float last_acc_lp[3] {};
	float rest_last_gyr_lp[3] {};
	float rest_last_acc_lp[3] {};
	float rest_last_sq_dev[2] {};
	float rest_t = 0.f;
	bool rest_detected = false;

	float bias[3] {};
	float bias_p[9] {};
	float bias_p0 = 0.f;
	float bias_v = 0.f;
	float bias_motion_w = 0.f;
	float bias_vertical_w = 0.f;
	float bias_rest_w = 0.f;
	float motion_bias_est_r_lp_state[18] {};
	float motion_bias_est_bias_lp_state[4] {};
};
