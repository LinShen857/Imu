#include "vqf.hpp"

#include <cmath>
#include <limits>

namespace
{
constexpr float kEps = 1e-8f;
constexpr float kSqrt2 = 1.4142135623730951f;
constexpr float kPi = 3.141592653589793f;

inline float Square(float x) { return x * x; }
inline float NaN() { return std::numeric_limits<float>::quiet_NaN(); }
inline bool IsNaN(float x) { return x != x; }
}

float Vqf::GainFromTau(float tau, float ts)
{
	if (tau < 0.f)
	{
		return 0.f;
	}
	if (tau == 0.f)
	{
		return 1.f;
	}
	return 1.f - expf(-ts / tau);
}

void Vqf::FilterCoeffs(float tau, float ts, float out_b[3], float out_a[2])
{
	if (tau < ts * 0.5f)
	{
		out_b[0] = 1.f;
		out_b[1] = 0.f;
		out_b[2] = 0.f;
		out_a[0] = 0.f;
		out_a[1] = 0.f;
		return;
	}

	const float fc = (kSqrt2 / (2.f * kPi)) / tau;
	const float c = tanf(kPi * fc * ts);
	const float d = c * c + kSqrt2 * c + 1.f;
	const float b0 = c * c / d;
	out_b[0] = b0;
	out_b[1] = 2.f * b0;
	out_b[2] = b0;
	out_a[0] = 2.f * (c * c - 1.f) / d;
	out_a[1] = (1.f - kSqrt2 * c + c * c) / d;
}

void Vqf::FilterInitialState(float x0, const float b[3], const float a[2], float out[2])
{
	out[0] = x0 * (1.f - b[0]);
	out[1] = x0 * (b[2] - a[1]);
}

float Vqf::FilterStep(float x, const float b[3], const float a[2], float state[2])
{
	const float y = b[0] * x + state[0];
	state[0] = b[1] * x - a[0] * y + state[1];
	state[1] = b[2] * x - a[1] * y;
	return y;
}

void Vqf::FilterVec(const float x[], uint8_t n, float tau, float ts, const float b[3],
	const float a[2], float state[], float out[])
{
	if (IsNaN(state[0]))
	{
		if (IsNaN(state[1]))
		{
			state[1] = 0.f;
			for (uint8_t i = 0; i < n; ++i)
			{
				state[2 + i] = 0.f;
			}
		}
		state[1] += 1.f;
		for (uint8_t i = 0; i < n; ++i)
		{
			state[2 + i] += x[i];
			out[i] = state[2 + i] / state[1];
		}
		if (state[1] * ts >= tau)
		{
			for (uint8_t i = 0; i < n; ++i)
			{
				FilterInitialState(out[i], b, a, &state[2 * i]);
			}
		}
		return;
	}

	for (uint8_t i = 0; i < n; ++i)
	{
		out[i] = FilterStep(x[i], b, a, &state[2 * i]);
	}
}

void Vqf::QuatMultiply(const float a[4], const float b[4], float out[4])
{
	const float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	const float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	const float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	const float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
	out[0] = w;
	out[1] = x;
	out[2] = y;
	out[3] = z;
}

void Vqf::QuatRotate(const float q[4], const float v[3], float out[3])
{
	const float t0 = 2.f * (q[2] * v[2] - q[3] * v[1]);
	const float t1 = 2.f * (q[3] * v[0] - q[1] * v[2]);
	const float t2 = 2.f * (q[1] * v[1] - q[2] * v[0]);
	out[0] = v[0] + q[0] * t0 + (q[2] * t2 - q[3] * t1);
	out[1] = v[1] + q[0] * t1 + (q[3] * t0 - q[1] * t2);
	out[2] = v[2] + q[0] * t2 + (q[1] * t1 - q[2] * t0);
}

float Vqf::Norm(const float v[3])
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void Vqf::Normalize(float v[], uint8_t n)
{
	float sum = 0.f;
	for (uint8_t i = 0; i < n; ++i)
	{
		sum += v[i] * v[i];
	}
	if (sum <= 0.f)
	{
		return;
	}
	const float inv = 1.f / sqrtf(sum);
	for (uint8_t i = 0; i < n; ++i)
	{
		v[i] *= inv;
	}
}

void Vqf::Clip(float v[], uint8_t n, float lo, float hi)
{
	for (uint8_t i = 0; i < n; ++i)
	{
		if (v[i] < lo)
		{
			v[i] = lo;
		}
		else if (v[i] > hi)
		{
			v[i] = hi;
		}
	}
}

void Vqf::Mat3SetScaledIdentity(float scale, float out[9])
{
	out[0] = scale;
	out[1] = 0.f;
	out[2] = 0.f;
	out[3] = 0.f;
	out[4] = scale;
	out[5] = 0.f;
	out[6] = 0.f;
	out[7] = 0.f;
	out[8] = scale;
}

void Vqf::Mat3Multiply(const float a[9], const float b[9], float out[9])
{
	float tmp[9] {};
	tmp[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
	tmp[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
	tmp[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
	tmp[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
	tmp[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
	tmp[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
	tmp[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
	tmp[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
	tmp[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
	for (uint8_t i = 0; i < 9; ++i)
	{
		out[i] = tmp[i];
	}
}

void Vqf::Mat3MultiplyTpsFirst(const float a[9], const float b[9], float out[9])
{
	float tmp[9] {};
	tmp[0] = a[0] * b[0] + a[3] * b[3] + a[6] * b[6];
	tmp[1] = a[0] * b[1] + a[3] * b[4] + a[6] * b[7];
	tmp[2] = a[0] * b[2] + a[3] * b[5] + a[6] * b[8];
	tmp[3] = a[1] * b[0] + a[4] * b[3] + a[7] * b[6];
	tmp[4] = a[1] * b[1] + a[4] * b[4] + a[7] * b[7];
	tmp[5] = a[1] * b[2] + a[4] * b[5] + a[7] * b[8];
	tmp[6] = a[2] * b[0] + a[5] * b[3] + a[8] * b[6];
	tmp[7] = a[2] * b[1] + a[5] * b[4] + a[8] * b[7];
	tmp[8] = a[2] * b[2] + a[5] * b[5] + a[8] * b[8];
	for (uint8_t i = 0; i < 9; ++i)
	{
		out[i] = tmp[i];
	}
}

void Vqf::Mat3MultiplyTpsSecond(const float a[9], const float b[9], float out[9])
{
	float tmp[9] {};
	tmp[0] = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	tmp[1] = a[0] * b[3] + a[1] * b[4] + a[2] * b[5];
	tmp[2] = a[0] * b[6] + a[1] * b[7] + a[2] * b[8];
	tmp[3] = a[3] * b[0] + a[4] * b[1] + a[5] * b[2];
	tmp[4] = a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
	tmp[5] = a[3] * b[6] + a[4] * b[7] + a[5] * b[8];
	tmp[6] = a[6] * b[0] + a[7] * b[1] + a[8] * b[2];
	tmp[7] = a[6] * b[3] + a[7] * b[4] + a[8] * b[5];
	tmp[8] = a[6] * b[6] + a[7] * b[7] + a[8] * b[8];
	for (uint8_t i = 0; i < 9; ++i)
	{
		out[i] = tmp[i];
	}
}

bool Vqf::Mat3Inv(const float in[9], float out[9])
{
	const double a = in[0];
	const double b = in[1];
	const double c = in[2];
	const double d = in[3];
	const double e = in[4];
	const double f = in[5];
	const double g = in[6];
	const double h = in[7];
	const double i = in[8];

	const double A = e * i - f * h;
	const double B = f * g - d * i;
	const double C = d * h - e * g;
	const double D = c * h - b * i;
	const double E = a * i - c * g;
	const double F = b * g - a * h;
	const double G = b * f - c * e;
	const double H = c * d - a * f;
	const double I = a * e - b * d;
	const double det = a * A + b * B + c * C;

	if (det > -kEps && det < kEps)
	{
		Mat3SetScaledIdentity(0.f, out);
		return false;
	}

	const double inv_det = 1.0 / det;
	out[0] = static_cast<float>(A * inv_det);
	out[1] = static_cast<float>(D * inv_det);
	out[2] = static_cast<float>(G * inv_det);
	out[3] = static_cast<float>(B * inv_det);
	out[4] = static_cast<float>(E * inv_det);
	out[5] = static_cast<float>(H * inv_det);
	out[6] = static_cast<float>(C * inv_det);
	out[7] = static_cast<float>(F * inv_det);
	out[8] = static_cast<float>(I * inv_det);
	return true;
}

Vqf::Vqf(float ts_)
{
	gyr_ts = ts_;
	acc_ts = ts_;

	FilterCoeffs(Vqf_N::tau_acc, acc_ts, acc_lp_b, acc_lp_a);
	FilterCoeffs(Vqf_N::rest_filter_tau, gyr_ts, rest_gyr_lp_b, rest_gyr_lp_a);
	FilterCoeffs(Vqf_N::rest_filter_tau, acc_ts, rest_acc_lp_b, rest_acc_lp_a);

	bias_p0 = Square(Vqf_N::bias_sigma_init_deg * 100.f);
	bias_v = Square(0.1f * 100.f) * acc_ts / Vqf_N::bias_forgetting_time;
	const float p_motion = Square(Vqf_N::bias_sigma_motion_deg * 100.f);
	bias_motion_w = Square(p_motion) / bias_v + p_motion;
	bias_vertical_w = bias_motion_w /
		(Vqf_N::bias_vertical_forgetting_factor > 1e-10f ? Vqf_N::bias_vertical_forgetting_factor : 1e-10f);
	const float p_rest = Square(Vqf_N::bias_sigma_rest_deg * 100.f);
	bias_rest_w = Square(p_rest) / bias_v + p_rest;

	Reset();
}

void Vqf::Reset()
{
	gyr_quat[0] = 1.f;
	gyr_quat[1] = 0.f;
	gyr_quat[2] = 0.f;
	gyr_quat[3] = 0.f;
	acc_quat[0] = 1.f;
	acc_quat[1] = 0.f;
	acc_quat[2] = 0.f;
	acc_quat[3] = 0.f;

	for (uint8_t i = 0; i < 3; ++i)
	{
		last_acc_lp[i] = 0.f;
		rest_last_gyr_lp[i] = 0.f;
		rest_last_acc_lp[i] = 0.f;
	}
	for (uint8_t i = 0; i < 6; ++i)
	{
		acc_lp_state[i] = NaN();
		rest_gyr_lp_state[i] = NaN();
		rest_acc_lp_state[i] = NaN();
	}
	rest_last_sq_dev[0] = 0.f;
	rest_last_sq_dev[1] = 0.f;
	rest_t = 0.f;
	rest_detected = false;

	for (uint8_t i = 0; i < 3; ++i)
	{
		bias[i] = 0.f;
	}
	Mat3SetScaledIdentity(bias_p0, bias_p);
	for (uint8_t i = 0; i < 18; ++i)
	{
		motion_bias_est_r_lp_state[i] = NaN();
	}
	for (uint8_t i = 0; i < 4; ++i)
	{
		motion_bias_est_bias_lp_state[i] = NaN();
	}
}

void Vqf::GetBias(float bias_out[3]) const
{
	for (uint8_t i = 0; i < 3; ++i)
	{
		bias_out[i] = bias[i];
	}
}

void Vqf::SetBias(const float bias_in[3], float sigma)
{
	for (uint8_t i = 0; i < 3; ++i)
	{
		bias[i] = bias_in[i];
	}
	if (sigma > 0.f)
	{
		Mat3SetScaledIdentity(Square(sigma * (180.f * 100.f / kPi)), bias_p);
	}
}

float Vqf::GetBiasSigma() const
{
	const float sum0 = fabsf(bias_p[0]) + fabsf(bias_p[1]) + fabsf(bias_p[2]);
	const float sum1 = fabsf(bias_p[3]) + fabsf(bias_p[4]) + fabsf(bias_p[5]);
	const float sum2 = fabsf(bias_p[6]) + fabsf(bias_p[7]) + fabsf(bias_p[8]);
	float p = sum0 > sum1 ? sum0 : sum1;
	if (sum2 > p)
	{
		p = sum2;
	}
	const float sigma = sqrtf(p) * (kPi / 100.f) / 180.f;
	const float sigma_max = Vqf_N::bias_sigma_init_deg * Vqf_N::deg_to_rad;
	return sigma < sigma_max ? sigma : sigma_max;
}

void Vqf::UpdateGyr(const float gyr[3])
{
	float gyr_lp[3] {};
	FilterVec(gyr, 3, Vqf_N::rest_filter_tau, gyr_ts, rest_gyr_lp_b, rest_gyr_lp_a,
		rest_gyr_lp_state, gyr_lp);

	rest_last_sq_dev[0] = Square(gyr[0] - gyr_lp[0]) + Square(gyr[1] - gyr_lp[1]) +
		Square(gyr[2] - gyr_lp[2]);

	const float bias_clip = Vqf_N::bias_clip_deg * Vqf_N::deg_to_rad;
	const float th_gyr = Vqf_N::rest_th_gyr_deg * Vqf_N::deg_to_rad;
	if (rest_last_sq_dev[0] >= Square(th_gyr) || fabsf(gyr_lp[0]) > bias_clip ||
		fabsf(gyr_lp[1]) > bias_clip || fabsf(gyr_lp[2]) > bias_clip)
	{
		rest_t = 0.f;
		rest_detected = false;
	}
	for (uint8_t i = 0; i < 3; ++i)
	{
		rest_last_gyr_lp[i] = gyr_lp[i];
	}

	float gyr_no_bias[3] {};
	for (uint8_t i = 0; i < 3; ++i)
	{
		gyr_no_bias[i] = gyr[i] - bias[i];
	}

	const float gyr_norm = Norm(gyr_no_bias);
	if (gyr_norm > kEps)
	{
		const float angle = gyr_norm * gyr_ts;
		const float c = cosf(angle * 0.5f);
		const float s = sinf(angle * 0.5f) / gyr_norm;
		const float step[4] = {c, s * gyr_no_bias[0], s * gyr_no_bias[1], s * gyr_no_bias[2]};
		QuatMultiply(gyr_quat, step, gyr_quat);
		Normalize(gyr_quat, 4);
	}
}

void Vqf::UpdateAcc(const float acc[3])
{
	if (acc[0] == 0.f && acc[1] == 0.f && acc[2] == 0.f)
	{
		return;
	}

	float acc_lp[3] {};
	FilterVec(acc, 3, Vqf_N::rest_filter_tau, acc_ts, rest_acc_lp_b, rest_acc_lp_a,
		rest_acc_lp_state, acc_lp);

	rest_last_sq_dev[1] = Square(acc[0] - acc_lp[0]) + Square(acc[1] - acc_lp[1]) +
		Square(acc[2] - acc_lp[2]);
	if (rest_last_sq_dev[1] >= Square(Vqf_N::rest_th_acc))
	{
		rest_t = 0.f;
		rest_detected = false;
	}
	else
	{
		rest_t += acc_ts;
		if (rest_t >= Vqf_N::rest_min_t)
		{
			rest_detected = true;
		}
	}
	for (uint8_t i = 0; i < 3; ++i)
	{
		rest_last_acc_lp[i] = acc_lp[i];
	}

	float acc_earth[3] {};
	QuatRotate(gyr_quat, acc, acc_earth);
	FilterVec(acc_earth, 3, Vqf_N::tau_acc, acc_ts, acc_lp_b, acc_lp_a, acc_lp_state,
		last_acc_lp);

	QuatRotate(acc_quat, last_acc_lp, acc_earth);
	Normalize(acc_earth, 3);

	float corr[4] {};
	const float q_w = sqrtf((acc_earth[2] + 1.f) * 0.5f);
	if (q_w > 1e-6f)
	{
		corr[0] = q_w;
		corr[1] = 0.5f * acc_earth[1] / q_w;
		corr[2] = -0.5f * acc_earth[0] / q_w;
		corr[3] = 0.f;
	}
	else
	{
		corr[0] = 0.f;
		corr[1] = 1.f;
		corr[2] = 0.f;
		corr[3] = 0.f;
	}
	QuatMultiply(corr, acc_quat, acc_quat);
	Normalize(acc_quat, 4);

	if (Vqf_N::rest_bias_est_enabled || Vqf_N::motion_bias_est_enabled)
	{
		const float bias_clip = Vqf_N::bias_clip_deg * Vqf_N::deg_to_rad;

		float quat6d[4] {};
		QuatMultiply(acc_quat, gyr_quat, quat6d);

		float rot[9] {};
		rot[0] = 1.f - 2.f * Square(quat6d[2]) - 2.f * Square(quat6d[3]);
		rot[1] = 2.f * (quat6d[2] * quat6d[1] - quat6d[0] * quat6d[3]);
		rot[2] = 2.f * (quat6d[0] * quat6d[2] + quat6d[3] * quat6d[1]);
		rot[3] = 2.f * (quat6d[0] * quat6d[3] + quat6d[2] * quat6d[1]);
		rot[4] = 1.f - 2.f * Square(quat6d[1]) - 2.f * Square(quat6d[3]);
		rot[5] = 2.f * (quat6d[2] * quat6d[3] - quat6d[1] * quat6d[0]);
		rot[6] = 2.f * (quat6d[3] * quat6d[1] - quat6d[0] * quat6d[2]);
		rot[7] = 2.f * (quat6d[0] * quat6d[1] + quat6d[3] * quat6d[2]);
		rot[8] = 1.f - 2.f * Square(quat6d[1]) - 2.f * Square(quat6d[2]);

		float bias_lp[2] {};
		bias_lp[0] = rot[0] * bias[0] + rot[1] * bias[1] + rot[2] * bias[2];
		bias_lp[1] = rot[3] * bias[0] + rot[4] * bias[1] + rot[5] * bias[2];

		FilterVec(rot, 9, Vqf_N::tau_acc, acc_ts, acc_lp_b, acc_lp_a,
			motion_bias_est_r_lp_state, rot);
		FilterVec(bias_lp, 2, Vqf_N::tau_acc, acc_ts, acc_lp_b, acc_lp_a,
			motion_bias_est_bias_lp_state, bias_lp);

		float w[3] {};
		float e[3] {};
		if (rest_detected && Vqf_N::rest_bias_est_enabled)
		{
			for (uint8_t i = 0; i < 3; ++i)
			{
				e[i] = rest_last_gyr_lp[i] - bias[i];
				w[i] = bias_rest_w;
			}
			Mat3SetScaledIdentity(1.f, rot);
		}
		else if (Vqf_N::motion_bias_est_enabled)
		{
			e[0] = -acc_earth[1] / acc_ts + bias_lp[0] - rot[0] * bias[0] -
				rot[1] * bias[1] - rot[2] * bias[2];
			e[1] = acc_earth[0] / acc_ts + bias_lp[1] - rot[3] * bias[0] -
				rot[4] * bias[1] - rot[5] * bias[2];
			e[2] = -rot[6] * bias[0] - rot[7] * bias[1] - rot[8] * bias[2];
			w[0] = bias_motion_w;
			w[1] = bias_motion_w;
			w[2] = bias_vertical_w;
		}
		else
		{
			w[0] = -1.f;
		}

		if (bias_p[0] < bias_p0)
		{
			bias_p[0] += bias_v;
		}
		if (bias_p[4] < bias_p0)
		{
			bias_p[4] += bias_v;
		}
		if (bias_p[8] < bias_p0)
		{
			bias_p[8] += bias_v;
		}

		if (w[0] >= 0.f)
		{
			Clip(e, 3, -bias_clip, bias_clip);

			float gain[9] {};
			Mat3MultiplyTpsSecond(bias_p, rot, gain);
			Mat3Multiply(rot, gain, gain);
			gain[0] += w[0];
			gain[4] += w[1];
			gain[8] += w[2];
			if (Mat3Inv(gain, gain))
			{
				Mat3MultiplyTpsFirst(rot, gain, gain);
				Mat3Multiply(bias_p, gain, gain);

				for (uint8_t i = 0; i < 3; ++i)
				{
					bias[i] += gain[3 * i + 0] * e[0] + gain[3 * i + 1] * e[1] +
						gain[3 * i + 2] * e[2];
				}

				Mat3Multiply(gain, rot, gain);
				Mat3Multiply(gain, bias_p, gain);
				for (uint8_t i = 0; i < 9; ++i)
				{
					bias_p[i] -= gain[i];
				}

				Clip(bias, 3, -bias_clip, bias_clip);
			}
		}
	}
}

void Vqf::Update(const float gyr[3], const float acc[3], float ts)
{
	if (ts > 0.f)
	{
		gyr_ts = ts;
		acc_ts = ts;
	}

	UpdateGyr(gyr);
	UpdateAcc(acc);
}

void Vqf::GetQuat(float q[4]) const
{
	QuatMultiply(acc_quat, gyr_quat, q);
}

void Vqf::GetEuler(float rpy[3]) const
{
	float q[4] {};
	GetQuat(q);
	rpy[0] = atan2f(2.f * (q[0] * q[1] + q[2] * q[3]),
		1.f - 2.f * (q[1] * q[1] + q[2] * q[2]));
	const float sin_p = 2.f * (q[0] * q[2] - q[3] * q[1]);
	rpy[1] = asinf(sin_p > 1.f ? 1.f : (sin_p < -1.f ? -1.f : sin_p));
	rpy[2] = atan2f(2.f * (q[0] * q[3] + q[1] * q[2]),
		1.f - 2.f * (q[2] * q[2] + q[3] * q[3]));
}
