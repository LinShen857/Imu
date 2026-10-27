# Imu

## 1. 模块作用

BMI088 六轴 IMU + VQF 姿态解算（含静止/运动双路零偏估计）。构造时完成芯片初始化与陀螺零偏标定，之后由使用方按自己的节奏（定时或 DRDY 中断）调用 `bmi088.WaitDataReady()` / `bmi088.Read()` / `vqf.Update()`。

BMI088 driver plus VQF attitude estimation with online gyro bias estimation; the caller drives the sampling loop.

## 2. 构造接口

```cpp
Imu(LibXR::SPI& spi, LibXR::GPIO& acc_cs, LibXR::GPIO& gyro_cs,
    LibXR::GPIO* acc_int);
```

依赖参数 / Dependencies:

- `LibXR::SPI& spi` — SPI 总线（mode 3）
- `LibXR::GPIO& acc_cs` / `LibXR::GPIO& gyro_cs` — 两片片选
- `LibXR::GPIO* acc_int` — 加速度计 INT1（用作 DRDY）；不用时配置写 `nullptr`

## 3. 公开成员

- `Bmi088 bmi088` — `Init()` / `Read()` / `CalibrateOffset()` / `WaitDataReady(ms)` / `data{gyro, accel, temperature}`
- `Vqf vqf` — `Update(gyro, accel, ts)` / `GetQuat()` / `GetEuler()` / `GetBias()` / `RestDetected()`
- `init_ans` / `acc_id` / `gyro_id` — 初始化结果与芯片 ID（期望 `0x1E` / `0x0F` / `0`）
- 常量在 `Imu_N`：`sample_freq_hz`、`sample_dt`、`drdy_timeout_ms`、`dt_min_us`、`dt_max_us`、`gyro_avg_tau`、`pitch_sign`
- `static Imu* Instance()`

## 4. 配置示例

```yaml
modules:
  - module: LinShen857/Imu
    id: imu
    args:
      - spi: spi2
      - acc_cs: ACC_CS
      - gyro_cs: GYRO_CS
      - acc_int: ACC_INT
```

## 5. 依赖与硬件

模块依赖 / Module dependencies:

无 / None

硬件 / Hardware:

- `LibXR::SPI`、`LibXR::GPIO` × 3
- 采样率必须与 `Imu_N::sample_dt` 一致，否则 VQF 的 Ts 失配
