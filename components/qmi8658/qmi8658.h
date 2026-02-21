#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace qmi8658 {

const float GRAVITY_EARTH = 9.80665f;

#pragma region Registers
const uint8_t QMI8658_REGISTER_WHO_AM_I = 0x00;     // Device Identifier
const uint8_t QMI8658_REGISTER_REVISION = 0x01;     // Device Revision ID
const uint8_t QMI8658_REGISTER_CTRL1 = 0x02;        // SPI Interface and Sensor Enable
const uint8_t QMI8658_REGISTER_CTRL2 = 0x03;        // Accelerometer: Output Data Rate, Full Scale, Self Test
const uint8_t QMI8658_REGISTER_CTRL3 = 0x04;        // Gyroscope: Output Data Rate, Full Scale, Self Test
const uint8_t QMI8658_REGISTER_CTRL4 = 0x05;        // Reserved
const uint8_t QMI8658_REGISTER_CTRL5 = 0x06;        // Low pass filter setting
const uint8_t QMI8658_REGISTER_CTRL6 = 0x07;        // AttitudeEngine™ Settings: Output Data Rate, Motion on Demand
const uint8_t QMI8658_REGISTER_CTRL7 = 0x08;        // Enable Sensors
const uint8_t QMI8658_REGISTER_CTRL8 = 0x09;        // Motion Detection Control
const uint8_t QMI8658_REGISTER_FIFO_WTM_TH = 0x13;  // FIFO watermark level, in ODRs
const uint8_t QMI8658_REGISTER_FIFO_CTRL = 0x14;    // FIFO Setup
const uint8_t QMI8658_REGISTER_FIFO_SMPL_CNT = 0x15;// FIFO sample count LSBs
const uint8_t QMI8658_REGISTER_FIFO_STATUS = 0x16;  // FIFO Status
const uint8_t QMI8658_REGISTER_FIFO_DATA = 0x17;    // FIFO Data
const uint8_t QMI8658_REGISTER_CTRL9 = 0x0A;        // Host Commands
const uint8_t QMI8658_REGISTER_CAL1_L = 0x0B;       // Calibration Register: lower 8 bits
const uint8_t QMI8658_REGISTER_CAL1_H = 0x0C;       // Calibration Register: upper 8 bits
const uint8_t QMI8658_REGISTER_STATUS_INT = 0x2D;   // Sensor Data Availability with the Locking mechanism
const uint8_t QMI8658_REGISTER_STATUS0 = 0x2E;      // Output Data Over Run and Data Availability
const uint8_t QMI8658_REGISTER_STATUS1 = 0x2F;      // Miscellaneous Status: Wake on Motion
const uint8_t QMI8658_REGISTER_TIMESTAMP_L = 0x30;  // Sample Time Stamp: lower 8 bits
const uint8_t QMI8658_REGISTER_TIMESTAMP_M = 0x31;  // Sample Time Stamp: middle 8 bits
const uint8_t QMI8658_REGISTER_TIMESTAMP_H = 0x32;  // Sample Time Stamp: upper 8 bits
const uint8_t QMI8658_REGISTER_TEMP_L = 0x33;       // Temperature Output Data: lower 8 bits
const uint8_t QMI8658_REGISTER_TEMP_H = 0x34;       // Temperature Output Data: upper 8 bits
const uint8_t QMI8658_REGISTER_AX_L = 0x35;         // X-axis Acceleration: lower 8 bits
const uint8_t QMI8658_REGISTER_AX_H = 0x36;         // X-axis Acceleration: upper 8 bits
const uint8_t QMI8658_REGISTER_AY_L = 0x37;         // Y-axis Acceleration: lower 8 bits
const uint8_t QMI8658_REGISTER_AY_H = 0x38;         // Y-axis Acceleration: upper 8 bits
const uint8_t QMI8658_REGISTER_AZ_L = 0x39;         // Z-axis Acceleration: lower 8 bits
const uint8_t QMI8658_REGISTER_AZ_H = 0x3A;         // Z-axis Acceleration: upper 8 bits
const uint8_t QMI8658_REGISTER_GX_L = 0x3B;         // X-axis Angular Rate: lower 8 bits
const uint8_t QMI8658_REGISTER_GX_H = 0x3C;         // X-axis Angular Rate: upper 8 bits
const uint8_t QMI8658_REGISTER_GY_L = 0x3D;         // X-axis Angular Rate: lower 8 bits
const uint8_t QMI8658_REGISTER_GY_H = 0x3E;         // X-axis Angular Rate: upper 8 bits
const uint8_t QMI8658_REGISTER_GZ_L = 0x3F;         // X-axis Angular Rate: lower 8 bits
const uint8_t QMI8658_REGISTER_GZ_H = 0x40;         // X-axis Angular Rate: upper 8 bits
#pragma endregion

#pragma region Commands
const uint8_t QMI8658_CMD_RST_FIFO = 0x04;
const uint8_t QMI8658_CMD_REQ_FIFO = 0x05;
const uint8_t QMI8658_CMD_WRITE_WOM_SETTING = 0x08;
#pragma endregion

#pragma region Sensors
const uint8_t QMI8658_SENSOR_NONE = 0;
const uint8_t QMI8658_SENSOR_ACCEL = 1 << 0;  // Accelerometer
const uint8_t QMI8658_SENSOR_GYRO = 1 << 1;   // Gyroscope
const uint8_t QMI8658_SENSOR_MAG = 1 << 2;    // Magnetometer
const uint8_t QMI8658_SENSOR_AE = 1 << 3;     // AttitudeEngine
#pragma endregion

#pragma region SPI Interface flags
const uint8_t QMI8658_SPI_BE = 1 << 5;  // Read data big endian
const uint8_t QMI8658_SPI_AI = 1 << 6;  // Address auto increment
#pragma endregion

#define FIELD_WITH_SETTER(type, name, def) \
 protected: \
  type name##_{def}; \
\
 public: \
  void set_##name(type value) { name##_ = value; }

enum QMI8658InterruptPin {
  QMI8658_INT1 = 0x00,
  QMI8658_INT2 = 0x01
};

enum QMI8658LPFMode {
  QMI8658_LPF_MODE_0 = 0x00,
  QMI8658_LPF_MODE_1 = 0x01,
  QMI8658_LPF_MODE_2 = 0x02,
  QMI8658_LPF_MODE_3 = 0x03,
  QMI8658_LPF_OFF = 0x04
};

enum QMI8658AccelRange {
  QMI8658_ACCEL_RANGE_2G = 0x00,
  QMI8658_ACCEL_RANGE_4G = 0x01,
  QMI8658_ACCEL_RANGE_8G = 0x02,
  QMI8658_ACCEL_RANGE_16G = 0x03
};

enum QMI8658AccelODR {
  QMI8658_ACCEL_ODR_8000HZ = 0x00,
  QMI8658_ACCEL_ODR_4000HZ = 0x01,
  QMI8658_ACCEL_ODR_2000HZ = 0x02,
  QMI8658_ACCEL_ODR_1000HZ = 0x03,
  QMI8658_ACCEL_ODR_500HZ = 0x04,
  QMI8658_ACCEL_ODR_250HZ = 0x05,
  QMI8658_ACCEL_ODR_125HZ = 0x06,
  QMI8658_ACCEL_ODR_62_5HZ = 0x07,
  QMI8658_ACCEL_ODR_31_25HZ = 0x08,
  QMI8658_ACCEL_ODR_LOWPOWER_128HZ = 0x0C,
  QMI8658_ACCEL_ODR_LOWPOWER_21HZ = 0x0D,
  QMI8658_ACCEL_ODR_LOWPOWER_11HZ = 0x0E,
  QMI8658_ACCEL_ODR_LOWPOWER_3HZ = 0x0F
};

enum QMI8658GyroRange {
  QMI8658_GYRO_RANGE_16DPS = 0x00,
  QMI8658_GYRO_RANGE_32DPS = 0x01,
  QMI8658_GYRO_RANGE_64DPS = 0x02,
  QMI8658_GYRO_RANGE_128DPS = 0x03,
  QMI8658_GYRO_RANGE_256DPS = 0x04,
  QMI8658_GYRO_RANGE_512DPS = 0x05,
  QMI8658_GYRO_RANGE_1024DPS = 0x06,
  QMI8658_GYRO_RANGE_2048DPS = 0x07
};

enum QMI8658GyroODR {
  QMI8658_GYRO_ODR_8000HZ = 0x00,
  QMI8658_GYRO_ODR_4000HZ = 0x01,
  QMI8658_GYRO_ODR_2000HZ = 0x02,
  QMI8658_GYRO_ODR_1000HZ = 0x03,
  QMI8658_GYRO_ODR_500HZ = 0x04,
  QMI8658_GYRO_ODR_250HZ = 0x05,
  QMI8658_GYRO_ODR_125HZ = 0x06,
  QMI8658_GYRO_ODR_62_5HZ = 0x07,
  QMI8658_GYRO_ODR_31_25HZ = 0x08
};

enum QMI8658FifoMode {
  QMI8658_FIFO_MODE_BYPASS = 0x00,
  QMI8658_FIFO_MODE_FIFO = 0x01,
  QMI8658_FIFO_MODE_STREAM = 0x02,
  QMI8658_FIFO_MODE_MAX = 0x03
};

enum QMI8658FifoSize {
  QMI8658_FIFO_SIZE_16 = 0x00,
  QMI8658_FIFO_SIZE_32 = 0x01,
  QMI8658_FIFO_SIZE_64 = 0x02,
  QMI8658_FIFO_SIZE_128 = 0x03
};

class QMI8658Component;

struct QMI8658SensorStore {
  volatile bool data_ready{true};
  ISRInternalGPIOPin pin;
  QMI8658Component *component_{nullptr};

  static void gpio_intr(QMI8658SensorStore *arg);
};

class QMI8658Component : public PollingComponent, public i2c::I2CDevice {
 public:
  FIELD_WITH_SETTER(InternalGPIOPin*, interrupt_pin_1, nullptr)
  FIELD_WITH_SETTER(InternalGPIOPin*, interrupt_pin_2, nullptr)
  FIELD_WITH_SETTER(QMI8658LPFMode, accel_lpf_mode, QMI8658_LPF_OFF)
  FIELD_WITH_SETTER(QMI8658AccelODR, accel_odr, QMI8658_ACCEL_ODR_8000HZ)
  FIELD_WITH_SETTER(QMI8658AccelRange, accel_range, QMI8658_ACCEL_RANGE_2G)
  FIELD_WITH_SETTER(QMI8658LPFMode, gyro_lpf_mode, QMI8658_LPF_OFF)
  FIELD_WITH_SETTER(QMI8658GyroODR, gyro_odr, QMI8658_GYRO_ODR_8000HZ)
  FIELD_WITH_SETTER(QMI8658GyroRange, gyro_range, QMI8658_GYRO_RANGE_16DPS)
  FIELD_WITH_SETTER(QMI8658FifoMode, fifo_mode, QMI8658_FIFO_MODE_BYPASS)
  FIELD_WITH_SETTER(QMI8658FifoSize, fifo_size, QMI8658_FIFO_SIZE_16)
  FIELD_WITH_SETTER(uint8_t, fifo_watermark, 8)
  SUB_SENSOR(accel_x)
  SUB_SENSOR(accel_y)
  SUB_SENSOR(accel_z)
  SUB_SENSOR(gyro_x)
  SUB_SENSOR(gyro_y)
  SUB_SENSOR(gyro_z)
  SUB_SENSOR(temperature)

  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;

  bool disable_wake_on_motion();
  bool enable_wake_on_motion(uint8_t threshold, QMI8658AccelODR accel_odr, QMI8658InterruptPin interrupt_pin,
                             uint8_t initial_pin_state, uint8_t blanking_time);

 protected:
  volatile float temp_{0};
  volatile float accel_x_{0};
  volatile float accel_y_{0};
  volatile float accel_z_{0};
  volatile float gyro_x_{0};
  volatile float gyro_y_{0};
  volatile float gyro_z_{0};

  bool clr_register_bit_(uint8_t reg, uint8_t bit);
  bool configure_accel_(QMI8658AccelODR accel_odr);
  bool configure_gyro_();
  bool configure_fifo_();
  bool disable_sensors_();
  void enable_data_ready_interrupt_();
  void enable_interrupt_(QMI8658InterruptPin interrupt_pin);
  bool enable_required_sensors_();
  bool enable_sensors_(uint8_t sensors_state);
  // HighFrequencyLoopRequester high_freq_;
  bool is_accel_required_() {
    return this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr || this->accel_z_sensor_ != nullptr;
  }
  bool is_gyro_required_() {
    return this->gyro_x_sensor_ != nullptr || this->gyro_y_sensor_ != nullptr || this->gyro_z_sensor_ != nullptr;
  }
  i2c::ErrorCode read_le_int16_(uint8_t reg, int16_t *value, uint8_t len);
  bool send_command_(uint8_t command, uint32_t wait_ms = 1000);
  bool set_register_bit_(uint8_t reg, uint8_t);
  QMI8658SensorStore store_{};
  void update_accel_sensor_(int16_t data_x, int16_t data_y, int16_t data_z);
  void update_gyro_sensor_(int16_t data_x, int16_t data_y, int16_t data_z);
  bool update_sensors_();
  void update_temp_sensor_(int16_t data);
};

}  // namespace qmi8658
}  // namespace esphome
