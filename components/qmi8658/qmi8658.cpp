#include "qmi8658.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace qmi8658 {

static const char *const TAG = "qmi8658";

void QMI8658Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up QMI8658...");
  uint8_t chipid;
  if (this->read_byte(QMI8658_REGISTER_WHO_AM_I, &chipid)) {
    if (chipid != 0x05) {
      ESP_LOGE(TAG, "This is not a QMI8658 chip");
      this->mark_failed();
      return;
    }
  } else {
    ESP_LOGE(TAG, "Can't read WHO_AM_I register");
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up SPI Interface...");
  uint8_t spi_config = QMI8658_SPI_BE | QMI8658_SPI_AI;
  ESP_LOGV(TAG, "  spi_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(spi_config));
  if (!this->write_byte(QMI8658_REGISTER_CTRL1, spi_config)) {
    ESP_LOGE(TAG, "Can't configure SPI Interface");
    this->mark_failed();
    return;
  }

  if (!this->configure_accel_(this->accel_odr_)) {
    this->mark_failed();
    return;
  }

  if (!this->configure_gyro_()) {
    this->mark_failed();
    return;
  }

  if (!this->configure_fifo_()) {
    this->mark_failed();
    return;
  }

  ESP_LOGV(TAG, "  Setting up LPF config...");
  uint8_t lpf_config = 0;
  if (this->accel_lpf_mode_ != QMI8658_LPF_OFF) {
    lpf_config |= (uint8_t) this->accel_lpf_mode_ << 1 | 1 << 0;
  }
  if (this->gyro_lpf_mode_ != QMI8658_LPF_OFF) {
    lpf_config |= (uint8_t) this->gyro_lpf_mode_ << 5 | 1 << 4;
  }
  ESP_LOGV(TAG, "  lpf_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(lpf_config));
  if (!this->write_byte(QMI8658_REGISTER_CTRL5, lpf_config)) {
    ESP_LOGE(TAG, "Can't configure LPF");
    this->mark_failed();
    return;
  }

  if (!this->enable_required_sensors_()) {
    this->mark_failed();
    return;
  }

  if (this->interrupt_pin_1_ != nullptr) {
    ESP_LOGCONFIG(TAG, "Setting up interrupt pin...");
    // this->high_freq_.start();
    this->interrupt_pin_1_->setup();
    this->store_.pin = this->interrupt_pin_1_->to_isr();
    this->store_.component_ = this;
    this->interrupt_pin_1_->attach_interrupt(QMI8658SensorStore::gpio_intr, &this->store_, gpio::INTERRUPT_RISING_EDGE);
    this->enable_interrupt_(QMI8658_INT1);
    this->enable_data_ready_interrupt_();
  }
  if (this->interrupt_pin_2_ != nullptr) {
    ESP_LOGCONFIG(TAG, "Setting up interrupt pin...");
    // this->high_freq_.start();
    this->interrupt_pin_2_->setup();
    this->store_.pin = this->interrupt_pin_2_->to_isr();
    this->store_.component_ = this;
    this->interrupt_pin_2_->attach_interrupt(QMI8658SensorStore::gpio_intr, &this->store_, gpio::INTERRUPT_RISING_EDGE);
    this->enable_interrupt_(QMI8658_INT2);
    this->enable_data_ready_interrupt_();
  }
}

void QMI8658Component::dump_config() {
  ESP_LOGCONFIG(TAG, "QMI8658:");
  if (this->interrupt_pin_1_ != nullptr) {
    LOG_PIN("  Interrupt pin 1: ", this->interrupt_pin_1_);
  }
  if (this->interrupt_pin_2_ != nullptr) {
    LOG_PIN("  Interrupt pin 2: ", this->interrupt_pin_2_);
  }
  ESP_LOGCONFIG(TAG, "    Acceleration ODR: %u", this->accel_odr_);
  ESP_LOGCONFIG(TAG, "    Acceleration range: %u", this->accel_range_);
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with QMI8658 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyroscope X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyroscope Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyroscope Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void QMI8658Component::loop() {
  if (this->interrupt_pin_1_ != nullptr || this->interrupt_pin_2_ != nullptr) {
    if (this->store_.data_ready) {
      this->store_.data_ready = false;
      this->update_sensors_();
    } else this->disable_loop();
  }
}

void QMI8658Component::update() {
  if (this->interrupt_pin_1_ != nullptr || this->interrupt_pin_2_ != nullptr) return;
  if (!this->update_sensors_()) return;

  this->status_clear_warning();
}

void QMI8658Component::update_accel_sensor_(int16_t data_x, int16_t data_y, int16_t data_z) {
  this->accel_x_ = (float) data_x / (float) 32768.f * (1 << ((uint8_t) this->accel_range_ + 1)) * GRAVITY_EARTH;
  this->accel_y_ = (float) data_y / (float) 32768.f * (1 << ((uint8_t) this->accel_range_ + 1)) * GRAVITY_EARTH;
  this->accel_z_ = (float) data_z / (float) 32768.f * (1 << ((uint8_t) this->accel_range_ + 1)) * GRAVITY_EARTH;
  if (this->accel_x_sensor_ != nullptr) this->accel_x_sensor_->publish_state(this->accel_x_);
  if (this->accel_y_sensor_ != nullptr) this->accel_y_sensor_->publish_state(this->accel_y_);
  if (this->accel_z_sensor_ != nullptr) this->accel_z_sensor_->publish_state(this->accel_z_);
  ESP_LOGVV(TAG, "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}", this->accel_x_, this->accel_y_, this->accel_z_);
}

void QMI8658Component::update_gyro_sensor_(int16_t data_x, int16_t data_y, int16_t data_z) {
  this->gyro_x_ = (float) data_x / (float) 32768.f * (1 << ((uint8_t) this->gyro_range_ + 4));
  this->gyro_y_ = (float) data_y / (float) 32768.f * (1 << ((uint8_t) this->gyro_range_ + 4));
  this->gyro_z_ = (float) data_z / (float) 32768.f * (1 << ((uint8_t) this->gyro_range_ + 4));
  if (this->gyro_x_sensor_ != nullptr) this->gyro_x_sensor_->publish_state(this->gyro_x_);
  if (this->gyro_y_sensor_ != nullptr) this->gyro_y_sensor_->publish_state(this->gyro_y_);
  if (this->gyro_z_sensor_ != nullptr) this->gyro_z_sensor_->publish_state(this->gyro_z_);
  ESP_LOGVV(TAG, "Got gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s}", this->gyro_x_, this->gyro_y_, this->gyro_z_);
}

bool QMI8658Component::update_sensors_() {
  ESP_LOGV(TAG, "    Updating QMI8658...");
  int16_t data[3];

  if (this->temperature_sensor_ != nullptr) {
    if (this->read_le_int16_(QMI8658_REGISTER_TEMP_L, data, 1) != i2c::ERROR_OK) {
      this->status_set_warning("Error reading temperature data register");
      return false;
    }
    this->update_temp_sensor_(data[0]);
  }

  if (this->fifo_mode_ == QMI8658_FIFO_MODE_BYPASS) {
    if (this->is_accel_required_()) {
      if (this->read_le_int16_(QMI8658_REGISTER_AX_L, data, 3) != i2c::ERROR_OK) {
        this->status_set_warning("Error reading acceleration data register");
        return false;
      }
      this->update_accel_sensor_(data[0], data[1], data[2]);
    }

    if (this->is_gyro_required_()) {
      if (this->read_le_int16_(QMI8658_REGISTER_GX_L, data, 3) != i2c::ERROR_OK) {
        this->status_set_warning("Error reading gyroscope data register");
        return false;
      }
      this->update_gyro_sensor_(data[0], data[1], data[2]);
    }
  } else {
    uint8_t val;
    
    if (this->read_register(QMI8658_REGISTER_FIFO_STATUS, &val, 1) != i2c::ERROR_OK) {
      this->status_set_warning("Error reading FIFO status register");
      return false;
    }
    ESP_LOGV(TAG, "  fifo status: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(val));
    if (val & (1 << 5)) ESP_LOGV(TAG, "FIFO overflowed (data dropped)");
    if (val & (1 << 6)) ESP_LOGV(TAG, "FIFO hit watermark level");
    if (val & (1 << 7)) ESP_LOGV(TAG, "FIFO is full");

    if (this->read_le_int16_(QMI8658_REGISTER_FIFO_SMPL_CNT, data, 1) != i2c::ERROR_OK) {
      this->status_set_warning("Error reading FIFO sample count register");
      return false;
    }
    const uint8_t sensor_count = this->is_accel_required_() + this->is_gyro_required_();
    const uint16_t frame_count = ((uint16_t) data[0]) & 0x03FF;
    const uint8_t sample_count = frame_count / (sensor_count * 3);
    const uint8_t values_per_sample = 3 * sensor_count;
    ESP_LOGV(TAG, "sensor_count: %d, frame_count: %d, sample_count: %d", sensor_count, frame_count, sample_count);

    ESP_LOGV(TAG, "  Requesting FIFO...");
    if (!this->send_command_(QMI8658_CMD_REQ_FIFO)) {
      this->status_set_warning("Error requesting FIFO");
      return false;
    }
    
//    uint32_t start = millis();
    int16_t words[frame_count];
    if (this->read_le_int16_(QMI8658_REGISTER_FIFO_DATA, words, frame_count) != i2c::ERROR_OK) {
      this->status_set_warning("Error reading FIFO data register");
      return false;
    }
//    ESP_LOGE(TAG, "read took %d ms", millis() - start);

//    for (uint8_t i = 0; i < sample_count; i++) {
//      ESP_LOGE(TAG, "sample %d: %d %d %d", i, words[i * 6], words[i * 6 + 1], words[i * 6 + 2]);
//      ESP_LOGE(TAG, "sample %d: %d %d %d", i, words[i * 6 + 3], words[i * 6 + 4], words[i * 6 + 5]);
//    }

    for (uint8_t i = 0; i < sample_count; i++) {
      uint16_t base = i * values_per_sample;
      uint8_t offset = 0;

      if (this->is_accel_required_()) {
        ESP_LOGE(TAG, "%d,%.2f", millis(), (float) words[base + offset + 1] / (float) 32768.f * (1 << ((uint8_t) this->accel_range_ + 1)) * GRAVITY_EARTH);
        this->update_accel_sensor_(words[base + offset], words[base + offset + 1], words[base + offset + 2]);
        offset += 3;
      }

      if (this->is_gyro_required_()) {
        this->update_gyro_sensor_(words[base + offset], words[base + offset + 1], words[base + offset + 2]);
      }
    }

    ESP_LOGV(TAG, "  Clearing FIFO read mode bit...");
    if (!this->clr_register_bit_(QMI8658_REGISTER_FIFO_CTRL, 7)) {
      this->status_set_warning("Error clearing FIFO read mode bit");
      return false;
    }
  }

  return true;
}

void QMI8658Component::update_temp_sensor_(int16_t data) {
  this->temp_ = (float) data / 32768.f * 64.5f + 23.f;
  if (this->temperature_sensor_ != nullptr) this->temperature_sensor_->publish_state(this->temp_);
  ESP_LOGVV(TAG, "Got temp=%.3f°C", this->temp_);
}

bool QMI8658Component::clr_register_bit_(uint8_t reg, uint8_t bit) {
  uint8_t value;
  if (this->read_register(reg, &value, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Error reading register");
    return false;
  }
  value &= ~(1 << bit);
  if (this->write_register(reg, &value, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Error writing register");
    return false;
  }
  return true;
}

bool QMI8658Component::configure_accel_(QMI8658AccelODR accel_odr) {
  ESP_LOGV(TAG, "  Setting up Accel Config...");
  uint8_t accel_config = (uint8_t) this->accel_range_ << 4 | (uint8_t) accel_odr;
  ESP_LOGV(TAG, "  accel_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(accel_config));
  if (!this->write_byte(QMI8658_REGISTER_CTRL2, accel_config)) {
    ESP_LOGE(TAG, "Can't configure accelerometer");
    return false;
  }
  return true;
}

bool QMI8658Component::configure_gyro_() {
  ESP_LOGV(TAG, "  Setting up Gyro Config...");
  uint8_t gyro_config = (uint8_t) this->gyro_range_ << 4 | (uint8_t) this->gyro_odr_;
  ESP_LOGV(TAG, "  gyro_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(gyro_config));
  if (!this->write_byte(QMI8658_REGISTER_CTRL3, gyro_config)) {
    ESP_LOGE(TAG, "Can't configure gyroscope");
    return false;
  }
  return true;
}

bool QMI8658Component::configure_fifo_() {
  ESP_LOGV(TAG, "  Setting up FIFO Config...");
  uint8_t fifo_config = (uint8_t) this->fifo_size_ << 2 | (uint8_t) this->fifo_mode_;
  ESP_LOGV(TAG, "  fifo_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(fifo_config));
  if (!this->write_byte(QMI8658_REGISTER_FIFO_CTRL, fifo_config)) {
    ESP_LOGE(TAG, "Can't configure FIFO");
    return false;
  }

  ESP_LOGV(TAG, "  Setting up FIFO watermark...");
  ESP_LOGV(TAG, "  fifo_watermark: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(this->fifo_watermark_));
  if (!this->write_byte(QMI8658_REGISTER_FIFO_WTM_TH, this->fifo_watermark_)) {
    ESP_LOGE(TAG, "Can't configure FIFO watermark");
    return false;
  }

  ESP_LOGV(TAG, "  Resetting FIFO...");
  if (!this->send_command_(QMI8658_CMD_RST_FIFO)) {
    this->status_set_warning("Error resetting FIFO");
    return false;
  }
  return true;
}

bool QMI8658Component::disable_sensors_() {
  ESP_LOGV(TAG, "  Disabling sensors...");
  ESP_LOGV(TAG, "  sensors_state: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(QMI8658_SENSOR_NONE));
  return this->enable_sensors_(QMI8658_SENSOR_NONE);
}

bool QMI8658Component::disable_wake_on_motion() {
  if (!this->disable_sensors_()) {
    this->status_set_warning("Error disabling WoM: disabling sensors");
    return false;
  }

  ESP_LOGV(TAG, "  Disabling WoM threshold...");
  uint8_t threshold = 0x00;
  ESP_LOGV(TAG, "  threshold: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(threshold));
  if (this->write_register(QMI8658_REGISTER_CAL1_L, &threshold, 1) != i2c::ERROR_OK) {
    this->status_set_warning("Error disabling WoM: disabling threshold");
    return false;
  }

  ESP_LOGV(TAG, "  Saving WoM settings...");
  if (!this->send_command_(QMI8658_CMD_WRITE_WOM_SETTING)) {
    this->status_set_warning("Error disabling WoM: saving WoM settings");
    return false;
  }
  return this->enable_required_sensors_();
}

void QMI8658Component::enable_data_ready_interrupt_() {
  this->clr_register_bit_(QMI8658_REGISTER_CTRL7, 5);
}

void QMI8658Component::enable_interrupt_(QMI8658InterruptPin interrupt_pin) {
  ESP_LOGV(TAG, "  Enabling interrupt...");
  switch (interrupt_pin) {
    case QMI8658_INT1:
      this->set_register_bit_(QMI8658_REGISTER_CTRL1, 3);
      break;
    case QMI8658_INT2:
      this->set_register_bit_(QMI8658_REGISTER_CTRL1, 4);
      break;
    }
}

bool QMI8658Component::enable_required_sensors_() {
  ESP_LOGV(TAG, "  Enabling sensors...");
  uint8_t sensors_state = QMI8658_SENSOR_NONE;
  if (this->is_accel_required_())
    sensors_state |= QMI8658_SENSOR_ACCEL;
  if (this->is_gyro_required_())
    sensors_state |= QMI8658_SENSOR_GYRO;
  ESP_LOGV(TAG, "  sensors_state: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(sensors_state));
  if (!this->enable_sensors_(sensors_state)) {
    ESP_LOGE(TAG, "Can't enable required sensors");
    return false;
  }
  return true;
}

bool QMI8658Component::enable_sensors_(uint8_t sensors_state) {
  if (this->write_register(QMI8658_REGISTER_CTRL7, &sensors_state, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Can't enable/disable sensors");
    return false;
  }
  return true;
}

bool QMI8658Component::enable_wake_on_motion(uint8_t threshold, QMI8658AccelODR accel_odr,
                                             QMI8658InterruptPin interrupt_pin, uint8_t initial_pin_state,
                                             uint8_t blanking_time) {
  if (!this->disable_sensors_()) {
    this->status_set_warning("Error enabling WoM: disabling sensors");
    return false;
  }

  if (!this->configure_accel_(accel_odr)) {
    this->status_set_warning("Error enabling WoM: configuring accel");
    return false;
  }

  ESP_LOGV(TAG, "  Configuring WoM threshold...");
  ESP_LOGV(TAG, "  threshold: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(threshold));
  if (this->write_register(QMI8658_REGISTER_CAL1_L, &threshold, 1) != i2c::ERROR_OK) {
    this->status_set_warning("Error enabling WoM: configuring threshold");
    return false;
  }

  ESP_LOGV(TAG, "  Configuring WoM interrupt pin...");
  uint8_t interrupt_pin_config = initial_pin_state << 7 | (uint8_t) interrupt_pin << 6 | (blanking_time & 0x3F);
  ESP_LOGV(TAG, "  interrupt_pin_config: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(interrupt_pin_config));
  if (this->write_register(QMI8658_REGISTER_CAL1_H, &interrupt_pin_config, 1) != i2c::ERROR_OK) {
    this->status_set_warning("Error enabling WoM: configuring interrupt pin");
    return false;
  }

  ESP_LOGV(TAG, "  Saving WoM settings...");
  if (!this->send_command_(QMI8658_CMD_WRITE_WOM_SETTING)) {
    this->status_set_warning("Error enabling WoM: saving WoM settings");
    return false;
  }

  if (!this->enable_sensors_(QMI8658_SENSOR_ACCEL)) {
    ESP_LOGE(TAG, "Can't enable accelerometer");
    return false;
  }

  if (!this->set_register_bit_(QMI8658_REGISTER_CTRL1, (uint8_t) interrupt_pin + 3)) {
    ESP_LOGE(TAG, "Can't enable interrupt");
    return false;
  }

  return true;
}

i2c::ErrorCode QMI8658Component::read_le_int16_(uint8_t reg, int16_t *value, uint8_t len) {
  uint8_t raw_data[len * 2];
  i2c::ErrorCode err = this->read_register(reg, raw_data, len * 2);
  if (err != i2c::ERROR_OK)
    return err;
  for (int i = 0; i < len; i++) {
    value[i] = (int16_t) ((uint16_t) raw_data[i * 2] | ((uint16_t) raw_data[i * 2 + 1] << 8));
  }
  return err;
}

bool QMI8658Component::send_command_(uint8_t command, uint32_t wait_ms) {
  ESP_LOGV(TAG, "  Sending command...");
  ESP_LOGV(TAG, "  command: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(command));
  if (this->write_register(QMI8658_REGISTER_CTRL9, &command, 1) != i2c::ERROR_OK) {
    this->status_set_warning("Error sending command");
    return false;
  }

  uint32_t start_millis = millis();
  uint8_t val;
  do {
    if (this->read_register(QMI8658_REGISTER_STATUS_INT, &val, 1) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "Error reading command status register");
      return false;
    }
    delay(1);
    if (millis() - start_millis > wait_ms) {
      ESP_LOGE(TAG, "Command timed out");
      return false;
    }
  } while (!(val & 0x80));
  return true;
}

bool QMI8658Component::set_register_bit_(uint8_t reg, uint8_t bit) {
  uint8_t value;
  if (this->read_register(reg, &value, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Error reading register");
    return false;
  }
  value |= (1 << bit);
  if (this->write_register(reg, &value, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Error writing register");
    return false;
  }
  return true;
}

void IRAM_ATTR QMI8658SensorStore::gpio_intr(QMI8658SensorStore *arg) {
  arg->data_ready = true;
  if (arg->component_ != nullptr) arg->component_->enable_loop_soon_any_context();
}

}  // namespace qmi8658
}  // namespace esphome
