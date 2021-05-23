#include "ze07.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ze07 {

static const char *TAG = "ze07";

static const uint8_t ZE07_MSG_LENGTH = 9;
static const uint8_t ZE07_MSG_HEAD = 0xff;
static const uint8_t ZE07_MSG_HBYTE = 0x00;
static const uint8_t ZE07_MSG_LBYTE = 0xff;
static const uint8_t ZE07_GAS_TYPE_CO = 0x04;
static const uint8_t ZE07_UNIT_PPM = 0x03;
static const uint8_t ZE07_COMMAND_ANSWER = 0x86;
static const uint8_t ZE07_COMMAND_SWITCH = 0x78;
static const uint8_t ZE07_COMMAND_MODE_QA = 0x41;
static const uint8_t ZE07_COMMAND_MODE_INI_UPLOAD = 0x40;
static const uint8_t ZE07_RESERVE = 0x00;
static const uint8_t ZE07_SEND = 0x01;

void ZE07COComponent::setup() {
  if (this->rx_mode_only_) {
    // In RX-only mode we do not setup the sensor, it is assumed to be setup
    // already
    return;
  } else {
    this->ze07_write_command_(ZE07_COMMAND_MODE_QA);
  }
}

void ZE07COComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ZE-07:");
  ESP_LOGCONFIG(TAG, "  Update Interval: %u min", this->update_interval_min_);
  ESP_LOGCONFIG(TAG, "  RX-only mode: %s", ONOFF(this->rx_mode_only_));
  LOG_SENSOR("  ", "CO", this->co_sensor_);
  this->check_uart_settings(9600);
}

void ZE07COComponent::loop() {
  const uint32_t now = millis();
  if ((now - this->last_transmission_ >= 500) && this->data_index_) {
    // last transmission too long ago. Reset RX index.
    ESP_LOGV(TAG, "Last transmission too long ago. Reset RX index.");
    this->data_index_ = 0;
  }

  if (this->available() == 0) {
    return;
  }

  this->last_transmission_ = now;
  while (this->available() != 0) {
    this->read_byte(&this->data_[this->data_index_]);
    auto check = this->check_byte_();
    if (!check.has_value()) {
      // finished
      this->parse_data_();
      this->data_index_ = 0;
    } else if (!*check) {
      // wrong data
      ESP_LOGV(TAG, "Byte %i of received data frame is invalid.", this->data_index_);
      this->data_index_ = 0;
    } else {
      // next byte
      this->data_index_++;
    }
  }
}

float ZE07COComponent::get_setup_priority() const { return setup_priority::DATA; }

void ZE07COComponent::set_rx_mode_only(bool rx_mode_only) { this->rx_mode_only_ = rx_mode_only; }

void ZE07COComponent::ze07_write_command_(uint8_t command) {
  /* possible commands :
    - ZE07_COMMAND_MODE_INI_UPLOAD
    - ZE07_COMMAND_MODE_QA (QA = Question/Answer)
    - ZE07_COMMAND_ANSWER
  */

  uint8_t command_data[ZE07_MSG_LENGTH] = {0};
  command_data[0] = ZE07_MSG_HEAD;
  command_data[1] = ZE07_SEND;

  switch (command) {
    case ZE07_COMMAND_ANSWER:
      command_data[2] = ZE07_COMMAND_ANSWER;
      break;

    default: {
      command_data[2] = ZE07_COMMAND_SWITCH;
      switch (command) {
        case ZE07_COMMAND_MODE_INI_UPLOAD:
          command_data[3] = ZE07_COMMAND_MODE_INI_UPLOAD;
          break;

        case ZE07_COMMAND_MODE_QA:
          command_data[3] = ZE07_COMMAND_MODE_QA;
          break;

        default:
          command_data[3] = 0x00;
          break;
      }
    } break;
  }

  command_data[ZE07_MSG_LENGTH - 1] = this->ze07_checksum_(command_data, ZE07_MSG_LENGTH);  // last one checksum

  this->write_array(command_data, ZE07_MSG_LENGTH);
}

uint8_t ZE07COComponent::ze07_checksum_(const uint8_t *i, uint8_t ln) const {
  // copied from documentation
  // https://www.winsen-sensor.com/d/files/PDF/Gas%20Sensor%20Module/CO%20Detection%20Module/ZE07%20CO%20Module%201.3V.pdf
  uint8_t j, tempq = 0;
  i += 1;
  for (j = 0; j < (ln - 2); j++) {
    tempq += *i;
    i++;
  }
  tempq = (~tempq) + 1;
  return (tempq);
}

optional<bool> ZE07COComponent::check_byte_() const {
  uint8_t index = this->data_index_;
  uint8_t byte = this->data_[index];

  if (index == 0) {
    return byte == ZE07_MSG_HEAD;
  }

  if (index == 1) {
    // in rx_only mode this byte contains the type of sensor
    // maybe some changes are needed here if more Sensors from winsen-sensor get adobted
    // in this case ZE07_GAS_TYPE_CO == 0x04 == The carbon monoxide sensor from winsen-sensor
    return byte == (this->rx_mode_only_) ? ZE07_GAS_TYPE_CO : ZE07_COMMAND_ANSWER;
  }

  if ((index >= 2) && (index <= 6)) {
    return true;
  }

  if (index == 8) {
    // checksum is without checksum bytes
    uint8_t checksum = ze07_checksum_(this->data_, ZE07_MSG_LENGTH);
    if (checksum != byte) {
      ESP_LOGW(TAG, "ZE07-CO Checksum doesn't match: 0x%02X!=0x%02X", byte, checksum);
      return false;
    }
    return true;
  }

  return {};
}

void ZE07COComponent::parse_data_() {
  this->status_clear_warning();
  float co_concentration;

  switch (this->data_[1]) {
    case ZE07_COMMAND_ANSWER:
      co_concentration = get_value_(this->data_[5], this->data_[4]);
      break;

    default:
      co_concentration = get_value_(this->data_[3], this->data_[2]);
      break;
  }

  ESP_LOGD(TAG, "Got CO Concentration: %.1f ppm", co_concentration);

  if (co_concentration <= 0) {
    // not yet any valid data
    return;
  }

  if (this->co_sensor_ != nullptr) {
    this->co_sensor_->publish_state(co_concentration);
  }
}

float ZE07COComponent::get_value_(uint8_t lbyte, uint8_t hbyte) const { return (hbyte * 256 + lbyte) * 0.1f; }

void ZE07COComponent::set_update_interval_min(uint8_t update_interval_min) {
  this->update_interval_min_ = update_interval_min;
}

}  // namespace ze07
}  // namespace esphome
