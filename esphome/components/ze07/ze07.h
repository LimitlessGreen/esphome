#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ze07 {

class ZE07COComponent : public Component, public uart::UARTDevice {
 public:
  ZE07COComponent() = default;

  /// Manually set the rx-only mode. Defaults to false.
  void set_rx_mode_only(bool rx_mode_only);

  void set_co_sensor(sensor::Sensor *co_sensor) { co_sensor_ = co_sensor; }

  void setup() override;
  void dump_config() override;
  void loop() override;

  float get_setup_priority() const override;

  void set_update_interval(uint32_t val) { /* ignore */
  }
  void set_update_interval_min(uint8_t update_interval_min);
  void set_working_state(bool working_state);

 protected:
  void ze07_write_command_(uint8_t command);
  uint8_t ze07_checksum_(const uint8_t *i, uint8_t ln) const;
  optional<bool> check_byte_() const;
  void parse_data_();
  float get_value_(uint8_t lbyte, uint8_t hbyte) const;

  sensor::Sensor *co_sensor_{nullptr};

  uint8_t data_[10];
  uint8_t data_index_{0};
  uint32_t last_transmission_{0};
  uint8_t update_interval_min_;

  bool rx_mode_only_;
};

}  // namespace ze07
}  // namespace esphome
