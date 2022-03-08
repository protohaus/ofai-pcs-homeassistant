#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ezo {

/// This class implements support for the EZO circuits in i2c mode
class CustomEZOSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  void loop() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_tempcomp_value(float temp);
  void set_sample_interval(uint32_t sample_interval){this->sample_interval_ = sample_interval;};
  void activate_calibration(){this->calibration_active_ = true;};
  void deactivate_calibration(){this->calibration_active_ = false;};
  
 protected:
  uint32_t start_time_ = 0;
  uint32_t wait_time_ = 0;
  uint16_t state_ = 0;
  float tempcomp_;
  uint32_t sample_interval_calibration_ = 1000; // sample_interval while calibration in [ms]
  uint32_t sample_interval_ = 10000; // sample_interval in [ms]
  
  uint32_t timestamp_0_ = 0;
  bool calibration_active_ = false;
};

}  // namespace ezo
}  // namespace esphome
