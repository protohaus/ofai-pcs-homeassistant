#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ezo {
enum eSensorType{
	  SENSOR_TYPE_AUX = 0,
	  SENSOR_TYPE_COND = 1,
	  SENSOR_TYPE_PH = 2,
	  SENSOR_TYPE_TEMP = 3
  };
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
  bool calibration_mode_active(){return this->calibration_active_;};
  
  void enable_sensor(){this->sensor_enabled_ = true;};
  void disable_sensor(){this->sensor_enabled_ = false;};
  bool sensor_enabled(){return this->sensor_enabled_;};
  
  void set_sensor_type(eSensorType sensor_type){this->sensor_type_ = sensor_type;};
  
  void enable_sending_values(){this->sending_values_ = true;}
  void disable_sending_values(){this->sending_values_ = false;}
  bool sending_values(){return this->sending_values_;};
  
 protected:
  uint32_t start_time_ = 0;
  uint32_t wait_time_ = 0;
  uint16_t state_ = 0;
  float tempcomp_{25.0};
  
  uint32_t sample_interval_calibration_ = 2000; // sample_interval while calibration in [ms]
  uint32_t sample_interval_ = 10000; // sample_interval in [ms]
  
  uint32_t timestamp_0_ = 0;
  bool calibration_active_ = false;
  bool sensor_enabled_ = false;
  bool sending_values_ = true;

  eSensorType sensor_type_ = SENSOR_TYPE_TEMP;
  
};

}  // namespace ezo
}  // namespace esphome
