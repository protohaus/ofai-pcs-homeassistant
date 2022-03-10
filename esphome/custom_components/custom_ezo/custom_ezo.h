#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"


namespace esphome {
namespace ezo {
enum eSensorType{
	  SENSOR_TYPE_AUX = 0,
	  SENSOR_TYPE_EC = 1,
	  SENSOR_TYPE_PH = 2,
	  SENSOR_TYPE_RTD = 3
  };
/// This class implements support for the EZO circuits in i2c mode
class CustomEZOSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  
  void loop() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_tempcomp_value(float temp);
  void set_sample_interval(uint32_t sample_interval)
  {
	  this->sample_interval_ = sample_interval;
  };
  
  void activate_calibration(){this->calibration_active_ = true;};
  void deactivate_calibration(){this->calibration_active_ = false;};
  bool calibration_mode_active(){return this->calibration_active_;};
  
  void enable_sensor(){this->sensor_enabled_ = true;};
  void disable_sensor()
  {
    this->sensor_enabled_ = false;
	this->retry_counter_ = 0;
	this->calibration_triggered_ = false, 
	this->state_ = 0;
  };
  bool sensor_enabled(){return this->sensor_enabled_;};
  
  void set_sensor_type(eSensorType sensor_type){this->sensor_type_ = sensor_type;};
  
  void enable_sending_values(){this->sending_values_ = true;}
  void disable_sending_values(){this->sending_values_ = false;}
  bool sending_values(){return this->sending_values_;};
  
  bool check_calibration_condition(void);
  
  bool calibration_triggered(){return this->calibration_triggered_;};
  void start_calibration_clear(void);
  void start_calibration_check(void);
  
  void start_calibration_ph_mid(void);
  void start_calibration_ph_low(void);
  void start_calibration_ph_high(void);
  
  void start_calibration_ec_dry(void);
  void start_calibration_ec(void);
  void start_calibration_ec_high(void);
  void start_calibration_ec_low(void);
  void start_send_ec_k_value(void);
  void start_check_ec_k_value(void);
  
  void start_calibration_rtd(void);
  
  void start_factory_reset(void);
  
  void set_ph_cal_mid_value(float ph_cal_mid_value){this->ph_cal_mid_value_ = ph_cal_mid_value;};
  void set_ph_cal_high_value(float ph_cal_high_value){this->ph_cal_high_value_ = ph_cal_high_value;};
  void set_ph_cal_low_value(float ph_cal_low_value){this->ph_cal_low_value_ = ph_cal_low_value;};
  
  void set_ec_cal_value(float ec_cal_value){this->ec_cal_value_ = ec_cal_value;};
  void set_ec_cal_high_value(float ec_cal_high_value){this->ec_cal_high_value_ = ec_cal_high_value;};
  void set_ec_cal_low_value(float ec_cal_low_value){this->ec_cal_low_value_ = ec_cal_low_value;};
  void set_ec_k_value(float ec_k_value){this->ec_k_value_ = ec_k_value;};
  
  void set_rtd_cal_value(float rtd_cal_value){this->rtd_cal_value_ = rtd_cal_value;};
  
  int retry_counter(void){return this->retry_counter_;};
  
 protected:
  uint32_t start_time_ = 0;
  uint32_t wait_time_ = 0;
  
  uint16_t state_ = 0;
  float tempcomp_{25.0};
  
  float ph_cal_mid_value_ = 7.0;
  float ph_cal_high_value_ = 10.0;
  float ph_cal_low_value_ = 4.0;
  
  float ec_cal_value_ = 80000.0;
  float ec_cal_high_value_ = 80000.0;
  float ec_cal_low_value_ = 12880.0;
  float ec_k_value_ = 1.0;
  float rtd_cal_value_ = 100.0;
  
  uint16_t retry_counter_ = 0;
  
  uint32_t sample_interval_calibration_ = 2000; // sample_interval while calibration in [ms]
  uint32_t sample_interval_ = 10000; // sample_interval in [ms]
  
  uint32_t timestamp_0_ = 0;
  bool calibration_active_ = false;
  bool calibration_triggered_ = false;
  
  bool sensor_enabled_ = false;
  bool sending_values_ = true;
  
  bool factory_reset_ = false;
  
  eSensorType sensor_type_ = SENSOR_TYPE_RTD;
  
};

}  // namespace ezo
}  // namespace esphome
