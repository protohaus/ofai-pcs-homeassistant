#include "custom_ezo.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ezo {

static const char *const TAG = "ezo.sensor";

static const uint16_t EZO_STATE_WAIT = 1; 	       					// 0000000000000001
static const uint16_t EZO_STATE_SEND_TEMP = 2; 	   					// 0000000000000010
static const uint16_t EZO_STATE_WAIT_TEMP = 4; 	   					// 0000000000000100
static const uint16_t EZO_STATE_SEND_PH_CAL_MID = 8; 	   			// 0000000000001000
static const uint16_t EZO_STATE_SEND_PH_CAL_LOW = 16;    			// 0000000000010000
static const uint16_t EZO_STATE_SEND_PH_CAL_HIGH = 32;    			// 0000000000100000
static const uint16_t EZO_STATE_SEND_PH_CAL_CLEAR = 64;    			// 0000000001000000
static const uint16_t EZO_STATE_SEND_EC_CAL_DRY = 128;   			// 0000000010000000
static const uint16_t EZO_STATE_SEND_EC_CAL = 256;   				// 0000000100000000
static const uint16_t EZO_STATE_SEND_EC_CAL_LOW = 512;   			// 0000001000000000
static const uint16_t EZO_STATE_SEND_EC_CAL_HIGH = 1024;  			// 0000010000000000
static const uint16_t EZO_STATE_SEND_EC_CAL_CLEAR = 2048;  			// 0000100000000000
static const uint16_t EZO_STATE_SEND_EC_K = 4096;  					// 0001000000000000
static const uint16_t EZO_STATE_SEND_RTD_CAL = 8192;  				// 0010000000000000
static const uint16_t EZO_STATE_SEND_RTD_CAL_CLEAR = 8192;  		// 0100000000000000
static const uint16_t EZO_STATE_WAIT_CAL = 16384; 					// 1000000000000000

void CustomEZOSensor::dump_config() {
  LOG_SENSOR("", "EZO", this);
  LOG_I2C_DEVICE(this);
  if (this->is_failed())
    ESP_LOGE(TAG, "Communication with EZO circuit failed!");
  LOG_UPDATE_INTERVAL(this);
}

void CustomEZOSensor::update() {
  uint32_t sample_interval = 0;
  uint32_t time_now = millis();

  if (!calibration_active_)
    sample_interval = this->sample_interval_;
  else
	sample_interval = this->sample_interval_calibration_;

  // integer overflow protection -> restart waiting timer -> (TODO: maybe implement exact value reset via max integer)
  if (time_now < this->timestamp_0_ )
  	this->timestamp_0_ = time_now;
  
  // check if we should send a sample
  if ((int)(time_now - this->timestamp_0_) < sample_interval)
    return;
  else
    this->timestamp_0_ = time_now;

  if (this->state_ & EZO_STATE_WAIT) 
  {
	if(this->sensor_enabled_  && this->sending_values_)
       ESP_LOGE(TAG, "update overrun, still waiting for previous response");
    return;
  }else
  {
    uint8_t c = 'R';
	this->write(&c, 1);
	this->state_ |= EZO_STATE_WAIT;
	this->start_time_ = time_now;
	this->wait_time_ = 900;
  }
}

void CustomEZOSensor::loop() {
  uint8_t buf[21];
  uint32_t time_now = millis();
  
  if (!(this->state_ & EZO_STATE_WAIT)) 
  {
    if (this->state_ & EZO_STATE_SEND_TEMP) 
	{
      int len = sprintf((char *) buf, "T,%0.3f", this->tempcomp_);
      this->write(buf, len);
      this->state_ = EZO_STATE_WAIT | EZO_STATE_WAIT_TEMP;
      this->start_time_ = millis();
      this->wait_time_ = 300;
    }
    return;
  }
  // integer overflow protection -> restart waiting timer
  if (time_now < this->start_time_ )
	this->start_time_ = time_now;  

  if ((int)(time_now - this->start_time_) < this->wait_time_)
    return;
  
  buf[0] = 0;
  if (!this->read_bytes_raw(buf, 20)) 
  {
    this->state_ = 0;
	if (this->sensor_enabled_  && this->sending_values_)
	  ESP_LOGE(TAG, "read error");
	return;
  }

  switch (buf[0]) {
	case 1:
	  break;
	case 2:
	  if (this->sensor_enabled_ && this->sending_values_)
		ESP_LOGE(TAG, "device returned a syntax error");
	  break;
	case 254:
	  return;  // keep waiting
	case 255:
	  if (this->sensor_enabled_ && this->sending_values_)
		ESP_LOGE(TAG, "device returned no data");
	  break;
	default:
	  if (this->sensor_enabled_  && this->sending_values_)
		ESP_LOGE(TAG, "device returned an unknown response: %d", buf[0]);
	  break;
  }
  
  if (this->state_ & EZO_STATE_WAIT_TEMP) 
  {
    this->state_ = 0;
    return;
  }
  
  this->state_ &= ~EZO_STATE_WAIT;
  
  if (buf[0] != 1)
    return;

  // some sensors return multiple comma-separated values, terminate string after first one
  for (size_t i = 1; i < sizeof(buf) - 1; i++)
    if (buf[i] == ',')
      buf[i] = '\0';

  float val = parse_number<float>((char *) &buf[1]).value_or(0);
  if (this->sensor_enabled_ && this->sending_values_)
	this->publish_state(val);
}

void CustomEZOSensor::set_tempcomp_value(float temp) {
  this->tempcomp_ = temp;
  this->state_ |= EZO_STATE_SEND_TEMP;
}

}  // namespace ezo
}  // namespace esphome
