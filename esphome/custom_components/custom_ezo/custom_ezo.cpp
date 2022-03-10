#include "custom_ezo.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <string.h>

namespace esphome {
namespace ezo {

static const char *const TAG = "ezo.sensor";

static const uint16_t EZO_STATE_WAIT = 1; 	       					// 0000000000000001
static const uint16_t EZO_STATE_SEND_TEMP = 2; 	   					// 0000000000000010
static const uint16_t EZO_STATE_WAIT_TEMP = 4; 	   					// 0000000000000100
static const uint16_t EZO_STATE_SEND_PH_CAL_MID = 8; 	   			// 0000000000001000
static const uint16_t EZO_STATE_SEND_PH_CAL_LOW = 16;    			// 0000000000010000
static const uint16_t EZO_STATE_SEND_PH_CAL_HIGH = 32;    			// 0000000000100000
static const uint16_t EZO_STATE_SEND_CAL_CLEAR = 64;    			// 0000000001000000
static const uint16_t EZO_STATE_SEND_EC_CAL_DRY = 128;   			// 0000000010000000
static const uint16_t EZO_STATE_SEND_EC_CAL = 256;   				// 0000000100000000
static const uint16_t EZO_STATE_SEND_EC_CAL_LOW = 512;   			// 0000001000000000
static const uint16_t EZO_STATE_SEND_EC_CAL_HIGH = 1024;  			// 0000010000000000
static const uint16_t EZO_STATE_SEND_CAL_CHECK = 2048;  			// 0000100000000000
static const uint16_t EZO_STATE_SEND_EC_K = 4096;  					// 0001000000000000
static const uint16_t EZO_STATE_SEND_RTD_CAL = 8192;  				// 0010000000000000
static const uint16_t EZO_STATE_SEND_EC_K_VALUE_CHECK = 16384;      // 0100000000000000
static const uint16_t EZO_STATE_WAIT_CAL = 32768; 					// 1000000000000000

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
	{
       ESP_LOGE(TAG, "update overrun, still waiting for previous response");
	   if (sensor_enabled())
		retry_counter_ += 1;
    }
    return;
  }else
  {
    uint8_t c = 'R';
	this->write(&c, 1);
	this->state_ |= EZO_STATE_WAIT;
	this->start_time_ = time_now;
	this->wait_time_ = 900;
	retry_counter_ = 0;
  }
}

void CustomEZOSensor::loop() {
  uint8_t buf[21];
  uint32_t time_now = millis();
  
  if (!(this->state_ & EZO_STATE_WAIT)) 
  {
    if (this->state_ & EZO_STATE_SEND_TEMP) 
	{
	  if(this->sensor_enabled_  && this->sending_values_)
		ESP_LOGD(TAG, "Set temp compensation value: %0.3f", this->tempcomp_);
      int len = sprintf((char *) buf, "T,%0.3f", this->tempcomp_);
      this->write(buf, len);
      this->state_ = EZO_STATE_WAIT | EZO_STATE_WAIT_TEMP;
      this->start_time_ = millis();
      this->wait_time_ = 300;
    }else
	{  
       if (this->factory_reset_)
	   {
		 int len = 0;
		 if (this->sensor_type_ == SENSOR_TYPE_EC || 
		     this->sensor_type_ == SENSOR_TYPE_RTD)
		 {
		   ESP_LOGD(TAG, "Reset to factory settings");
		   len = sprintf((char *) buf, "FACTORY");
		 }else if (this->sensor_type_ == SENSOR_TYPE_PH)
		 {
		   ESP_LOGD(TAG, "Reset to factory settings");
		   len = sprintf((char *) buf, "X");
		 }
		 
	     this->write(buf, len);
	     this->state_ |= EZO_STATE_WAIT;
	     this->start_time_ = millis();
	     this->wait_time_ = 5000; 
		 this->factory_reset_ = false;
		 return;
	   }   
	   
	   if (this->state_ & EZO_STATE_SEND_CAL_CHECK)
	   {
		 ESP_LOGD(TAG, "Check calibration");
		 int len = sprintf((char *) buf, "CAL,?");
	     this->write(buf, len);
	     this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
	     this->start_time_ = millis();
	     this->wait_time_ = 300; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_CAL_CLEAR)
	   {
		 ESP_LOGD(TAG, "Clear calibration");
		 int len = sprintf((char *) buf, "CAL,CLEAR");
	     this->write(buf, len);
		 this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
	     this->start_time_ = millis();
	     this->wait_time_ = 300; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_PH_CAL_MID)
	   {
		 ESP_LOGD(TAG, "CAL,MID,%0.3f", this->ph_cal_mid_value_);  
	     int len = sprintf((char *) buf, "CAL,MID,%0.3f", this->ph_cal_mid_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 1300; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_PH_CAL_HIGH)
	   {
		 ESP_LOGD(TAG, "CAL,HIGH,%0.3f", this->ph_cal_high_value_);  
	     int len = sprintf((char *) buf, "CAL,HIGH,%0.3f", this->ph_cal_high_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 1300; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_PH_CAL_LOW)
	   {
		 ESP_LOGD(TAG, "CAL,LOW,%0.3f", this->ph_cal_low_value_);  
	     int len = sprintf((char *) buf, "CAL,LOW,%0.3f", this->ph_cal_low_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 1300; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_EC_CAL_DRY)
	   {
		 ESP_LOGD(TAG, "CAL,DRY");  
	     int len = sprintf((char *) buf, "CAL,DRY");
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 600; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_EC_CAL)
	   {
		 ESP_LOGD(TAG, "CAL,%0.3f", this->ec_cal_value_);  
	     int len = sprintf((char *) buf, "CAL,%0.3f", this->ec_cal_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 600; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_EC_CAL_HIGH)
	   {
		 ESP_LOGD(TAG, "CAL,HIGH,%0.3f", this->ec_cal_high_value_);  
	     int len = sprintf((char *) buf, "CAL,HIGH,%0.3f", this->ec_cal_high_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 600; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_EC_CAL_LOW)
	   {
		 ESP_LOGD(TAG, "CAL,LOW,%0.3f", this->ec_cal_low_value_);  
	     int len = sprintf((char *) buf, "CAL,LOW,%0.3f", this->ec_cal_low_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 600; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_EC_K)
	   {
		 ESP_LOGD(TAG, "K,%0.3f", this->ec_k_value_);  
	     int len = sprintf((char *) buf, "K,%0.3f", this->ec_k_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 300; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_EC_K_VALUE_CHECK)
	   {
		 ESP_LOGD(TAG, "K,?");  
	     int len = sprintf((char *) buf, "K,?");
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 600; 
	   }
	   
	   if (this->state_ & EZO_STATE_SEND_RTD_CAL)
	   {
		 ESP_LOGD(TAG, "CAL,%0.3f", this->rtd_cal_value_);  
	     int len = sprintf((char *) buf, "CAL,%0.3f", this->rtd_cal_value_);
         this->write(buf, len);
         this->state_ |= EZO_STATE_WAIT;
	     this->state_ |= EZO_STATE_WAIT_CAL;
         this->start_time_ = millis();
         this->wait_time_ = 600; 
	   }
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
	  retry_counter_ = 0;
	  break;
	case 2:
	  if (this->sensor_enabled_ && this->sending_values_)
	  {
		ESP_LOGE(TAG, "device returned a syntax error");
		retry_counter_ += 1;
	  }
	  break;
	case 254:
	  return;  // keep waiting
	case 255:
	  if (this->sensor_enabled_ && this->sending_values_)
	  {
		ESP_LOGE(TAG, "device returned no data");
		retry_counter_ += 1;
	  }
	  break;
	default:
	  if (this->sensor_enabled_  && this->sending_values_)
	  {
		ESP_LOGE(TAG, "device returned an unknown response: %d", buf[0]);
		retry_counter_ += 1;
	  }
	  break;
  }
  
  if (this->state_ & EZO_STATE_WAIT_TEMP) 
  {
	if (buf[0] != 1)
		ESP_LOGE(TAG, "Temperature compensation failed (unknown error)");
    this->state_ = 0;
	this->calibration_triggered_ = false;
    return;
  }
  
  if (this->state_ & EZO_STATE_WAIT_CAL) 
  {
	if (this->state_ & EZO_STATE_SEND_CAL_CHECK)
	{
		std::string s( buf, buf+20 );
		ESP_LOGD(TAG, "Calibration-state: %s", s.c_str());
	}
	else if (this->state_ & EZO_STATE_SEND_CAL_CLEAR)
	{
	  if (buf[0] == 1)
	  	ESP_LOGD(TAG, "Clear Calibration: successful");
	  else
		ESP_LOGE(TAG, "Clear Calibration: failed");
	}
	else if (this->state_ & EZO_STATE_SEND_EC_K_VALUE_CHECK)
	{
		std::string s( buf, buf+20 );
		ESP_LOGD(TAG, "EC k-value: %s", s.c_str());
	} 
	else
	{
	  if (buf[0] == 1)
	  	ESP_LOGD(TAG, "Action: successful");
	  else
		ESP_LOGE(TAG, "Action: failed");
	}
  
	this->calibration_triggered_ = false;
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

/******************************************************************************
** TEMPCOMP Method
*******************************************************************************/
void CustomEZOSensor::set_tempcomp_value(float temp) {
  if(this->calibration_active_)
   {
  	 return;
   }
   
  if(this->calibration_triggered_)
  {
 	ESP_LOGE(TAG, "ERROR: calibration pending, skipping tempcomp action");
 	return;
  }
  this->tempcomp_ = temp;
  this->state_ |= EZO_STATE_SEND_TEMP;
}


/******************************************************************************
** general calibration methods
*******************************************************************************/
void CustomEZOSensor::start_calibration_check()
{
	if(!check_calibration_condition())
		return;
	
	ESP_LOGD(TAG, "Trigger Calibration check");
	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_CAL_CHECK;
		
}

void CustomEZOSensor::start_calibration_clear()
{
	if(!check_calibration_condition())
		return;
	
	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_CAL_CLEAR;
};

bool CustomEZOSensor::check_calibration_condition()
{	
	if(!this->calibration_active_)
	{
		ESP_LOGE(TAG, "ERROR: activate calibration mode first");
		return false;
	}
	
	if(this->calibration_triggered_)
	{
		ESP_LOGE(TAG, "ERROR: calibration already pending, try again after 1.3s");
		return false;
	}
	
	if (this->state_ & EZO_STATE_SEND_TEMP)
	{
		ESP_LOGE(TAG, "ERROR: temperature compensation pending, try again");
		return false;
	}
	
	return true;
}

/******************************************************************************
** PH calibration methods
*******************************************************************************/
void CustomEZOSensor::start_calibration_ph_high()
{
	if (this->sensor_type_ != SENSOR_TYPE_PH)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;
	
	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_PH_CAL_HIGH;
};
  
  
void CustomEZOSensor::start_calibration_ph_low()
{
	if (this->sensor_type_ != SENSOR_TYPE_PH)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;
	
	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_PH_CAL_LOW;
};


void CustomEZOSensor::start_calibration_ph_mid()
{
	if (this->sensor_type_ != SENSOR_TYPE_PH)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;
	
	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_PH_CAL_MID;
};



/******************************************************************************
** EC calibration methods
*******************************************************************************/
void CustomEZOSensor::start_calibration_ec_dry()
{
	if (this->sensor_type_ != SENSOR_TYPE_EC)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;

	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_EC_CAL_DRY;

}

void CustomEZOSensor::start_calibration_ec()
{
	if (this->sensor_type_ != SENSOR_TYPE_EC)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;

	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_EC_CAL;
}

void CustomEZOSensor::start_calibration_ec_high()
{
	if (this->sensor_type_ != SENSOR_TYPE_EC)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;

	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_EC_CAL_HIGH;
}

void CustomEZOSensor::start_calibration_ec_low()
{
	if (this->sensor_type_ != SENSOR_TYPE_EC)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;

	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_EC_CAL_LOW;
}

void CustomEZOSensor::start_send_ec_k_value()
{
	if (this->sensor_type_ != SENSOR_TYPE_EC)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;

	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_EC_K;
}

void CustomEZOSensor::start_check_ec_k_value()
{
	if (this->sensor_type_ != SENSOR_TYPE_EC)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;

	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_EC_K_VALUE_CHECK;
}	
	
/******************************************************************************
** RTD calibration methods
*******************************************************************************/	
void CustomEZOSensor::start_calibration_rtd()
{
	if (this->sensor_type_ != SENSOR_TYPE_RTD)
	{
		ESP_LOGE(TAG, "ERROR: wrong sensor type!");
		return;  
	}
	
	if(!check_calibration_condition())
		return;

	this->calibration_triggered_ = true;
	this->state_ |= EZO_STATE_SEND_RTD_CAL;
}	

/******************************************************************************
** RTD calibration methods
*******************************************************************************/	
void CustomEZOSensor::start_factory_reset(void)
{
	if(this->calibration_triggered_)
	{
		ESP_LOGE(TAG, "ERROR: calibration already pending, try again after 1.3s");
		return;
	}
	
	if (this->state_ & EZO_STATE_SEND_TEMP)
	{
		ESP_LOGE(TAG, "ERROR: temperature compensation pending, try again");
		return;
	}
	
	ESP_LOGD(TAG, "Issuing Factory reset");
	this->factory_reset_ = true;
}
	
}  // namespace ezo
}  // namespace esphome
