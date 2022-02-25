#pragma once

#include <esphome.h>
#include <stepper_modes.h>
#include <stepper_error.h>
#include <algorithm>

#define get_controller(constructor) static_cast<cStepperController *>(const_cast<esphome::custom_component::CustomComponentConstructor *>(&constructor)->get_component(0))

#define SECONDS_TO_MILLISECONDS 1000
#define SECONDS_TO_MICROSECONDS 1000000
#define NUMBER_OF_RANGEESTIMATIONS 5

// kinematics
#define KINEMATICS_PLANT_GEAR_TEETH 12
#define KINEMATICS_PINION_WHEEL_RODS_UPPER 40
#define KINEMATICS_PINION_WHEEL_RODS_LOWER 64
#define KINEMATICS_TRANSMISSION_GEAR_BIG_TEETH 15
#define KINEMATICS_TRANSMISSION_GEAR_SMALL_TEETH 140
#define KINEMATICS_STEPS_PER_ROTATION 200
#define KINEMATICS_MICROSTEPS_PER_STEP 64
#define KINEMATICS_MOTOR_GEAR_TEETH 10
#define KINEMATICS_FULL_TURN_STEPS 764586

#define KINEMATICS_PLANT_GEAR_SLOTS 6
#define KINEMATICS_PLANT_GEAR_DEGREES_PER_SLOT 200


#define PINION_WHEEL_ERROR 3
#define HOMING_DIFF_ERROR 500
#define STEPPER_ERROR_FULL_TURN_STEPS 0.001

// mutex definitions
static SemaphoreHandle_t mutex_main_loop = NULL;
static SemaphoreHandle_t mutex_pinion_wheels = NULL;
//static SemaphoreHandle_t mutex_homing;
//static SemaphoreHandle_t mutex_rangeestimation;
//static SemaphoreHandle_t mutex_drive;
//static SemaphoreHandle_t mutex_manual;



static const char *TAG = "custom.StepperController";

class cStepperController : public PollingComponent, public CustomAPIDevice {
	public:
	    
		float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }
		
		void dump_config()
		{
			ESP_LOGCONFIG(TAG, "Stepper Controller");
		}
	    
		/********************************************************************************
	    ** cCustomController Constructor
		**  loads all settings and other external objects like stepper, sensors, buttons
		********************************************************************************/
		cStepperController
		(
		  int initial_speed_,
		  int max_speed_,
		  int acceleration_,
		  int max_acceleration_,
		  int deceleration_,
		  int max_deceleration_,
		  int step_width_,
		  std::string device_name_,
		  std::string stepper_id_,
		  std:: string controller_id_,
		  a4988::A4988 *stepper_,
		  gpio::GPIOBinarySensor *proximity_switch_home_,
		  gpio::GPIOBinarySensor *proximity_switch_global_home_,
		  gpio::GPIOBinarySensor *proximity_switch_pinion_wheel_,
		  template_::TemplateSensor *sensor_stepper_mode_,
		  int update_interval_ms_,
		  int sensor_update_interval_slow_,
		  int sensor_update_interval_mid_,
		  int sensor_update_interval_fast_,
		  int sensor_update_interval_realtime_,
		  int homing_high_speed_,
		  int homing_mid_speed_,
		  int homing_low_speed_
		)
		:
		  PollingComponent(update_interval_ms_),
		  CustomAPIDevice(),
		  m_max_speed(max_speed_),
		  m_max_acceleration(max_acceleration_),
		  m_max_deceleration(max_deceleration_),
		  m_device_name(device_name_),
		  m_stepper_id(stepper_id_),
		  m_controller_id(controller_id_),
		  m_stepper(stepper_),
		  m_proximity_switch_home(proximity_switch_home_),
		  m_proximity_switch_global_home(proximity_switch_global_home_),
		  m_proximity_switch_pinion_wheel(proximity_switch_pinion_wheel_),
		  m_sensor_stepper_mode(sensor_stepper_mode_),
		  m_update_interval_ms(update_interval_ms_),
		  m_sensor_update_interval_slow(sensor_update_interval_slow_),
		  m_sensor_update_interval_mid(sensor_update_interval_mid_),
		  m_sensor_update_interval_fast(sensor_update_interval_fast_),
		  m_sensor_update_interval_realtime(sensor_update_interval_realtime_),
		  m_homing(tHoming(	homing_high_speed_, 
							homing_mid_speed_, 
							homing_low_speed_))
		{	
			// initialize stepper parameters with set functions to check wromg input
			speed(initial_speed_);
			requested_speed(initial_speed_);
			acceleration(acceleration_);
			acceleration_requested(acceleration_);
			deceleration(deceleration_);
			deceleration_requested(deceleration_);
			step_width(step_width_);
			
			// initialize permanent global variable
			// !!! attention: duplicated hashes may cause problems 
			// !!! watch out: esphome autopgenerates hashes itself (check main.cpp)
			m_measured_full_turn_steps = new globals::RestoringGlobalsComponent<int>(KINEMATICS_FULL_TURN_STEPS);
			m_measured_full_turn_steps->set_component_source("globals");
			m_measured_full_turn_steps->set_name_hash(659212362);
			App.register_component(m_measured_full_turn_steps);
			
			m_number_of_cycles = new globals::RestoringGlobalsComponent<int>(0);
			m_number_of_cycles->set_component_source("globals");
			m_number_of_cycles->set_name_hash(79172764);
			App.register_component(m_number_of_cycles);
			
			
			m_sensor_check_position = new template_::TemplateBinarySensor();
			m_sensor_check_position->set_component_source("template.binary_sensor");
			App.register_component(m_sensor_check_position);
			App.register_binary_sensor(m_sensor_check_position);
			m_sensor_check_position->set_name(m_device_name + "." + m_stepper_id + "." + m_controller_id + "." + "check_and_reset_position");
			m_sensor_check_position->set_disabled_by_default(false);
			m_sensor_check_position->set_internal(true);
			
			m_sensor_check_position->set_template([=]() -> optional<bool> 
			{
				if(static_cast<int>(stepper_mode()) == STEPPER_MODE_OFF ||
				   static_cast<int>(stepper_mode()) == STEPPER_MODE_READY ||
				   static_cast<int>(stepper_mode()) == STEPPER_MODE_DRIVE ||
				   static_cast<int>(stepper_mode()) == STEPPER_MODE_MANUAL || 
				  (static_cast<int>(stepper_mode()) == STEPPER_MODE_HOMING && !m_calibration_mode_active))
				{
					return check_and_reset_position();
				}else
				{
					return true;
				}
			});
				
			}
		
		/********************************************************************************
	    ** Overloading Setup method of Base class
		** register services for Home Assistant integration
		********************************************************************************/
		void setup() override 
		{
			// Declare a service "stepper_set_requested_target_position"
			//  - Service will be called "esphome.<NODE_NAME>_<m_stepper_id>_<m_controller_id>_set_requested_target_position" in Home Assistant.
			//  - The service has one argument (type inferred from method definition):
			//     - requested_target_position: integer
			//  - The function on_set_requested_target_position declared below will attached to the service.
			register_service(&cStepperController::on_set_requested_target_position, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_requested_target_position",
						{"requested_target_position"});		
						
			register_service(&cStepperController::on_set_deceleration, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_deceleration",
						{"deceleration"});
			
			register_service(&cStepperController::on_set_acceleration, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_acceleration",
						{"acceleration"});
			
			register_service(&cStepperController::on_set_speed, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_speed",
						{"speed"});
						
			register_service(&cStepperController::on_set_speed_turns_per_hour, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_speed_turns_per_hour",
						{"turns_per_hour"});			
			
			register_service(&cStepperController::on_set_step_width, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_step_width",
						{"step_width"});

			register_service(&cStepperController::on_set_zero_position, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_zero_position",
						{"zero_position"});
		    
			mutex_main_loop = xSemaphoreCreateMutex();			
			mutex_pinion_wheels = xSemaphoreCreateMutex();			
			//mutex_homing = xSemaphoreCreateMutex();		
			//mutex_rangeestimation = xSemaphoreCreateMutex();		
			//mutex_drive = xSemaphoreCreateMutex();		
			//mutex_manual = xSemaphoreCreateMutex();		
			
        };	


		/********************************************************************************
	    ** Overloading Update method of PollingComponent class
		** called every m_update_interval_ms milliseconds
		********************************************************************************/
		void update() override 
		{
          // This will be called every "update_interval" milliseconds.
		  main_loop();
        }
		
		/*************************************************************************
		*************** GETTER, SETTER AND SERVICE METHOD DEFINITIONS ************
		**************************************************************************/
		
		/***************************************
	    ** direction_forward (get/set - methods)
		***************************************/
		void direction_forward(bool  direction_forward_)
		{
			m_direction_forward = direction_forward_;
		}
		
		bool direction_forward(){return m_direction_forward;}
		
		int direction_sign(){return (direction_forward() == true ? 1 : -1);}
		
		/***************************************
	    ** full_turn_steps (get/set/service - methods)
		***************************************/
		int measured_full_turn_steps(){return m_measured_full_turn_steps->value();}

		/***************************************
	    ** target_position (get/set - methods)
		***************************************/
		void target_position(int  target_position_)
		{
			m_target_position = target_position_;
		}
		
		int target_position(){return m_stepper->target_position;}
		
		
		/***************************************
	    ** speed (get/set - methods)
		***************************************/
		void speed(int  speed_)
		{
			int loc_speed = (speed_ > m_max_speed) ?  m_max_speed : speed_;
			m_speed = loc_speed;
			m_stepper->set_max_speed(loc_speed);
			m_stepper->on_update_speed();
		}
		
		int speed(){return m_speed;}
		
		void on_set_speed(int speed_)
		{
			requested_speed(speed_);
			speed(speed_);
		}
		
		void on_set_speed_turns_per_hour(float turns_per_hour_)
		{
			float angular_velocity = turns_per_hour_ / (60 * 60 / 360.0);
			float speed_ = angular_velocity / (360.0 / KINEMATICS_FULL_TURN_STEPS);
			requested_speed(speed_);
			speed(speed_);
		}
		
		void set_speed_zero()
		{
			speed(0);
		}
		
		void set_speed_requested()
		{
			speed(m_requested_speed);
		}
		
		void set_speed_high()
		{
			speed(m_homing.high_speed);
		}
		
		void set_speed_mid()
		{
			speed(m_homing.mid_speed);
		}
		
		void set_speed_low()
		{
			speed(m_homing.low_speed);
		}
		
		/***************************************
	    ** requested_speed (get/set - methods)
		***************************************/
		void requested_speed(int  requested_speed_)
		{
			m_requested_speed = requested_speed_;
		}
		
		int requested_speed(){return m_requested_speed;}
		
		/***************************************
	    ** acceleration (get/set - methods)
		***************************************/
		void acceleration(int  acceleration_)
		{
			int loc_acceleration = (acceleration_ > m_max_acceleration) ?  m_max_acceleration : acceleration_;
			m_acceleration = loc_acceleration;
			m_stepper->set_acceleration(loc_acceleration);
		}
		
		int acceleration(){return m_acceleration;}
		
		void set_acceleration_max()
		{
			acceleration(m_max_acceleration);
		}
		
		void set_acceleration_requested()
		{
			acceleration(m_acceleration_requested);
		}
		
		void on_set_acceleration(int acceleration_)
		{
			ESP_LOGD(TAG, "Set: acceleration!");
			acceleration_requested(acceleration_);
			acceleration(acceleration_);
		}
		
		void acceleration_requested(int acceleration_)
		{
			m_acceleration_requested = acceleration_;
		}
		
		/***************************************
	    ** deceleration (get/set - methods)
		***************************************/
		void deceleration(int  deceleration_)
		{
			int loc_deceleration = (deceleration_ > m_max_deceleration) ?  m_max_deceleration : deceleration_;
			m_deceleration = loc_deceleration;
			m_stepper->set_deceleration(loc_deceleration);
		}
		
		int deceleration(){return m_deceleration;}
		
		void set_deceleration_max()
		{
			deceleration(m_max_deceleration);
		}
		
		void set_deceleration_requested()
		{
			deceleration(m_deceleration_requested);
		}
		
		void on_set_deceleration(int deceleration_)
		{
			ESP_LOGD(TAG, "Set: deceleration!");
			deceleration_requested(deceleration_);
			deceleration(deceleration_);
		}
		
		void deceleration_requested(int deceleration_)
		{
			m_deceleration_requested = deceleration_;
		}
			
		
		/***************************************
	    ** step_width (get/set - methods)
		***************************************/
		void step_width(int  step_width_)
		{
			m_step_width = step_width_;
		}
		
		int step_width(){return m_step_width;}
		
	    void on_set_step_width(int step_width_)
		{
			step_width(step_width_);
		}
		
		/***************************************
	    ** stepper_error (get/set - methods)
		***************************************/
		void stepper_error(int  stepper_error_)
		{
			m_stepper_error = static_cast<eStepperError>(stepper_error_);
			// TODO: update stepper errpr sensor
		}
		
		eStepperError stepper_error(){return m_stepper_error;}
		
		/***************************************
	    ** stepper_mode (get/set - methods)
		***************************************/
		void stepper_mode(int stepper_mode_)
		{
			eStepperModes loc_stepper_mode = static_cast<eStepperModes>(stepper_mode_);
			init_stepper_mode(loc_stepper_mode);
			m_stepper_mode = loc_stepper_mode;
			m_sensor_stepper_mode->update();
			update();
		}
		
		eStepperModes stepper_mode(){return m_stepper_mode;}
		
		eAutomationModes automation_mode(){return m_automation_mode;}
		
		/***************************************
	    ** motor_enabled (get/set - methods)
		***************************************/
		void motor_enabled(bool motor_enabled_)
		{
			m_motor_enabled = motor_enabled_;
			if(m_motor_enabled)
			{
				stepper_mode(STEPPER_MODE_READY);
			}else
			{
				stop();
				stepper_mode(STEPPER_MODE_OFF);
			}
		}
		
		bool motor_enabled(){return m_motor_enabled;}
		
		void enable_motor(){motor_enabled(true);}
		
		void disable_motor(){motor_enabled(false);}
		
		void emergency_stop(const char* error_msg, eStepperError errorLevel)
		{
			ESP_LOGD(TAG, "Emergency-stop: %s  (errorCode: %d)", error_msg,  errorLevel);
			disable_motor();
			stepper_error(errorLevel);
		}
		
		
		void reset_errors()
		{
			stepper_error(STEPPER_ERROR_NONE);
			m_homing_difference = 0;
			m_pinion_wheel_count = expected_pinion_wheel_count();
		}
		
		/***************************************
	    ** requested_target_position (get/set/service - methods)
		***************************************/
        void requested_target_position(int requested_target_position_)
		{
			m_requested_target_position = requested_target_position_;
		}
		
		int requested_target_position(){return m_requested_target_position;}
		
		void on_set_requested_target_position(int requested_target_position_)
		{
			ESP_LOGD(TAG, "Set: requested_target_position!");
			requested_target_position(requested_target_position_);
		}
		
		/***************************************
	    ** set_zero_position
		***************************************/
		void set_zero_position(int zero_position)
		{
			xSemaphoreTake(mutex_pinion_wheels, 100 );
			int position = current_position() - zero_position;
			stop();
			set_position(position);
			target_position(position);
			pinion_wheel_count(expected_pinion_wheel_count());
			m_first_pinionwheel_measured = false;
			stop();
			xSemaphoreGive(mutex_pinion_wheels);
		}
		
		void on_set_zero_position(int zero_position_)
		{
			ESP_LOGD(TAG, "Set: zero_position!");
			set_zero_position(zero_position_);
		}
		
		void set_current_position_to_zero()
		{
			xSemaphoreTake(mutex_pinion_wheels, 100 );
			ESP_LOGD(TAG, "Set: zero_position = current_position!");
			stop();
			set_position(0);
			target_position(0);
			pinion_wheel_count(expected_pinion_wheel_count());
			m_first_pinionwheel_measured = false;
			stop();
			xSemaphoreGive(mutex_pinion_wheels);
		}
		
		
		/***************************************
	    ** current_position (steps)
		***************************************/
		int current_position()
		{
			return m_stepper->current_position;
		}
		
		
		float number_of_cycles()
		{
			float result = m_number_of_cycles->value();
			if (homing_is_valid() && m_automation_mode != AUTOMATION_MODE_OFF)
			{
				result += current_angle() / (3.0 * 360.0);
			}
			return result;
		}
		
		
		bool check_and_reset_position()
		{
			
			int position_loc = current_position();
			
			if (position_loc >= 3 * KINEMATICS_FULL_TURN_STEPS)
			{
				if (homing_is_valid() && m_automation_mode != AUTOMATION_MODE_OFF)
				{
					m_number_of_cycles->value() += 1;
				}
				
				xSemaphoreTake(mutex_main_loop, 100);
				xSemaphoreTake(mutex_pinion_wheels, 100 );
				{
					//pause();
					int position = current_position();
					int new_position = position - (3 * KINEMATICS_FULL_TURN_STEPS);
					int new_target = m_target_position - (3 * KINEMATICS_FULL_TURN_STEPS);
					
					set_position(new_position);
					target_position(new_target);
					//int new_pinion_wheel_count = pinion_wheel_count() - (3 * KINEMATICS_PINION_WHEEL_RODS_LOWER);
					int new_pinion_wheel_count = 0;
					pinion_wheel_count(new_pinion_wheel_count);
					//pause();
				}
				xSemaphoreGive(mutex_pinion_wheels);
				xSemaphoreGive(mutex_main_loop);
				return false;
				
			}else if(position_loc < 0)
			{
				if (homing_is_valid() && m_automation_mode != AUTOMATION_MODE_OFF)
				{
					m_number_of_cycles->value() -= 1;
				}
				
				xSemaphoreTake(mutex_main_loop, 100);
				xSemaphoreTake(mutex_pinion_wheels, 100 );
				{
					//pause();
					int position = current_position();
					int new_position = (3 * KINEMATICS_FULL_TURN_STEPS -1)  + position;
					int new_target = (3 * KINEMATICS_FULL_TURN_STEPS -1)  + m_target_position;

					set_position(new_position);
					//pause();
					target_position(new_target);
					pinion_wheel_count(expected_pinion_wheel_count());
					//int new_pinion_wheel_count = (3 * KINEMATICS_FULL_TURN_STEPS -1)  + pinion_wheel_count(); 
					int new_pinion_wheel_count = 0; 
					pinion_wheel_count(new_pinion_wheel_count);
					//pause();
				}
				xSemaphoreGive(mutex_pinion_wheels);
				xSemaphoreGive(mutex_main_loop);
				return false;
			}else
			{
				return true;
			}
		}
		
		
		/***************************************
	    ** current_angle (Degree)
		***************************************/
		float current_angle()
		{
			return (current_position() / (float)KINEMATICS_FULL_TURN_STEPS) * 360.0;
		}
		
		/***************************************
	    ** current_global_angle (Degree)
		***************************************/
		float current_global_angle()
		{
			return number_of_cycles() * 3 * 360.0;
		}
		
		
		/***************************************
	    ** current_angular_velocity (Degree)
		***************************************/
		float current_angular_velocity()
		{
			return speed() * (360.0 / (float)KINEMATICS_FULL_TURN_STEPS) ;
		}
	    
		/***************************************
	    ** current_turns_per_hour (Degree)
		***************************************/
		float current_turns_per_hour()
		{
			return current_angular_velocity() * 60 * 60 / (360.0);
		}
		
		/***************************************
	    ** current_cycles_per_hour (Degree)
		***************************************/
		float current_cycles_per_hour()
		{
			return current_turns_per_hour()*3.0;
		}
		
		
		/***************************************
	    ** Plant gear angle
		***************************************/
		float plant_gear_rotation_angle()
		{
			//float total_angle = ((float)KINEMATICS_PLANT_GEAR_DEGREES_PER_SLOT/(float)KINEMATICS_PLANT_GEAR_SLOTS) *
									current_angle();
			float total_angle = (200.0/60.0) * current_angle();
			float relative_angle = total_angle / 360.0;
			return 360.0 * (relative_angle - (int)relative_angle);
		}
		
		
		/***************************************
	    ** pinion_wheel_count
		***************************************/
		int pinion_wheel_count(){return m_pinion_wheel_count;}
		
		void pinion_wheel_count(int count_)
		{
			m_pinion_wheel_count = count_;
		}
		
		double expected_pinion_wheel_count()
		{
			double position = current_position();
			double expected_pinion_wheel_count = 0;
			
			if (position != 0)
			{
				expected_pinion_wheel_count = 0.5 + (position / (float)KINEMATICS_FULL_TURN_STEPS ) * (float)KINEMATICS_PINION_WHEEL_RODS_LOWER;
			}
			
			return expected_pinion_wheel_count;
		}
		
		bool get_pinion_wheel_diff_error()
		{
			int diff = abs(pinion_wheel_count() - expected_pinion_wheel_count());
			
			if (diff > PINION_WHEEL_ERROR && diff < KINEMATICS_PINION_WHEEL_RODS_LOWER*3 - PINION_WHEEL_ERROR)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		
		
		bool get_homing_diff_error()
		{
			int diff = homing_difference();
			
			if (abs(diff) > HOMING_DIFF_ERROR && homing_is_valid())
			{
				return true;
			}else
			{
				return false;
			}
		}
		
		
		/***************************************
	    ** homing_valid?
		***************************************/
		bool homing_is_valid()
		{
			return m_homing.valid;
		}
		
		void set_homing_invalid()
		{
			m_homing.valid = false;
		}
		
		void set_homing_valid()
		{
			m_homing.valid = true;
			m_homing_difference = 0;
		}
		
		/***************************************
	    ** start/stop/pause/resume?
		***************************************/
		void start()
		{
			if (motor_enabled())
			{
				ESP_LOGD(TAG, "Start stepper motor");
				set_target(m_target_position);
			}else
			{
				ESP_LOGD(TAG, "Motor is not enabled");
				stepper_mode(STEPPER_MODE_OFF);
			}
		}
		
		void stop()
		{
			ESP_LOGD(TAG, "Stop stepper motor");
			set_target(current_position());
			stepper_mode(STEPPER_MODE_READY);
		}
		
		void pause()
		{
			//ESP_LOGD(TAG, "Pause stepper motor");
			set_target(current_position());
		}
		

		void start_homing()
		{
			//xSemaphoreTake(mutex_main_loop, portMAX_DELAY);
			m_calibration_mode_active = true;
			m_global_homing = true;
			direction_forward(true);
			stepper_mode(STEPPER_MODE_HOMING);
			//xSemaphoreGive(mutex_main_loop);
		}
		
		void find_next_home()
		{
			//xSemaphoreTake(mutex_main_loop, portMAX_DELAY);
			m_calibration_mode_active = false;
			m_global_homing = false;
			direction_forward(true);
			stepper_mode(STEPPER_MODE_HOMING);
			//xSemaphoreGive(mutex_main_loop);
		}
		
		void find_next_global_home()
		{
			//xSemaphoreTake(mutex_main_loop, portMAX_DELAY);
			m_calibration_mode_active = false;
			m_global_homing = true;
			direction_forward(true);
			stepper_mode(STEPPER_MODE_HOMING);
			//xSemaphoreGive(mutex_main_loop);
		}
		
		int homing_difference(){return m_homing_difference;}
		
		void start_rangeestimation()
		{
			if(motor_enabled())
				stepper_mode(STEPPER_MODE_RANGEESTIMATION);
		}
		
		// TODO: resume() method
		
		/***************************************
	    ** Control methods
		***************************************/
		void goto_target(int target)
		{
			//xSemaphoreTake(mutex_main_loop, portMAX_DELAY);
			
			if(motor_enabled())
			{
				if (target > current_position())
				{
					direction_forward(true);
				}else
				{
					direction_forward(false);
				}
				
				target_position(target);
				stepper_mode(STEPPER_MODE_MANUAL);
				start();
			}
			
			//xSemaphoreGive(mutex_main_loop);
		}
		void goto_global_home()
		{
			//xSemaphoreTake(mutex_main_loop, portMAX_DELAY);
			int target = 0;
			int position = current_position();
			
			if (position > 0.5 * 3 * KINEMATICS_FULL_TURN_STEPS)
			{
				target = 3 * KINEMATICS_FULL_TURN_STEPS;
			}
				
			if(motor_enabled()){goto_target(target);}
			
			//xSemaphoreGive(mutex_main_loop);
			
		}
		void goto_requested_target(){if(motor_enabled()){goto_target(requested_target_position());}}
		
		void start_drive(bool direction_forward_)
		{
			//xSemaphoreTake(mutex_main_loop, portMAX_DELAY);
			
			direction_forward(direction_forward_);
			target_position(increment_target_position(2.0));
			start();
			stepper_mode(STEPPER_MODE_DRIVE);
			
			//xSemaphoreGive(mutex_main_loop);
			
		}
		
		void start_drive_forward(){if(motor_enabled()){start_drive(true);}}
		void start_drive_backward(){if(motor_enabled()){start_drive(false);}}
		

		
		void step(bool direction_forward_)
		{
			//xSemaphoreTake(mutex_main_loop, portMAX_DELAY);
			
			direction_forward(direction_forward_);
			target_position(current_position() + ( direction_sign() * m_step_width));
			stepper_mode(STEPPER_MODE_MANUAL);
			start();
			
			//xSemaphoreGive(mutex_main_loop);
		}
		void step_forward(){if(motor_enabled()){step(true);}}
		void step_backward(){if(motor_enabled()){step(false);}}
		
		bool is_active()
		{
			if(current_position() != target_position())
				return true;
			else
				return false;
		}
		
		
		/***************************************
	    ** Homing methods
		***************************************/
		void proximity_switch_home_pressed()
		{
			switch(m_stepper_mode_last)
			{
				case STEPPER_MODE_HOMING:
				{
					if((m_proximity_switch_global_home->state || !m_global_homing) && 
					   m_homing.sensors_enabled && 
					   m_homing.started &&
					   !m_homing.found_low_precision)
					{
						pause();
						m_homing.found_low_precision = true;
						m_homing.sensors_enabled = false;
						pause();
					}
					break;
				}
				
				case STEPPER_MODE_RANGEESTIMATION:
				{
					if (m_rangeestimation.started)
					{
						set_speed_mid();
					}
					break;
				} 
				
				default:
				{
					break;
				}
			}   
		}
		
		
		void proximity_switch_home_released()
		{
			
			switch(m_stepper_mode_last)
			{
				case STEPPER_MODE_HOMING:
				{
					if((m_proximity_switch_global_home->state || !m_global_homing)&& 
					m_homing.sensors_enabled && 
					m_homing.started &&
					m_homing.found_low_precision &&
					!m_homing.found_high_precision)
					{
						pause();
						m_homing.found_high_precision = true;
						m_homing.sensors_enabled = false;
						pause();
					}
					break;
				}
				
				case STEPPER_MODE_RANGEESTIMATION:
				{
					if (m_rangeestimation.started)
					{
						if (m_rangeestimation.found_home_counter == 0)
						{
							m_rangeestimation.range_0 = current_position();
							set_speed_high();
							m_rangeestimation.found_home_counter += 1;
							
						}else if((m_rangeestimation.found_home_counter > 0) && 
								(m_rangeestimation.found_home_counter <= NUMBER_OF_RANGEESTIMATIONS + 1))
						{
							m_rangeestimation.range_1 = current_position();
							m_rangeestimation.ranges[m_rangeestimation.found_home_counter - 1] =
									m_rangeestimation.range_1 - m_rangeestimation.range_0;
							int sum = 0;
							int sum_size = m_rangeestimation.found_home_counter;
							for (int i = 0; i < sum_size; i++)
							{
								sum += m_rangeestimation.ranges[i];
							}
							m_rangeestimation.range_average = (float)sum/sum_size;   
							
							int full_turn_steps_loc = (int)round(m_rangeestimation.range_average);
							m_measured_full_turn_steps->value() = full_turn_steps_loc;
							
							m_rangeestimation.range_0 = m_rangeestimation.range_1;
							set_speed_high();
							m_rangeestimation.found_home_counter += 1;
						}    
					}
					break;
				}
				default:
				{
					break;
				}
			}
		}
		
		/***************************************
	    ** Speed measurement methods
		***************************************/		
		void proximity_switch_pinion_wheel_pressed()
		{
			xSemaphoreTake(mutex_pinion_wheels, 100 );
			if(direction_forward())
			{
				//if (!m_first_pinionwheel_measured)
				//{
				//	m_first_pinionwheel_measured = true;
				//	pinion_wheel_count(0);
				//}else
				//{
					if(m_pinion_wheel_count>= KINEMATICS_PINION_WHEEL_RODS_LOWER*3 - 1)
					{
						m_pinion_wheel_count = 0 ; 
					}else
					{
						m_pinion_wheel_count += 1; 
					}
					
				//}
			}
			xSemaphoreGive(mutex_pinion_wheels);
		}
		
		void proximity_switch_pinion_wheel_released()
		{
			xSemaphoreTake(mutex_pinion_wheels, 100 );
			if(!direction_forward())
			{
				//if (!m_first_pinionwheel_measured)
				//{
				//	m_first_pinionwheel_measured = true;
				//	pinion_wheel_count(-1);
				//}else
				//{
					if(m_pinion_wheel_count <= 0)
					{
						m_pinion_wheel_count = KINEMATICS_PINION_WHEEL_RODS_LOWER*3 - 1 ; 
					}else
					{
						m_pinion_wheel_count -= 1; 
					}
				//}	
			}
			xSemaphoreGive(mutex_pinion_wheels);
		}
		
    protected:
	     
		bool m_direction_forward{false};
		globals::RestoringGlobalsComponent<int> * m_measured_full_turn_steps; 
		globals::RestoringGlobalsComponent<int> * m_number_of_cycles; 
		int m_target_position{0};
		int m_speed{0};
		int m_max_speed{0};
		int m_requested_speed{0};
		int m_acceleration{0};
		int m_max_acceleration{0};
		int m_acceleration_requested;
		int m_deceleration{0};
		int m_max_deceleration{0};
		int m_deceleration_requested{0};
		int m_step_width{0};
		eStepperError m_stepper_error{STEPPER_ERROR_NONE};
		eStepperModes m_stepper_mode{STEPPER_MODE_OFF};
		eAutomationModes m_automation_mode{AUTOMATION_MODE_OFF};
		eStepperModes m_stepper_mode_last{STEPPER_MODE_OFF};
		bool m_motor_enabled{false};
		int m_requested_target_position{0}; 
		
		int m_homing_difference{0};
		
		std::string m_device_name{std::string("")};
		std::string m_stepper_id{std::string("")};
		std::string m_controller_id{std::string("")};
		a4988::A4988 *m_stepper;
	    gpio::GPIOBinarySensor *m_proximity_switch_home;
		gpio::GPIOBinarySensor *m_proximity_switch_global_home;
		gpio::GPIOBinarySensor *m_proximity_switch_pinion_wheel;
		template_::TemplateSensor *m_sensor_stepper_mode;
		template_::TemplateBinarySensor *m_sensor_check_position;
		
		int m_update_interval_ms{1000};
		int m_sensor_update_interval_slow{60000};
		int m_sensor_update_interval_mid{10000};
		int m_sensor_update_interval_fast{1000};
		int m_sensor_update_interval_realtime{100};
		
		// homing parameters
		struct tHoming
		{
			bool valid{false};
			bool started{false};
			int high_speed{0};
			int mid_speed{0};
			int low_speed{0};
			bool sensors_enabled{false};
			bool found_low_precision{false};
			bool found_high_precision{false};
			
			tHoming(int high_speed_, 
					int mid_speed_, 
					int low_speed_):
					high_speed(high_speed_),
					mid_speed(mid_speed_),
					low_speed(low_speed_){}
		};
		
		tHoming m_homing{tHoming(0,0,0)};
		
		// Rangeestimation parameters
		struct tRangeestimation
		{
			bool started{false};
			int found_home_counter{0};
			//int ranges[NUMBER_OF_RANGEESTIMATIONS];
			std::vector<int> ranges;
			int range_0{0};
			int range_1{0};
			float range_average{0.0};
			
			tRangeestimation() : ranges(NUMBER_OF_RANGEESTIMATIONS, 0){}
		};
		
		tRangeestimation m_rangeestimation{tRangeestimation()};
		
		int m_pinion_wheel_count{0};
		bool m_first_pinionwheel_measured{false};
		
		bool m_calibration_mode_active{false};
		bool m_global_homing{true};
		
		/***************************************
	    ** target methods:
		** set_target
		***************************************/
		void set_target(int target)
		{
			m_stepper->set_target((m_motor_enabled == true) ? target : current_position());
		}
		
		void set_position(int position)
		{
			m_stepper->report_position(position);
		}
		
		int increment_target_position(float factor)
		{
			return (current_position() + (direction_sign() *  m_speed * (m_update_interval_ms/SECONDS_TO_MILLISECONDS) * factor));
		}
		
		/***************************************
	    ** check state changes
		***************************************/
		bool check_update_stepper_mode()
		{
			if (m_stepper_mode != m_stepper_mode_last)
			{
				m_stepper_mode_last = m_stepper_mode;
				return true;
			}else
			{
				return false;
			}
		}
		
		/***************************************
	    ** initialize states
		***************************************/
		void init_stepper_mode(eStepperModes stepper_mode)
		{
			switch(stepper_mode)
            {
				case STEPPER_MODE_OFF:
				{
					set_speed_zero();
					set_acceleration_requested();
					set_deceleration_requested();
					break;
				}
				
				case STEPPER_MODE_READY:
				{
					set_speed_requested();
					set_acceleration_requested();
					set_deceleration_requested();
					break;
				}
				
				case STEPPER_MODE_HOMING:
				{
					//m_homing.valid = false; //TODO: hinterfragen: wirklich zuruecksetzen??
					m_homing.started = false;
					m_homing.sensors_enabled = false;
					m_homing.found_low_precision = false;
					m_homing.found_high_precision = false;
					set_speed_high();
					set_acceleration_max();
					set_deceleration_max();
					break;
				}
				
				case STEPPER_MODE_RANGEESTIMATION:
				{
					m_rangeestimation.found_home_counter  = 0;
					m_rangeestimation.range_0 = 0;
					m_rangeestimation.range_1 = 0;
					m_rangeestimation.range_average  = 0.0;
					m_rangeestimation.started = false;
					std::fill(m_rangeestimation.ranges.begin(), m_rangeestimation.ranges.end(), 0);
					set_speed_high();
					set_acceleration_max();
					set_deceleration_max();
					break;
				}
				
				case STEPPER_MODE_DRIVE:
				{
				    set_speed_requested();
					set_acceleration_requested();
					set_deceleration_requested();
					break;
				}
				
				case STEPPER_MODE_MANUAL:
				{
				    set_speed_requested();
					set_acceleration_requested();
					set_deceleration_requested();
					break;
				}
				
				default:
				{
					break;
				}
            }
		}
		
		/***************************************
	    ** main loop methods
		** get called by update() periodically
		***************************************/
		void main_loop()
		{
			xSemaphoreTake(mutex_main_loop, 100 );
			if (check_update_stepper_mode())
				return;
			
			switch(m_stepper_mode_last)
            {
				case STEPPER_MODE_HOMING:
				{
					stepper_mode_homing();
					break;
				}
				
				case STEPPER_MODE_RANGEESTIMATION:
				{
					stepper_mode_rangeestimation();
					break;
				}
				
				case STEPPER_MODE_DRIVE:
				{
				    stepper_mode_drive();
					break;
				}
				
				case STEPPER_MODE_MANUAL:
				{
				    stepper_mode_manual();
					break;
				}
				
				default:
				{
					break;
				}
            }
			xSemaphoreGive(mutex_main_loop);
		}
		
		
		/***************************************
	    ** actual state methods
		***************************************/
		
		/******************************************************************************************
        * 
		* **STEPPER_MODE_HOMING**
		*
		* homing mode: find out, if we already are on the homing position/sensor is already active
        * if so: drive backward until sensor turns off
        * then, start homing procedure:
        * drive forward with homing high speed until sensor detects home position at low accuracy
        * drive slowly forward until sensor detects "on_release" and set home position to this point
        ******************************************************************************************/
		void stepper_mode_homing()
		{
			if(!m_homing.started)
            {
				// drive backward until we are out of range from the homing position stop
				if(m_proximity_switch_home->state)
				{
					set_speed_high();
					direction_forward(false);
					drive(3.0);
				}else
				{
					m_homing.started = true;
				}
            }else
            {
				if(!m_homing.found_low_precision)
				{ 
					set_speed_high();
					direction_forward(true);
					m_homing.sensors_enabled = true;               
					drive(1.5);
				}else
				{
					if(!m_homing.found_high_precision)
					{
						set_speed_low();
						direction_forward(false);
						m_homing.sensors_enabled = true;                 
						drive(1.5);
					}else
					{
						set_homing_valid();
						m_homing.sensors_enabled = false;
						set_speed_requested();
						if (m_calibration_mode_active)
						{
							set_current_position_to_zero();
							m_calibration_mode_active = false;
						}else
						{
							
							if (homing_is_valid())
							{
								int loc_position = current_position();
								if (m_global_homing)
								{
									m_homing_difference = loc_position;
								}else
								{
									if ( loc_position >= 0 * KINEMATICS_FULL_TURN_STEPS &&
										 loc_position < 0.5 * KINEMATICS_FULL_TURN_STEPS)
									{
										m_homing_difference = loc_position;
										
									}
									else if ( loc_position >= 2.5 * KINEMATICS_FULL_TURN_STEPS ||
									    loc_position < 0 * KINEMATICS_FULL_TURN_STEPS)
									{
										m_homing_difference = 3 * KINEMATICS_FULL_TURN_STEPS - loc_position;
										
									}
									else if(loc_position >=  0.5 * KINEMATICS_FULL_TURN_STEPS &&
											 loc_position < 1.5 * KINEMATICS_FULL_TURN_STEPS)
									{
										m_homing_difference = KINEMATICS_FULL_TURN_STEPS - loc_position;
									}
									else if(loc_position >= 1.5 * KINEMATICS_FULL_TURN_STEPS &&
									        loc_position < 2.5 * KINEMATICS_FULL_TURN_STEPS)
									{
										m_homing_difference = 2 * KINEMATICS_FULL_TURN_STEPS - loc_position;
									}
								}
							}
						}
						
						stepper_mode(STEPPER_MODE_READY);
					}
				}
            }
		}
		
		/******************************************************************************************
        * 
		* **STEPPER_MODE_RANGEESTIMATION**
		*
        * measure full turn steps: go two rounds and calculate
        * the difference in steps between both "on_release" triggers
        * of the distance sensor
        * and check if the result is reasonable, 
        * TODO: if not reasonable -> set an error condition
        ******************************************************************************************/
		void stepper_mode_rangeestimation()
		{
			if(!m_rangeestimation.started)
            {
				// drive backward until we are out of range from the homing position stop
				if(m_proximity_switch_home->state)
				{
					set_speed_high();
					direction_forward(false);
					drive(3.0);
				}else
				{
					m_rangeestimation.started = true;
				}
            }else
            {
				if (m_rangeestimation.found_home_counter <= NUMBER_OF_RANGEESTIMATIONS + 1)
				{
					direction_forward(true);
					drive(1.5);
				}else
				{
					stop();
					m_rangeestimation.found_home_counter = 0;
					m_rangeestimation.range_0 = 0;
					m_rangeestimation.range_1 = 0;
					m_rangeestimation.started = false;
					set_speed_requested();
					stepper_mode(STEPPER_MODE_READY);
				}
            }
		}
		
		void stepper_mode_drive()
		{        
			drive(1.5);
		}
		
		void stepper_mode_manual()
		{
			start();
		}

		// oprational methods
		void drive(float increment_step)
		{
			target_position(increment_target_position(increment_step));
			start();
			
		}
		
		
	private:
	   
	
};