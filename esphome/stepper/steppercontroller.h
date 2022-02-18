#pragma once

#include <esphome.h>
#include <stepper_modes.h>
#include <stepper_error.h>
#define get_controller(constructor) static_cast<cStepperController *>(const_cast<esphome::custom_component::CustomComponentConstructor *>(&constructor)->get_component(0))

class cStepperController : public Component, public CustomAPIDevice {
	public:
	    
	    /********************************************************************************
	    ** cCustomController Constructor
		**  loads all settings and other external objects like stepper, sensors, buttons
		********************************************************************************/
		cStepperController
		(
		  int initial_full_turn_steps,
		  int initial_speed,
		  int max_speed,
		  int acceleration,
		  int max_acceleration,
		  int deceleration,
		  int max_deceleration,
		  int step_width,
		  std::string device_name,
		  std::string stepper_id,
		  std:: string controller_id,
		  a4988::A4988 *stepper
		)
		:
		  Component(),
		  CustomAPIDevice(),
		  m_initial_full_turn_steps(initial_full_turn_steps),
		  m_speed(initial_speed),
		  m_max_speed(max_speed),
		  m_requested_speed(initial_speed),
		  m_acceleration(acceleration),
		  m_max_acceleration(max_acceleration),
		  m_deceleration(deceleration),
		  m_max_deceleration(max_deceleration),
		  m_step_width(step_width),
		  m_device_name(device_name),
		  m_stepper_id(stepper_id),
		  m_controller_id(controller_id),
		  m_stepper(stepper)
		{
			// attention: duplicated hashes may lead to problems 
			// watch out: esphome autopgenerates hashes itself (check main.cpp)
			m_full_turn_steps = new globals::RestoringGlobalsComponent<int>(initial_full_turn_steps);
			m_full_turn_steps->set_component_source("globals");
			m_full_turn_steps->set_name_hash(659212362);
			App.register_component(m_full_turn_steps);
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
						
			register_service(&cStepperController::on_set_full_turn_steps, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_full_turn_steps",
						{"full_turn_steps"});	
						
			register_service(&cStepperController::on_set_deceleration, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_deceleration",
						{"deceleration"});
			
			register_service(&cStepperController::on_set_acceleration, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_acceleration",
						{"acceleration"});
			
			register_service(&cStepperController::on_set_speed, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_speed",
						{"speed"});
			
			register_service(&cStepperController::on_set_step_width, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_step_width",
						{"step_width"});

			register_service(&cStepperController::on_set_zero_position, 
						m_stepper_id + "_" + m_controller_id + "_" + "set_zero_position",
						{"zero_position"});
		    
						
        };					 
		
		/*************************************************************************
		*************** GETTER, SETTER AND SERVICE METHOD DEFINITIONS ************
		**************************************************************************/
		
		/***************************************
	    ** direction_forward (get/set - methods)
		***************************************/
		void direction_forward(int  direction_forward_)
		{
			m_direction_forward = direction_forward_;
		}
		
		bool direction_forward(){return m_direction_forward;}
		
		int direction_sign(){return (direction_forward() == true ? 1 : -1);}
		
		/***************************************
	    ** full_turn_steps (get/set/service - methods)
		***************************************/
		void full_turn_steps(int  full_turn_steps_)
		{
			/* TODO:
			*  - add pause call
			*/
			
			if(std::abs((full_turn_steps_ / m_initial_full_turn_steps) - 1) < 0.05)
            {
				m_full_turn_steps->value() = full_turn_steps_;
            }else
            {
				ESP_LOGD("custom", "Warning: Full turn steps are too far off... using default value!");
				m_full_turn_steps->value() = m_initial_full_turn_steps;
            }
		}
		
		int full_turn_steps(){return m_full_turn_steps->value();}
		
		void on_set_full_turn_steps(int full_turn_steps_)
		{
			ESP_LOGD("custom", "Set: full_turn_steps!");
			full_turn_steps(full_turn_steps_);
		}
		
		/***************************************
	    ** target_position (get/set - methods)
		***************************************/
		void target_position(int  target_position_)
		{
			m_target_position = target_position_;
		}
		
		int target_position(){return m_target_position;}
		
		
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
		
		void on_set_acceleration(int acceleration_)
		{
			ESP_LOGD("custom", "Set: acceleration!");
			acceleration(acceleration_);
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
		
		void on_set_deceleration(int deceleration_)
		{
			ESP_LOGD("custom", "Set: deceleration!");
			deceleration(deceleration_);
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
			m_stepper_mode = static_cast<eStepperModes>(stepper_mode_);
			// TODO: update stepper mode sensor
		}
		
		eStepperModes stepper_mode(){return m_stepper_mode;}
		
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
			ESP_LOGD("custom", "Set: requested_target_position!");
			requested_target_position(requested_target_position_);
		}
		
		/***************************************
	    ** set_zero_position
		***************************************/
		void set_zero_position(int zero_position)
		{
			pause();
			set_position(current_position() - zero_position);
			pause();
		}
		
		void on_set_zero_position(int zero_position_)
		{
			ESP_LOGD("custom", "Set: zero_position!");
			set_zero_position(zero_position_);
		}
		
		void set_current_position_to_zero()
		{
			ESP_LOGD("custom", "Set: zero_position = current_position!");
			pause();
			set_position(0);
			target_position(0);
			pause();
		}
		
		
		/***************************************
	    ** current_position
		***************************************/
		int current_position()
		{
			return m_stepper->current_position;
		}
		
		/***************************************
	    ** start/stop/pause/resume?
		***************************************/
		void start()
		{
			if (m_motor_enabled)
			{
				ESP_LOGD("custom", "Start stepper motor");
				set_target(m_target_position);
			}else
			{
				ESP_LOGD("custom", "Motor is not enabled");
				stepper_mode(STEPPER_MODE_OFF);
			}
		}
		
		void stop()
		{
			ESP_LOGD("custom", "Stop stepper motor");
			set_target(current_position());
			stepper_mode(STEPPER_MODE_READY);
		}
		
		void pause()
		{
			ESP_LOGD("custom", "Pause stepper motor");
			set_target(current_position());
		}
		
		// TODO: resume() method
		
		/***************************************
	    ** Control methods
		***************************************/
		void goto_target(int target)
		{
			set_target(target);
			stepper_mode(STEPPER_MODE_TARGET);
			start();
		}
		void goto_global_home(){goto_target(0);}
		void goto_requested_target(){goto_target(requested_target_position());}
		
		void drive(bool direction_forward_)
		{
			direction_forward(direction_forward_);
			set_target(current_position() + (direction_sign() * m_max_speed * 2 ));
			stepper_mode(STEPPER_MODE_DRIVE);
			start();
		}
		void drive_forward(){drive(true);}
		void drive_backward(){drive(true);}

		void step(bool direction_forward_)
		{
			direction_forward(direction_forward_);
			target_position(current_position() + ( direction_sign() * m_step_width));
			stepper_mode(STEPPER_MODE_STEP_WIDTH);
			start();
		}
		void step_forward(){step(true);}
		void step_backward(){step(false);}
		
		
    protected:
	    
		bool m_direction_forward{false};
		globals::RestoringGlobalsComponent<int> * m_full_turn_steps; 
		int m_initial_full_turn_steps{0};
		int m_target_position{0};
		int m_speed{0};
		int m_max_speed{0};
		int m_requested_speed{0};
		int m_acceleration{0};
		int m_max_acceleration{0};
		int m_deceleration{0};
		int m_max_deceleration{0};
		int m_step_width{0};
		eStepperError m_stepper_error{STEPPER_ERROR_NONE};
		eStepperModes m_stepper_mode{STEPPER_MODE_OFF};
		bool m_motor_enabled{false};
		int m_requested_target_position{0}; 
		
		std::string m_device_name{std::string("")};
		std::string m_stepper_id{std::string("")};
		std::string m_controller_id{std::string("")};
		a4988::A4988 *m_stepper;
	    
		
		/***************************************
	    ** target methods:
		** set_target
		***************************************/
		void set_target(int target)
		{
			m_stepper->set_target((m_motor_enabled == true) ? target : m_stepper->current_position);
		}
		
		void set_position(int position)
		{
			m_stepper->report_position(position);
		}
		
	private:
	   
	
};