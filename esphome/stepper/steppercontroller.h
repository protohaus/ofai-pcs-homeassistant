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
		  int acceleration,
		  int deceleration,
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
		  m_requested_speed(initial_speed),
		  m_acceleration(acceleration),
		  m_deceleration(deceleration),
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
				ESP_LOGD("custom", "Warning: Full turn steps are OFF too far, using default value!");
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
			m_speed = speed_;
		}
		
		int speed(){return m_speed;}
		
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
			m_acceleration = acceleration_;
		}
		
		int acceleration(){return m_acceleration;}
		
		
		/***************************************
	    ** deceleration (get/set - methods)
		***************************************/
		void deceleration(int  deceleration_)
		{
			m_deceleration = deceleration_;
		}
		
		int deceleration(){return m_deceleration;}
		
		
		/***************************************
	    ** step_width (get/set - methods)
		***************************************/
		void step_width(int  step_width_)
		{
			m_step_width = step_width_;
		}
		
		int step_width(){return m_step_width;}
		
	    
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
		}
		bool motor_enabled(){return m_motor_enabled;}
		
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
		
		
    protected:
	    
		bool m_direction_forward{false};
		globals::RestoringGlobalsComponent<int> * m_full_turn_steps; 
		int m_initial_full_turn_steps{0};
		int m_target_position{0};
		int m_speed{0};
		int m_requested_speed{0};
		int m_acceleration{0};
		int m_deceleration{0};
		int m_step_width{0};
		eStepperError m_stepper_error{STEPPER_ERROR_NONE};
		eStepperModes m_stepper_mode{STEPPER_MODE_OFF};
		bool m_motor_enabled{false};
		int m_requested_target_position{0}; 
		
		std::string m_device_name{std::string("")};
		std::string m_stepper_id{std::string("")};
		std::string m_controller_id{std::string("")};
		a4988::A4988 *m_stepper;
	
	private:
	   
	
};