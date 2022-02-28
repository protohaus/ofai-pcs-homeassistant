#pragma once

#include <esphome.h>

// TODO: implement an error handling with an uint_32t error parameter and use bitshifting operations
//       to store errors on different bit positions and give back the whole integer number as error code to the API
//       then decode the uint32 to get the set error bits -> this will save storage + make the error handling easier


struct tStepperError
{
	bool ERROR_DIAG{false};
	bool ERROR_STUCK{false};
	bool ERROR_PINIONWHEEL_MISMATCH{false};
	bool ERROR_API_CONNECTION{false};
	bool ERROR_USER{false};
	bool ERROR_HOMING{false};
	
	bool error_state()
	{
		if(ERROR_DIAG ||
		   ERROR_STUCK ||
		   ERROR_PINIONWHEEL_MISMATCH ||
		   ERROR_API_CONNECTION ||
		   ERROR_USER ||
		   ERROR_HOMING
		   )
		   {
			   return true;
		   }else
		   {
			   return false;
		   }
	}
	
	void reset_error_state()
	{
		ERROR_DIAG = true;
		ERROR_STUCK = true;
		ERROR_PINIONWHEEL_MISMATCH = true;
		ERROR_API_CONNECTION = true;
		ERROR_USER = true;
		ERROR_HOMING = true;
	}
	
};

enum eStepperError
{
	STEPPER_ERROR_NONE = 0,
	STEPPER_ERROR_DIAG = 1,
	STEPPER_ERROR_STUCK = 2,
	STEPPER_ERROR_PINIONWHEEL_MISMATCH = 3,
	STEPPER_ERROR_CONNECTION = 4,
	STEPPER_ERROR_USER = 5,
	STEPPER_ERROR_HOMING = 6
};