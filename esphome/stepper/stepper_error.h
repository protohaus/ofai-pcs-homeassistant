#pragma once

#include <esphome.h>

enum eStepperError
{
	STEPPER_ERROR_NONE = 0,
	STEPPER_ERROR_DIAG = 1,
	STEPPER_ERROR_STUCK = 2
};