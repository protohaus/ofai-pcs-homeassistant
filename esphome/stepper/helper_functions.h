#include <esphome.h>

enum eStepperError
{
	STEPPER_ERROR_NONE = 0,
	STEPPER_ERROR_DIAG = 1,
	STEPPER_ERROR_STUCK = 2
}stepper_errors;

enum eStepperModes
{
	STEPPER_MODE_OFF = 0,
	STEPPER_MODE_READY = 1,
	STEPPER_MODE_HOMING = 2,
	STEPPER_MODE_RANGEESTIMATION = 3,
	STEPPER_MODE_DRIVE = 10,
	STEPPER_MODE_TARGET = 11,
	STEPPER_MODE_STEP_WIDTH = 12,
	STEPPER_MODE_STEP_30DEG = 20,
	STEPPER_MODE_STEP_60DEG = 21,
	STEPPER_MODE_STEP_180DEG = 22,
	STEPPER_MODE_STEP_360DEG = 23
}stepper_modes;

int increment_target_position(int position, bool direction, int speed, float factor)
{
	return (position + (direction == true ? 1 : -1) *  speed * factor);
}

