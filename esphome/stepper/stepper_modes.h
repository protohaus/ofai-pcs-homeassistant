#pragma once

#include <esphome.h>

enum eStepperModes
{
	STEPPER_MODE_OFF = 0,
	STEPPER_MODE_READY = 1,
	STEPPER_MODE_HOMING = 2,
	STEPPER_MODE_RANGEESTIMATION = 3,
	STEPPER_MODE_DRIVE = 10,
	STEPPER_MODE_MANUAL = 11
};

enum eAutomationModes
{
	AUTOMATION_MODE_OFF = 0,
	AUTOMATION_MODE_30DEG = 1,
	AUTOMATION_MODE_60DEG = 2,
	AUTOMATION_MODE_180DEG = 3,
	AUTOMATION_MODE_360DEG = 4,
	AUTOMATION_MODE_720DEG = 5,
	AUTOMATION_MODE_1080DEG = 6
};