#pragma once

#include <esphome.h>
int increment_target_position(int position, bool direction, int speed, float factor)
{
	return (position + (direction == true ? 1 : -1) *  speed * factor);
}

