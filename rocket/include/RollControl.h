#pragma once

#ifndef ROLLCONTROL_H
#define ROLLCONTROL_H

#include <Arduino.h>
#include <settings.h>

int roll_p_controller(float yaw_angle, int pwm);



#endif