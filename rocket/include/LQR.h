// ==================================
// ========= LQR (Header) ===========
// ==================================

/*
 *
* By Gunnar and Nicholas
*/

#pragma once

#ifndef LQR_H
#define LQR_H

#include <Arduino.h>
#include <settings.h>
#include <GlobalDecRocket.h>
#include <BasicLinearAlgebra.h>

// Inits
void lqrInit();
void get_tradj_ref(float current_time);
void lqr(float x_dot, float gamma1, float gamma1_dot, float y_dot, float gamma2, float gamma2_dot, float z, float z_dot, float t0, LqrSignals& lqrSignals);

#endif