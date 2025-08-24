// Copyright 2022 Dado Mista
// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include "atr.h"
#include "conf/datatypes.h"
#include "imu.h"
#include "motor_data.h"
#include <stddef.h>

typedef struct {
    float factor;
    float target;
    float setpoint;
    bool hold_tilt_active;
    float hold_tilt_value;
    int hold_counter;

    // Hold-Tilt detection state
    float pitch_timer;
    float pitch_at_trigger;
    int hold_tilt_pitch_drop_detected;
} BrakeTilt;

void brake_tilt_init(BrakeTilt *bt);

void brake_tilt_reset(BrakeTilt *bt);

void brake_tilt_configure(BrakeTilt *bt, const RefloatConfig *config);

// New extended update function with IMU, dt and wheelslip for Hold-Tilt trigger logic
void brake_tilt_update(
    BrakeTilt *bt,
    const MotorData *motor,
    const ATR *atr,
    const RefloatConfig *config,
    float balance_offset,
    const IMU *imu,
    float dt,
    bool wheelslip
);

// Backward-compatible inline wrapper for legacy calls (without IMU/dt)
static inline void brake_tilt_update_legacy(
    BrakeTilt *bt,
    const MotorData *motor,
    const ATR *atr,
    const RefloatConfig *config,
    float balance_offset
) {
    brake_tilt_update(bt, motor, atr, config, balance_offset, NULL, 0.0f, false);
}

void brake_tilt_winddown(BrakeTilt *bt);
