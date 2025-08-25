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

#include "brake_tilt.h"

#include "utils.h"

#include <math.h>

void brake_tilt_init(BrakeTilt *bt) {
    bt->factor = 0.0f;
    brake_tilt_reset(bt);
}

void brake_tilt_reset(BrakeTilt *bt) {
    bt->target = 0;
    bt->setpoint = 0;
    bt->hold_tilt_active = false;
    bt->hold_tilt_value = 0;
    bt->hold_counter = 0;
    bt->pitch_timer = 0.0f;
    bt->pitch_at_trigger = 0.0f;
    bt->hold_tilt_pitch_drop_detected = 0;
}

void brake_tilt_configure(BrakeTilt *bt, const RefloatConfig *config) {
    if (config->braketilt_strength == 0) {
        bt->factor = 0;
    } else {
        // incorporate negative sign into braketilt factor instead of adding it each balance loop
        bt->factor = -(0.5f + (20 - config->braketilt_strength) / 5.0f);
    }
}

void brake_tilt_update(
    BrakeTilt *bt,
    const MotorData *motor,
    const ATR *atr,
    const RefloatConfig *config,
    float balance_offset,
    const IMU *imu,
    float dt,  // time since last update, in seconds
    bool wheelslip
) {
    // braking also should cause setpoint change lift, causing a delayed lingering nose lift
    // --- Threshold selection for incline/decline detection ---
    float incline_threshold = config->incline_threshold_default;

    // --- Only on level ground and no wheelslip ---
    if (wheelslip || atr->accel_diff > incline_threshold || atr->accel_diff < -incline_threshold) {
        // Deactivate Brake-Tilt and Hold-Tilt if wheelslip, incline or decline detected
        bt->target = 0;
        if (bt->hold_tilt_active) {
            bt->hold_tilt_active = false;
            bt->hold_counter = 0;
        }

        return;
    }

    // --- Brake-Tilt Activation (only on level ground) ---
    if (bt->factor < 0 && motor->braking && motor->abs_erpm > 2000) {
        // negative currents alone don't necessarily constitute active braking, look at
        // proportional:
        if (sign(balance_offset) != motor->erpm_sign) {
            float downhill_damper = 1;
            // if we're braking on a downhill we don't want braking to lift the setpoint quite as
            // much
            if ((motor->erpm > 1000 && atr->accel_diff < -1) ||
                (motor->erpm < -1000 && atr->accel_diff > 1)) {
                downhill_damper += fabsf(atr->accel_diff) / 2;
            }
            bt->target = balance_offset / bt->factor / downhill_damper;
            if (downhill_damper > 2) {
                // steep downhills, we don't enable this feature at all!
                bt->target = 0;
            }
        }
    } else {
        bt->target = 0;
    }

    // Detect rapid pitch drop while Brake-Tilt is active
    if (!bt->hold_tilt_active && fabsf(bt->target) > config->hold_tilt_min_target && !wheelslip) {
        // Start or continue pitch monitoring window
        if (bt->pitch_timer <= 0.0f) {
            bt->pitch_at_trigger = imu->pitch;
            bt->pitch_timer = config->hold_tilt_time_window;
            bt->hold_tilt_pitch_drop_detected = 0;
        } else {
            float pitch_delta = bt->pitch_at_trigger - imu->pitch;
            if (pitch_delta > config->hold_tilt_pitch_delta_threshold) {
                bt->hold_tilt_pitch_drop_detected = 1;
            }
            bt->pitch_timer -= dt;
        }
    } else {
        bt->pitch_timer = 0.0f;
        bt->hold_tilt_pitch_drop_detected = 0;
    }

    // Activate Hold-Tilt if pitch drop detected
    if (!bt->hold_tilt_active && bt->hold_tilt_pitch_drop_detected) {
        bt->hold_tilt_active = true;
        bt->hold_tilt_value = config->hold_tilt_angle;
        bt->hold_counter = config->hold_tilt_timeout;
        bt->pitch_timer = 0.0f;
        bt->hold_tilt_pitch_drop_detected = 0;
    }

    // --- Hold-Tilt Behavior ---
    if (bt->hold_tilt_active) {
        bt->setpoint = bt->hold_tilt_value;
        // Enhancement: check if the balance pitch is below the target hold tilt value
        if (imu->balance_pitch < bt->hold_tilt_value) {
            float diff = bt->hold_tilt_value - imu->balance_pitch;
            bt->setpoint += 0.5f * diff;
        }
        if (--bt->hold_counter <= 0) {
            bt->hold_tilt_active = false;
            bt->hold_counter = 0;
        }
        return;
    }

    // --- Brake-Tilt Step Logic ---
    float braketilt_step_size = atr->off_step_size / config->braketilt_lingering;
    if (fabsf(bt->target) > fabsf(bt->setpoint)) {
        braketilt_step_size = atr->on_step_size * 1.5f;
    } else if (motor->abs_erpm < 800) {
        braketilt_step_size = atr->on_step_size;
    }
    if (motor->abs_erpm < 500) {
        braketilt_step_size /= 2;
    }
    rate_limitf(&bt->setpoint, bt->target, braketilt_step_size);
}

void brake_tilt_winddown(BrakeTilt *bt) {
    bt->setpoint *= 0.995f;
    bt->target *= 0.99f;
    // Ensure hold-tilt mode is cleared on winddown
    if (bt->hold_tilt_active) {
        bt->hold_tilt_active = false;
        bt->hold_counter = 0;
    }
}
