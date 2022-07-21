// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Bldc.cpp -- Bldc type stepper drivers
*/

#include "BldcServo.h"
#include "../System.h"  // mpos_to_steps() etc
#include "../Limits.h"  // limitsMaxPosition
#include "../NutsBolts.h"

namespace MotorDrivers {
    void BldcServo::init() {
        RcServo::init();

        _dir_pin.setAttr(Pin::Attr::Output);
    }

    void BldcServo::set_location() {
        if (_disabled || _has_errors) {
            return;
        }

        uint32_t servo_pulse_len;
        float    servo_pos;

        read_settings();

        float mpos = steps_to_mpos(get_axis_motor_steps(_axis_index), _axis_index);  // get the axis machine position in mm
        servo_pos  = mpos;                                         // determine the current work position
        
        if (servo_pos > 0)
        {
            _dir_pin.off();
        }
        else
        {
            _dir_pin.on();
        }

        // determine the pulse length
        servo_pulse_len = static_cast<uint32_t>(mapConstrain(
            servo_pos, limitsMinPosition(_axis_index), limitsMaxPosition(_axis_index), (float)_min_pulse_cnt, (float)_max_pulse_cnt));

        //log_info("su " << servo_pulse_len);

        _write_pwm(servo_pulse_len);
    }

    void BldcServo::validate() const {
        RcServo::validate();
        
        Assert(_dir_pin.defined(), "Step pin must be configured.");
    }
}