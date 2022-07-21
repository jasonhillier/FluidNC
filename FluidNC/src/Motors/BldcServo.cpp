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
        _stop_pin.setAttr(Pin::Attr::Output);
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

        servo_pos ? _stop_pin.on() : _stop_pin.off();
        
        //log_info("servo_pos: " << servo_pos);
        if (servo_pos >= 0)
        {
            _dir_pin.off();
        }
        else
        {
            _dir_pin.on();
        }

        // determine the pulse length
        servo_pulse_len = static_cast<uint32_t>(mapConstrain(
            fabs(servo_pos), 0.0f, fabs(limitsMinPosition(_axis_index)), (float)_min_duty, (float)_max_duty));

        //log_info("pwm: " << servo_pulse_len << " [" << _min_duty << "|" << _max_duty);

        _write_pwm(servo_pulse_len);
    }

    void BldcServo::validate() const {
        RcServo::validate();
        
        Assert(_dir_pin.defined(), "Dir pin must be configured.");
        //stop_pin optional
    }

    float BldcServo::fabs(const float p)
    {
        return p > 0 ? p : p*-1.0f;
    }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<BldcServo> registration("bldc_servo");
    }
}