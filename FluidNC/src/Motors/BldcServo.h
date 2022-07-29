// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "RcServo.h"

namespace MotorDrivers {
    class BldcServo : public RcServo {
    private:
        float fabs(const float p);

    protected:
        Pin _dir_pin;
        Pin _stop_pin;
        uint32_t _min_duty = 0;
        uint32_t _max_duty = SERVO_PWM_MAX_DUTY;
        bool _running=false;
        bool _reverse=false;

        void set_location() override;
        void validate() const override;        

    public:
        BldcServo() = default;

        void init() override;

        void group(Configuration::HandlerBase& handler) override {
            RcServo::group(handler);

            handler.item("direction_pin", _dir_pin);
            handler.item("stop_pin", _stop_pin);
            handler.item("min_duty", _min_duty);
            handler.item("max_duty", _max_duty);
        }

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override { return "bldc_servo"; }
    };
}
