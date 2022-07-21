// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "RcServo.h"

namespace MotorDrivers {
    class BldcServo : public RcServo {
    protected:
        Pin _dir_pin;

        void set_location() override;
        void validate() const override;

    public:

        void init() override;

        void group(Configuration::HandlerBase& handler) override {
            RcServo::group(handler);

            handler.item("direction_pin", _dir_pin);
        }

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override { return "bldc_servo"; }
    };
}
