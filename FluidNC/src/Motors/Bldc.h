// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    Bldc.cpp -- Bldc type stepper drivers
*/

#include "StandardStepper.h"

namespace MotorDrivers {
    class Bldc : public StandardStepper {

    public:
        Bldc() = default;

        void init() override;

        // Configuration handlers:
        void validate() const override;
        void group(Configuration::HandlerBase& handler) override;

        void step() override;
        void unstep() override;
        void set_disable(bool disable) override;

        void afterParse() override;

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override;

        ~Bldc() = default;
    };
}
