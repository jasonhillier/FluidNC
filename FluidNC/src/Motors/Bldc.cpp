// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Bldc.cpp -- Bldc type stepper drivers
*/

#include "Bldc.h"

namespace MotorDrivers {
    void Bldc::init() {

        StandardStepper::init();
    }

    // Configuration handlers:
    void Bldc::validate() const { StandardStepper::validate(); }

    void Bldc::group(Configuration::HandlerBase& handler) {
        StandardStepper::group(handler);
    }

    void Bldc::afterParse() {
    }

    void IRAM_ATTR Bldc::step() {
        _step_pin.on();
    }

    void IRAM_ATTR Bldc::unstep() {
        //_step_pin.off();
    }

    void IRAM_ATTR Bldc::set_disable(bool disable)
    {
        if (disable)
        {
            _step_pin.off();
        }

        StandardStepper::set_disable(disable);
    }

    // Name of the configurable. Must match the name registered in the cpp file.
    const char* Bldc::name() const { return "Bldc"; }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<Bldc> registration("Bldc");
    }
}
