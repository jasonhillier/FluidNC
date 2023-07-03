// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    EncodedStepper.cpp -- EncodedStepper type stepper drivers
*/

#include "../Gcode.h"
#include "StandardStepper.h"
#include "Modbus.h"

//extern parser_state_t gc_state;

namespace MotorDrivers {
    class EncodedStepper : public StandardStepper, public ModbusHandler {
    protected:
        uint32_t _angle_step_ratio;
        int32_t _current_angle = -1;
        int32_t _homing_angle_offset = 0;
        int _axis_index;

    public:
        EncodedStepper() = default;

        void init() override;

        void update() override;

        void onSetCurrentAngle(int32_t angle) override;

        // Configuration handlers:
        void validate() const override;
        void group(Configuration::HandlerBase& handler) override;

        void afterParse() override;

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override;

        ~EncodedStepper() = default;
    };
}
