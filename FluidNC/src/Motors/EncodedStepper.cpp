// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    EncodedStepper.cpp -- EncodedStepper type stepper drivers
    TODO: we may need a PID controller here
    TODO: make it so we can attach different encoders.
*/

#include "EncodedStepper.h"
#include "../System.h"  // mpos_to_steps() etc

namespace MotorDrivers {
    
    void EncodedStepper::init() {
        _axis_index = axis_index();

        StandardStepper::init();

        Modbus::init(this);

        //config_message();
    }

    void EncodedStepper::onSetCurrentAngle(int32_t angle)
    {
        _current_angle = angle;
    }

    //Feedback. Check if motor is in position.
    void EncodedStepper::update()
    {
        int32_t angle_pos = 0;

        //TODO: angle offset after homing

        if (sys.state == State::Idle)
        {
            float mpos = steps_to_mpos(get_axis_motor_steps(_axis_index), _axis_index);  // get the axis machine position in mm
            float wco = gc_state.coord_system[_axis_index] + gc_state.coord_offset[_axis_index]; //get the work offset
            float wpos = mpos - wco; //compute current work position

            if (_angle_step_ratio > 0)
                angle_pos = _current_angle / _angle_step_ratio;
            else
                angle_pos = _current_angle;

            if (_homing_angle_offset == INT32_MIN)
            {
                _homing_angle_offset = angle_pos;
            }

            angle_pos -= _homing_angle_offset;

            //ask the encoder where the motor really is
            log_info("<M_ENC"<< _axis_index << ":" << wpos << "," << angle_pos << ">");
        }
        else if (sys.state == State::Homing)
        {
            _homing_angle_offset = INT32_MIN;
        }
    }

    // Configuration handlers:
    void EncodedStepper::validate() const {
        StandardStepper::validate();
        Assert(Modbus::_uart != nullptr, "EncodedStepper: missing UART configuration");
        Assert(_encoder_modbus_id > 0, "EncodedStepper: Invalid modbus id");
    }

    void EncodedStepper::group(Configuration::HandlerBase& handler) {
        StandardStepper::group(handler);

        handler.section("uart", Modbus::_uart);
        handler.item("modbus_id", _encoder_modbus_id, 0, 247); // per https://modbus.org/docs/PI_MBUS_300.pdf
        handler.item("angle_step_ratio", _angle_step_ratio, 0);
    }

    void EncodedStepper::afterParse() {
        if (_encoder_modbus_id > 0) {
            log_info("Using EncodedStepper Driver: modbus_id=" << _encoder_modbus_id);
        }
        if (_angle_step_ratio > 0) {
            log_info("EncodedStepper ratio: " << _angle_step_ratio);
        }
    }

    // Name of the configurable. Must match the name registered in the cpp file.
    const char* EncodedStepper::name() const { return "EncodedStepper"; }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<EncodedStepper> registration("EncodedStepper");
    }
}
