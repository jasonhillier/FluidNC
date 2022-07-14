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
        _encoder_modbus_id = 0;
        _axis_index = axis_index();

        StandardStepper::init();

        _uart->begin();

        if (_uart->setHalfDuplex()) {
            log_info("EncodedStepper: RS485 UART set half duplex failed");
            return;
        }
    }

    //Feedback. Check if motor is in position.
    void EncodedStepper::update()
    {
        float mpos = steps_to_mpos(motor_steps[_axis_index], _axis_index);  // get the axis machine position in mm
        //servo_pos  = mpos;                                                  // determine the current work position

        //ask the encoder where the motor really is
    }

    EncodedStepper::response_parser EncodedStepper::get_encoder_pos(ModbusCommand& data)
    {
        data.tx_length = 6;
        data.rx_length = 5;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x03;
        data.msg[2] = 0x00;
        data.msg[3] = 0x06; //angle
        data.msg[4] = 0x00;
        data.msg[5] = 0x01;

        return [](const uint8_t* response, MotorDrivers::EncodedStepper* vfd) -> bool {
            // 01 04 04 [freq 16] [set freq 16] [crc16]
            uint16_t angle = (uint16_t(response[3]) << 8) | uint16_t(response[4]);

            log_info("GOT ANGLE " << angle);

            // Store speed for synchronization
            //vfd->_sync_dev_speed = frequency;
            return true;
        };
    }

    // Calculate the CRC on all of the byte except the last 2
    // It then added the CRC to those last 2 bytes
    // full_msg_len This is the length of the message including the 2 crc bytes
    // Source: https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/
    uint16_t EncodedStepper::ModRTU_CRC(uint8_t* buf, int msg_len) {
        uint16_t crc = 0xFFFF;
        for (int pos = 0; pos < msg_len; pos++) {
            crc ^= uint16_t(buf[pos]);  // XOR byte into least sig. byte of crc.

            for (int i = 8; i != 0; i--) {  // Loop over each bit
                if ((crc & 0x0001) != 0) {  // If the LSB is set
                    crc >>= 1;              // Shift right and XOR 0xA001
                    crc ^= 0xA001;
                } else {        // Else LSB is not set
                    crc >>= 1;  // Just shift right
                }
            }
        }

        return crc;
    }

    // Configuration handlers:
    void EncodedStepper::validate() const {
        StandardStepper::validate();
        Assert(_uart != nullptr, "EncodedStepper: missing UART configuration");
    }

    void EncodedStepper::group(Configuration::HandlerBase& handler) {
        StandardStepper::group(handler);

        handler.section("uart", _uart);
        handler.item("modbus_id", _encoder_modbus_id, 0, 247); // per https://modbus.org/docs/PI_MBUS_300.pdf
    }

    void EncodedStepper::afterParse() {
        if (_encoder_modbus_id > 0) {
            log_info("Using EncodedStepper Mode");
        }
    }

    // Name of the configurable. Must match the name registered in the cpp file.
    const char* EncodedStepper::name() const { return "EncodedStepper"; }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<EncodedStepper> registration("EncodedStepper");
    }
}
