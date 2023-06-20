// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    EncodedStepper.cpp -- EncodedStepper type stepper drivers
*/

#include "../Gcode.h"
#include "StandardStepper.h"
#include "../Uart.h"

extern parser_state_t gc_state;

namespace MotorDrivers {
    extern Uart _uart;

    class EncodedStepper : public StandardStepper {
    protected:
        static const int RS485_MAX_MSG_SIZE     = 16;  // more than enough for a modbus message
        static const int MAX_RETRIES            = 2;   // otherwise the spindle is marked 'unresponsive'

        Uart*   _uart      = nullptr;
        uint8_t _encoder_modbus_id;
        uint32_t _angle_step_ratio;
        int32_t _current_angle = -1;
        int32_t _homing_angle_offset = 0;
        int _axis_index;

        struct ModbusCommand {
            bool critical;  // TODO SdB: change into `uint8_t critical : 1;`: We want more flags...

            uint8_t tx_length;
            uint8_t rx_length;
            uint8_t msg[RS485_MAX_MSG_SIZE];
        };

        using response_parser = bool (*)(const uint8_t* response, EncodedStepper* spindle);

        response_parser get_encoder_pos(ModbusCommand& data);

    private:
        static QueueHandle_t uart_cmd_queue;
        static TaskHandle_t  uart_cmdTaskHandle;
        static void          uart_cmd_task(void* pvParameters);

        static void reportParsingErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length);
        static void reportCmdErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length, uint8_t id);

        static uint16_t ModRTU_CRC(uint8_t* buf, int msg_len);

    public:
        EncodedStepper() = default;

        void init() override;

        void update() override;

        // Configuration handlers:
        void validate() const override;
        void group(Configuration::HandlerBase& handler) override;

        void afterParse() override;

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override;

        ~EncodedStepper() = default;
    };
}
