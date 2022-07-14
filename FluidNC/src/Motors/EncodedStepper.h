// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    EncodedStepper.cpp -- EncodedStepper type stepper drivers
*/

#include "StandardStepper.h"
#include "../Uart.h"

namespace MotorDrivers {
    extern Uart _uart;

    class EncodedStepper : public StandardStepper {
    private:
        static const int RS485_MAX_MSG_SIZE     = 16;  // more than enough for a modbus message
        static const int MAX_RETRIES            = 2;   // otherwise the spindle is marked 'unresponsive'

        //static QueueHandle_t uart_cmd_queue;
        static TaskHandle_t  uart_cmdTaskHandle;
        static void          uart_cmd_task(void* pvParameters);

        struct ModbusCommand {
            bool critical;  // TODO SdB: change into `uint8_t critical : 1;`: We want more flags...

            uint8_t tx_length;
            uint8_t rx_length;
            uint8_t msg[RS485_MAX_MSG_SIZE];
        };

        static uint16_t ModRTU_CRC(uint8_t* buf, int msg_len);

    protected:
        Uart*   _uart      = nullptr;
        uint8_t _encoder_modbus_id;
        int _axis_index;

        using response_parser = bool (*)(const uint8_t* response, EncodedStepper* spindle);

        response_parser get_encoder_pos(ModbusCommand& data);

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
