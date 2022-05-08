// Copyright (c) 2020 -	Bart Dring
// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Spindle.h"
#include "../Types.h"

#include "../Uart.h"

#define DEBUG_RoboATCSpindle
#define DEBUG_RoboATCSpindle_ALL

namespace Spindles {
    extern Uart _uart;

    class RoboATCSpindle : public Spindle {
    private:
        static const int RoboATCSpindle_RS485_MAX_MSG_SIZE = 16;  // more than enough for a modbus message
        static const int MAX_RETRIES            = 2;   // otherwise the spindle is marked 'unresponsive'

        void set_mode(SpindleState mode, bool critical);
        bool get_pins_and_settings();

        int32_t  _current_dev_speed   = -1;
        uint32_t _last_speed          = 0;
        Percent  _last_override_value = 100;  // no override is 100 percent

        static QueueHandle_t vfd_cmd_queue;
        static TaskHandle_t  vfd_cmdTaskHandle;
        static void          vfd_cmd_task(void* pvParameters);

        static uint16_t ModRTU_CRC(uint8_t* buf, int msg_len);
        enum RoboATCSpindleactionType : uint8_t { actionSetSpeed, actionSetMode };
        struct RoboATCSpindleaction {
            RoboATCSpindleactionType action;
            bool          critical;
            uint32_t      arg;
        };

    protected:
        struct ModbusCommand {
            bool critical;  // TODO SdB: change into `uint8_t critical : 1;`: We want more flags...

            uint8_t tx_length;
            uint8_t rx_length;
            uint8_t msg[RoboATCSpindle_RS485_MAX_MSG_SIZE];
        };

    private:
        bool prepareSetModeCommand(SpindleState mode, ModbusCommand& data);
        bool prepareSetSpeedCommand(uint32_t speed, ModbusCommand& data);

        static void reportParsingErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length);
        static void reportCmdErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length, uint8_t id);

    protected:
        uint16_t _minFrequency = 0;   
        uint16_t _maxFrequency = 4000;  // H100 works with frequencies scaled by 10.

        void updateRPM();
        // Commands:
        void direction_command(SpindleState mode, ModbusCommand& data);
        void set_speed_command(uint32_t rpm, ModbusCommand& data);

        // Commands that return the status. Returns nullptr if unavailable by this RoboATCSpindle (default):
        using response_parser = bool (*)(const uint8_t* response, RoboATCSpindle* spindle);

        virtual response_parser initialization_sequence(int index, ModbusCommand& data) { return nullptr; }
        response_parser get_current_speed(ModbusCommand& data);
        virtual response_parser get_current_direction(ModbusCommand& data) { return nullptr; }
        virtual response_parser get_status_ok(ModbusCommand& data) { return nullptr; }
        virtual bool            safety_polling() const { return true; }

        // The constructor sets these
        Uart*   _uart      = nullptr;
        uint8_t _modbus_id = 1;

        void setSpeed(uint32_t dev_speed);

        volatile bool _syncing;

    public:
        RoboATCSpindle() {}
        RoboATCSpindle(const RoboATCSpindle&) = delete;
        RoboATCSpindle(RoboATCSpindle&&)      = delete;
        RoboATCSpindle& operator=(const RoboATCSpindle&) = delete;
        RoboATCSpindle& operator=(RoboATCSpindle&&) = delete;

        void init();
        void config_message();
        void setState(SpindleState state, SpindleSpeed speed);
        void setSpeedfromISR(uint32_t dev_speed) override;

        volatile uint32_t _sync_dev_speed;
        SpindleSpeed      _slop;

        // Configuration handlers:
        void validate() const override {
            Spindle::validate();
            Assert(_uart != nullptr, "RoboATCSpindle: missing UART configuration");
        }

        void group(Configuration::HandlerBase& handler) override {
            handler.section("uart", _uart);
            handler.item("modbus_id", _modbus_id, 0, 247); // per https://modbus.org/docs/PI_MBUS_300.pdf

            Spindle::group(handler);
        }

        // Name of the configurable. Must match the name registered in the cpp file.
        const char* name() const override { return "RoboATCSpindle"; }

        virtual ~RoboATCSpindle() {}
    };
}
