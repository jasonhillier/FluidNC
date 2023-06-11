// Copyright (c) 2021 - Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    EncodedStepper.cpp -- EncodedStepper type stepper drivers
    TODO: we may need a PID controller here
    TODO: make it so we can attach different encoders.
*/

#include "EncodedStepper.h"
#include "../System.h"  // mpos_to_steps() etc
#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>

const int        RS485_QUEUE_SIZE = 10; 
const int        RS485_POLL_RATE  = 2000;                                    // in milliseconds between commands
const int        RESPONSE_WAIT_MS     = 1000;                                   // how long to wait for a response
const TickType_t response_ticks       = RESPONSE_WAIT_MS / portTICK_PERIOD_MS;  // in milliseconds between commands

namespace MotorDrivers {
    QueueHandle_t EncodedStepper::uart_cmd_queue     = nullptr;
    TaskHandle_t  EncodedStepper::uart_cmdTaskHandle = nullptr;

    void EncodedStepper::reportParsingErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length) {
//#ifdef DEBUG_RoboATCSpindle
        hex_msg(cmd.msg, "e RS485 Tx: ", cmd.tx_length);
        hex_msg(rx_message, "e RS485 Rx: ", read_length);
//#endif
    }
    void EncodedStepper::reportCmdErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length, uint8_t id) {
//#ifdef DEBUG_RoboATCSpindle
        hex_msg(cmd.msg, "c RS485 Tx: ", cmd.tx_length);
        hex_msg(rx_message, "c RS485 Rx: ", read_length);

        if (read_length != 0) {
            if (rx_message[0] != id) {
                log_info("RS485 received message from other modbus device");
            } else if (read_length != cmd.rx_length) {
                log_info("RS485 received message of unexpected length; expected:" << int(cmd.rx_length) << " got:" << int(read_length));
            } else {
                log_info("RS485 CRC check failed");
            }
        } else {
            log_info("RS485 No response");
        }
//#endif
    }
    
    void EncodedStepper::init() {
        _axis_index = axis_index();

        StandardStepper::init();

        _uart->begin();

        if (_uart->setHalfDuplex()) {
            log_info("EncodedStepper: RS485 UART set half duplex failed");
            return;
        }

        // Initialization is complete, so now it's okay to run the queue task:
        if (!uart_cmd_queue) {  // init can happen many times, we only want to start one task
            uart_cmd_queue = xQueueCreate(RS485_QUEUE_SIZE, sizeof(int));
            xTaskCreatePinnedToCore(uart_cmd_task,         // task
                                    "uart_cmdTaskHandle",  // name for task
                                    2048,                 // size of task stack
                                    this,                 // parameters
                                    1,                    // priority
                                    &uart_cmdTaskHandle,
                                    SUPPORT_TASK_CORE  // core
            );
        }

        //config_message();
    }

    // The communications task
    void EncodedStepper::uart_cmd_task(void* pvParameters)
    {
        static bool unresponsive = false;  // to pop off a message once each time it becomes unresponsive
        static int  pollidx      = -1;

        EncodedStepper*          instance = static_cast<EncodedStepper*>(pvParameters);
        auto& uart = *instance->_uart;
        ModbusCommand next_cmd;
        uint8_t       rx_message[RS485_MAX_MSG_SIZE];

        for (; true; vTaskDelay(RS485_POLL_RATE / portTICK_PERIOD_MS)) {
            std::atomic_thread_fence(std::memory_order::memory_order_seq_cst);  // read fence for settings
            response_parser parser = nullptr;

            parser = instance->get_encoder_pos(next_cmd);

            // At this point next_cmd has been filled with a command block
            {
                // Fill in the fields that are the same for all protocol variants
                next_cmd.msg[0] = instance->_encoder_modbus_id;

                // Grabbed the command. Add the CRC16 checksum:
                auto crc16                         = ModRTU_CRC(next_cmd.msg, next_cmd.tx_length);
                next_cmd.msg[next_cmd.tx_length++] = (crc16 & 0xFF);
                next_cmd.msg[next_cmd.tx_length++] = (crc16 & 0xFF00) >> 8;
                next_cmd.rx_length += 2;

//#ifdef DEBUG_RoboATCSpindle_ALL
                if (parser == nullptr) {
                    hex_msg(next_cmd.msg, "RS485 Tx: ", next_cmd.tx_length);
                }
//#endif
            }

            // Assume for the worst, and retry...
            int retry_count = 0;
            for (; retry_count < MAX_RETRIES; ++retry_count) {
                // Flush the UART and write the data:
                uart.flush();
                uart.write(next_cmd.msg, next_cmd.tx_length);
                uart.flushTxTimed(response_ticks);

                // Read the response
                size_t read_length  = 0;
                size_t current_read = uart.readBytes(rx_message, next_cmd.rx_length, response_ticks);
                read_length += current_read;

                //log_info("read " << current_read);

                while (read_length < next_cmd.rx_length && current_read > 0) {
                    // Try to read more; we're not there yet...
                    current_read = uart.readBytes(rx_message + read_length, next_cmd.rx_length - read_length, response_ticks);
                    read_length += current_read;
                }

                // Generate crc16 for the response:
                auto crc16response = ModRTU_CRC(rx_message, next_cmd.rx_length - 2);

                if (read_length == next_cmd.rx_length &&                             // check expected length
                    rx_message[0] == instance->_encoder_modbus_id &&                         // check address
                    rx_message[read_length - 1] == (crc16response & 0xFF00) >> 8 &&  // check CRC byte 1
                    rx_message[read_length - 2] == (crc16response & 0xFF)) {         // check CRC byte 1

                    // Success
                    unresponsive = false;
                    retry_count  = MAX_RETRIES + 1;  // stop retry'ing

                    // Should we parse this?
                    if (parser != nullptr) {
                        if (parser(rx_message, instance)) {
                            // If we're initializing, move to the next initialization command:
                            if (pollidx < 0) {
                                --pollidx;
                            }
                        } else {
                            // Parsing failed
                            reportParsingErrors(next_cmd, rx_message, read_length);

                            // If we were initializing, move back to where we started.
                            unresponsive = true;
                            pollidx      = -1;  // Re-initializing the RoboATCSpindle seems like a plan
                            log_info("EncoderStepper RS485 did not give a satisfying response");
                        }
                    }
                } else {
                    reportCmdErrors(next_cmd, rx_message, read_length, instance->_encoder_modbus_id);

                    // Wait a bit before we retry. Set the delay to poll-rate. Not sure
                    // if we should use a different value...
                    vTaskDelay(RS485_POLL_RATE / portTICK_PERIOD_MS);

#ifdef DEBUG_TASK_STACK
                    static UBaseType_t uxHighWaterMark = 0;
                    reportTaskStackSize(uxHighWaterMark);
#endif
                }
            }

            //just inject this for now TODO: fix it right
            instance->update();
        }
    }

    //Feedback. Check if motor is in position.
    void EncodedStepper::update()
    {
        if (sys.state == State::Idle)
        {
            float mpos = steps_to_mpos(get_axis_motor_steps(_axis_index), _axis_index);  // get the axis machine position in mm
            float wco = gc_state.coord_system[_axis_index] + gc_state.coord_offset[_axis_index]; //get the work offset
            float wpos = mpos - wco; //compute current work position

            //ask the encoder where the motor really is
            log_info("WORK POS to ANGLE: "<< wpos << " -> " << _current_angle);
        }
    }

    EncodedStepper::response_parser EncodedStepper::get_encoder_pos(ModbusCommand& data)
    {
        data.tx_length = 6;
        data.rx_length = 5;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x03;
        data.msg[2] = 0x00;
        data.msg[3] = 0x04; //total (incremented) angle
        data.msg[4] = 0x00;
        data.msg[5] = 0x01;

        return [](const uint8_t* response, MotorDrivers::EncodedStepper* instance) -> bool {
            // 01 04 04 [freq 16] [set freq 16] [crc16]
            uint16_t angle = (uint16_t(response[3]) << 8) | uint16_t(response[4]);

            //log_info("GOT ANGLE " << angle);
            instance->_current_angle = angle;

            // Store speed for synchronization
            //instance->_sync_dev_speed = frequency;
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
        Assert(_encoder_modbus_id > 0, "EncodedStepper: Invalid modbus id");
    }

    void EncodedStepper::group(Configuration::HandlerBase& handler) {
        StandardStepper::group(handler);

        handler.section("uart", _uart);
        handler.item("modbus_id", _encoder_modbus_id, 0, 247); // per https://modbus.org/docs/PI_MBUS_300.pdf
        handler.item("base_angle", _base_angle, -1, 365);
    }

    void EncodedStepper::afterParse() {
        if (_encoder_modbus_id > 0) {
            log_info("Using EncodedStepper Mode");
        }
        if (_base_angle > 0) {
            log_info("Monitoring motor angle from baseline: " << _base_angle);
        }
    }

    // Name of the configurable. Must match the name registered in the cpp file.
    const char* EncodedStepper::name() const { return "EncodedStepper"; }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<EncodedStepper> registration("EncodedStepper");
    }
}
