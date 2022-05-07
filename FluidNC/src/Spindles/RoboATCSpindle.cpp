// Copyright (c) 2020 -	Bart Dring
// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    This is for a RoboATCSpindle based spindles via RS485 Modbus. The details of the 
    RoboATCSpindle protocol heavily depend on the RoboATCSpindle in question here. We have some 
    implementations, but if yours is not here, the place to start is the 
    manual. This RoboATCSpindle class implements the modbus functionality.

                         WARNING!!!!
    RoboATCSpindles are very dangerous. They have high voltages and are very powerful
    Remove power before changing bits.

    TODO:
      - We can report spindle_state and rpm better with RoboATCSpindle's that support 
        either mode, register RPM or actual RPM.
      - Destructor should break down the task.

*/
#include "RoboATCSpindle.h"

#include "../Machine/MachineConfig.h"
#include "../MotionControl.h"  // mc_reset
#include "../Protocol.h"       // rtAlarm
#include "../Report.h"         // hex message

#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>

const int        RoboATCSpindle_RS485_BUF_SIZE   = 127;
const int        RoboATCSpindle_RS485_QUEUE_SIZE = 10;                                     // number of commands that can be queued up.
const int        RESPONSE_WAIT_MS     = 1000;                                   // how long to wait for a response
const int        RoboATCSpindle_RS485_POLL_RATE  = 250;                                    // in milliseconds between commands
const TickType_t response_ticks       = RESPONSE_WAIT_MS / portTICK_PERIOD_MS;  // in milliseconds between commands

namespace Spindles {
    QueueHandle_t RoboATCSpindle::vfd_cmd_queue     = nullptr;
    TaskHandle_t  RoboATCSpindle::vfd_cmdTaskHandle = nullptr;

    void RoboATCSpindle::reportParsingErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length) {
#ifdef DEBUG_RoboATCSpindle
        hex_msg(cmd.msg, "RS485 Tx: ", cmd.tx_length);
        hex_msg(rx_message, "RS485 Rx: ", read_length);
#endif
    }
    void RoboATCSpindle::reportCmdErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length, uint8_t id) {
#ifdef DEBUG_RoboATCSpindle
        hex_msg(cmd.msg, "RS485 Tx: ", cmd.tx_length);
        hex_msg(rx_message, "RS485 Rx: ", read_length);

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
#endif
    }

    void IRAM_ATTR RoboATCSpindle::set_speed_command(uint32_t dev_speed, ModbusCommand& data) {
        data.tx_length = 6;
        data.rx_length = 6;

        if (dev_speed != 0 && (dev_speed < _minFrequency || dev_speed > _maxFrequency)) {
            log_warn(name() << " requested freq " << (dev_speed) << " is outside of range (" << _minFrequency << "," << _maxFrequency << ")");
        }

#ifdef DEBUG_RoboATCSpindle
        log_debug("Setting VFD dev_speed to " << dev_speed);
#endif

        //[01] [06] [0201] [07D0] Set frequency to [07D0] = 200.0 Hz. (2000 is written!)

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x06;  // Set register command
        data.msg[2] = 0x00; //0x02;
        data.msg[3] = 0x0B; //11
        //data.msg[4] = dev_speed >> 8; - BC 11/24/21
        data.msg[4] = dev_speed >> 8;
        data.msg[5] = dev_speed & 0xFF;
    }


    // The communications task
    void RoboATCSpindle::vfd_cmd_task(void* pvParameters) {
        static bool unresponsive = false;  // to pop off a message once each time it becomes unresponsive
        static int  pollidx      = -1;

        RoboATCSpindle*          instance = static_cast<RoboATCSpindle*>(pvParameters);
        auto&         uart     = *instance->_uart;
        ModbusCommand next_cmd;
        uint8_t       rx_message[RoboATCSpindle_RS485_MAX_MSG_SIZE];
        bool          safetyPollingEnabled = instance->safety_polling();

        for (; true; vTaskDelay(RoboATCSpindle_RS485_POLL_RATE / portTICK_PERIOD_MS)) {
            std::atomic_thread_fence(std::memory_order::memory_order_seq_cst);  // read fence for settings
            response_parser parser = nullptr;

            // First check if we should ask the RoboATCSpindle for the speed parameters as part of the initialization.
            if (pollidx < 0 && (parser = instance->initialization_sequence(pollidx, next_cmd)) != nullptr) {
            } else {
                pollidx = 1;  // Done with initialization. Main sequence.
            }
            next_cmd.critical = false;

            RoboATCSpindleaction action;
            if (parser == nullptr) {
                // If we don't have a parser, the queue goes first.
                if (xQueueReceive(vfd_cmd_queue, &action, 0)) {
                    switch (action.action) {
                        case actionSetSpeed:
                            if (!instance->prepareSetSpeedCommand(action.arg, next_cmd)) {
                                // prepareSetSpeedCommand() can return false if the speed
                                // change is unnecessary - already at that speed.
                                // In that case we just discard the command.
                                continue;  // main loop
                            }
                            next_cmd.critical = action.critical;
                            break;
                        case actionSetMode:
                            log_debug("vfd_cmd_task mode:" << action.action);
                            if (!instance->prepareSetModeCommand(SpindleState(action.arg), next_cmd)) {
                                continue;  // main loop
                            }
                            next_cmd.critical = action.critical;
                            break;
                    }
                } else {
                    // We do not have a parser and there is nothing in the queue, so we cycle
                    // through the set of periodic queries.

                    // We poll in a cycle. Note that the switch will fall through unless we encounter a hit.
                    // The weakest form here is 'get_status_ok' which should be implemented if the rest fails.
                    if (instance->_syncing) {
                        parser = instance->get_current_speed(next_cmd);
                    } else if (safetyPollingEnabled) {
                        switch (pollidx) {
                            case 1:
                                parser = instance->get_current_speed(next_cmd);
                                if (parser) {
                                    pollidx = 2;
                                    break;
                                }
                                // fall through if get_current_speed did not return a parser
                            case 2:
                                parser = instance->get_current_direction(next_cmd);
                                if (parser) {
                                    pollidx = 3;
                                    break;
                                }
                                // fall through if get_current_direction did not return a parser
                            case 3:
                            default:
                                parser  = instance->get_status_ok(next_cmd);
                                pollidx = 1;

                                // we could complete this in case parser == nullptr with some ifs, but let's
                                // just keep it easy and wait an iteration.
                                break;
                        }
                    }

                    // If we have no parser, that means get_status_ok is not implemented (and we have
                    // nothing resting in our queue). Let's fall back on a simple continue.
                    if (parser == nullptr) {
                        continue;  // main loop
                    }
                }
            }

            // At this point next_cmd has been filled with a command block
            {
                // Fill in the fields that are the same for all protocol variants
                next_cmd.msg[0] = instance->_modbus_id;

                // Grabbed the command. Add the CRC16 checksum:
                auto crc16                         = ModRTU_CRC(next_cmd.msg, next_cmd.tx_length);
                next_cmd.msg[next_cmd.tx_length++] = (crc16 & 0xFF);
                next_cmd.msg[next_cmd.tx_length++] = (crc16 & 0xFF00) >> 8;
                next_cmd.rx_length += 2;

#ifdef DEBUG_RoboATCSpindle_ALL
                if (parser == nullptr) {
                    hex_msg(next_cmd.msg, "RS485 Tx: ", next_cmd.tx_length);
                }
#endif
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

                // Apparently some Huanyang report modbus errors in the correct way, and the rest not. Sigh.
                // Let's just check for the condition, and truncate the first byte.
                if (read_length > 0 && instance->_modbus_id != 0 && rx_message[0] == 0) {
                    memmove(rx_message + 1, rx_message, read_length - 1);
                }

                while (read_length < next_cmd.rx_length && current_read > 0) {
                    // Try to read more; we're not there yet...
                    current_read = uart.readBytes(rx_message + read_length, next_cmd.rx_length - read_length, response_ticks);
                    read_length += current_read;
                }

                // Generate crc16 for the response:
                auto crc16response = ModRTU_CRC(rx_message, next_cmd.rx_length - 2);

                if (read_length == next_cmd.rx_length &&                             // check expected length
                    rx_message[0] == instance->_modbus_id &&                         // check address
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
                            log_info("Spindle RS485 did not give a satisfying response");
                        }
                    }
                } else {
                    reportCmdErrors(next_cmd, rx_message, read_length, instance->_modbus_id);

                    // Wait a bit before we retry. Set the delay to poll-rate. Not sure
                    // if we should use a different value...
                    vTaskDelay(RoboATCSpindle_RS485_POLL_RATE / portTICK_PERIOD_MS);

#ifdef DEBUG_TASK_STACK
                    static UBaseType_t uxHighWaterMark = 0;
                    reportTaskStackSize(uxHighWaterMark);
#endif
                }
            }

            /*
            if (retry_count == MAX_RETRIES) {
                if (!unresponsive) {
                    log_info("RoboATCSpindle RS485 Unresponsive");
                    unresponsive = true;
                    pollidx      = -1;
                }
                if (next_cmd.critical) {
                    log_error("Critical RoboATCSpindle RS485 Unresponsive");
                    mc_reset();
                    rtAlarm = ExecAlarm::SpindleControl;
                }
            }
            */
        }
    }

    // ================== Class methods ==================================

    void RoboATCSpindle::init() {
        _sync_dev_speed = 0;
        _syncing        = false;

        // fail if required items are not defined
        // get_pins_and_settings reports detailed error messages
        if (!get_pins_and_settings()) {
            return;
        }

        _uart->begin();

        if (_uart->setHalfDuplex()) {
            log_info("RoboATCSpindle: RS485 UART set half duplex failed");
            return;
        }

        // These RoboATCSpindles are always reversable, but most can be set via the operator panel
        // to only allow one direction.  In principle we could check that setting and
        // automatically set is_reversable.
        is_reversable = true;

        _current_state = SpindleState::Disable;

        // Initialization is complete, so now it's okay to run the queue task:
        if (!vfd_cmd_queue) {  // init can happen many times, we only want to start one task
            vfd_cmd_queue = xQueueCreate(RoboATCSpindle_RS485_QUEUE_SIZE, sizeof(RoboATCSpindleaction));
            xTaskCreatePinnedToCore(vfd_cmd_task,         // task
                                    "vfd_cmdTaskHandle",  // name for task
                                    2048,                 // size of task stack
                                    this,                 // parameters
                                    1,                    // priority
                                    &vfd_cmdTaskHandle,
                                    SUPPORT_TASK_CORE  // core
            );
        }

        config_message();

        set_mode(SpindleState::Disable, true);
    }

    // Checks for all the required pin definitions
    // It returns a message for each missing pin
    // Returns true if all pins are defined.
    bool RoboATCSpindle::get_pins_and_settings() { return true; }

    void RoboATCSpindle::config_message() { _uart->config_message(name(), " Spindle "); }

    void RoboATCSpindle::setState(SpindleState state, SpindleSpeed speed) {
        log_info("RoboATCSpindle setState:" << uint8_t(state) << " SpindleSpeed:" << speed);
        if (sys.abort) {
            return;  // Block during abort.
        }

        bool critical = (sys.state == State::Cycle || state != SpindleState::Disable);

        uint32_t dev_speed = mapSpeed(speed);
        log_debug("RPM:" << speed << " mapped to device units:" << dev_speed);

        if (_current_state != state) {
            // Changing state
            set_mode(state, critical);  // critical if we are in a job

            setSpeed(dev_speed);
        } else {
            // Not changing state
            if (_current_dev_speed != dev_speed) {
                // Changing speed
                setSpeed(dev_speed);
            }
        }
        if (use_delay_settings()) {
            spindleDelay(state, speed);
        } else {
            // _sync_dev_speed is set by a callback that handles
            // responses from periodic get_current_speed() requests.
            // It changes as the actual speed ramps toward the target.

            _syncing = true;  // poll for speed

            auto minSpeedAllowed = dev_speed > _slop ? (dev_speed - _slop) : 0;
            auto maxSpeedAllowed = dev_speed + _slop;

            int       unchanged = 0;
            const int limit     = 20;  // 20 * 0.5s = 10 sec
            auto      last      = _sync_dev_speed;

            while ((_last_override_value == sys.spindle_speed_ovr) &&  // skip if the override changes
                   ((_sync_dev_speed < minSpeedAllowed || _sync_dev_speed > maxSpeedAllowed) && unchanged < limit)) {
#ifdef DEBUG_RoboATCSpindle
                log_debug("Syncing speed. Requested: " << int(dev_speed) << " current:" << int(_sync_dev_speed));
#endif
                // if (!mc_dwell(500)) {
                //     // Something happened while we were dwelling, like a safety door.
                //     unchanged = limit;
                //     last      = _sync_dev_speed;
                //     break;
                // }
                delay(500);

                // unchanged counts the number of consecutive times that we see the same speed
                unchanged = (_sync_dev_speed == last) ? unchanged + 1 : 0;
                last      = _sync_dev_speed;
            }
            _last_override_value = sys.spindle_speed_ovr;

#ifdef DEBUG_RoboATCSpindle
            log_debug("Synced speed. Requested:" << int(dev_speed) << " current:" << int(_sync_dev_speed));
#endif

            if (unchanged == limit) {
                log_error(name() << " spindle did not reach device units " << dev_speed << ". Reported value is " << _sync_dev_speed);
                mc_reset();
                rtAlarm = ExecAlarm::SpindleControl;
            }

            _syncing = false;
            // spindleDelay() sets these when it is used
            _current_state = state;
            _current_speed = speed;
        }
        //        }
    }

    bool RoboATCSpindle::prepareSetModeCommand(SpindleState mode, ModbusCommand& data) {
        // Do variant-specific command preparation
        direction_command(mode, data);

        if (mode == SpindleState::Disable) {
            if (!xQueueReset(vfd_cmd_queue)) {
                log_info(name() << " spindle off, queue could not be reset");
            }
        }

        _current_state = mode;
        return true;
    }

    void RoboATCSpindle::set_mode(SpindleState mode, bool critical) {
        _last_override_value = sys.spindle_speed_ovr;  // sync these on mode changes
        if (vfd_cmd_queue) {
            RoboATCSpindleaction action;
            action.action   = actionSetMode;
            action.arg      = uint32_t(mode);
            action.critical = critical;
            if (xQueueSend(vfd_cmd_queue, &action, 0) != pdTRUE) {
                log_info("RoboATCSpindle Queue Full");
            }
        }
    }

    void IRAM_ATTR RoboATCSpindle::setSpeedfromISR(uint32_t dev_speed) {
        if (_current_dev_speed == dev_speed || _last_speed == dev_speed) {
            return;
        }

        _last_speed = dev_speed;

        if (vfd_cmd_queue) {
            RoboATCSpindleaction action;
            action.action   = actionSetSpeed;
            action.arg      = dev_speed;
            action.critical = (dev_speed == 0);
            if (xQueueSendFromISR(vfd_cmd_queue, &action, 0) != pdTRUE) {
                log_info("RoboATCSpindle Queue Full");
            }
        }
    }

    void RoboATCSpindle::setSpeed(uint32_t dev_speed) {
        if (vfd_cmd_queue) {
            RoboATCSpindleaction action;
            action.action   = actionSetSpeed;
            action.arg      = dev_speed;
            action.critical = dev_speed == 0;
            if (xQueueSend(vfd_cmd_queue, &action, 0) != pdTRUE) {
                log_info("RoboATCSpindle Queue Full");
            }
        }
    }

    bool RoboATCSpindle::prepareSetSpeedCommand(uint32_t speed, ModbusCommand& data) {
        log_debug("prep speed " << speed << " curr " << _current_dev_speed);
        if (speed == _current_dev_speed) {  // prevent setting same speed twice
            return false;
        }
        _current_dev_speed = speed;

#ifdef DEBUG_RoboATCSpindle_ALL
        log_debug("Setting spindle speed to:" << int(speed));
#endif
        // Do variant-specific command preparation
        set_speed_command(speed, data);

        // Sometimes sync_dev_speed is retained between different set_speed_command's. We don't want that - we want
        // spindle sync to kick in after we set the speed. This forces that.
        _sync_dev_speed = UINT32_MAX;

        return true;
    }

    // Calculate the CRC on all of the byte except the last 2
    // It then added the CRC to those last 2 bytes
    // full_msg_len This is the length of the message including the 2 crc bytes
    // Source: https://ctlsys.com/support/how_to_compute_the_modbus_rtu_message_crc/
    uint16_t RoboATCSpindle::ModRTU_CRC(uint8_t* buf, int msg_len) {
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
}
