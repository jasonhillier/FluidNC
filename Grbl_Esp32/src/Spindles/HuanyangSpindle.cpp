/*
    HuanyangSpindle.cpp

    This is for a Huanyang VFD based spindle via RS485 Modbus.
    Sorry for the lengthy comments, but finding the details on this
    VFD was a PITA. I am just trying to help the next person.

    Part of Grbl_ESP32
    2020 -  Bart Dring
    2020 -  Stefan de Bruijn

    Grbl is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

                         WARNING!!!!
    VFDs are very dangerous. They have high voltages and are very powerful
    Remove power before changing bits.

    ==============================================================================

    If a user changes state or RPM level, the command to do that is sent. If
    the command is not responded to a message is sent to serial that there was
    a timeout. If the Grbl is in a critical state, an alarm will be generated and
    the machine stopped.

    If there are no commands to execute, various status items will be polled. If there
    is no response, it will behave as described above. It will stop any running jobs with
    an alarm.

    ===============================================================================

    Protocol Details

    A lot of good information about the details of all these parameters and how they should 
    be setup can be found on this page: 
    https://community.carbide3d.com/t/vfd-parameters-huanyang-model/15459/7 . 

    Before using spindle, VFD must be setup for RS485 and match your spindle:

    PD004   400  Base frequency as rated on my spindle (default was 50)
    PD005   400  Maximum frequency Hz (Typical for spindles)
    PD011   120  Min Speed (Recommend Aircooled=120 Water=100)
    PD014   10   Acceleration time (Test to optimize)
    PD015   10   Deceleration time (Test to optimize)
    PD023   1    Reverse run enabled
    PD141   220  Spindle max rated voltage
    PD142   3.7  Max current Amps (0.8kw=3.7 1.5kw=7.0, 2.2kw=??)
    PD143   2    Poles most are 2 (I think this is only used for RPM calc from Hz)
    PD144   3000 Max rated motor revolution at 50 Hz => 24000@400Hz = 3000@50HZ

    PD001   2    RS485 Control of run commands
    PD002   2    RS485 Control of operating frequency
    PD163   1    RS485 Address: 1 (Typical. OK to change...see below)
    PD164   1    RS485 Baud rate: 9600 (Typical. OK to change...see below)
    PD165   3    RS485 Mode: RTU, 8N1

    The official documentation of the RS485 is horrible. I had to piece it together from
    a lot of different sources:

    Manuals: https://github.com/RobertOlechowski/Huanyang_VFD/tree/master/Documentations/pdf
    Reference: https://github.com/Smoothieware/Smoothieware/blob/edge/src/modules/tools/spindle/HuanyangSpindleControl.cpp
    Refernece: https://gist.github.com/Bouni/803492ed0aab3f944066
    VFD settings: https://www.hobbytronics.co.za/Content/external/1159/Spindle_Settings.pdf
    Spindle Talker 2 https://github.com/GilchristT/SpindleTalker2/releases
    Python https://github.com/RobertOlechowski/Huanyang_VFD

    =========================================================================

    Commands
    ADDR    CMD     LEN     DATA    CRC
    0x01    0x03    0x01    0x01    0x31 0x88                   Start spindle clockwise
    0x01    0x03    0x01    0x08    0xF1 0x8E                   Stop spindle
    0x01    0x03    0x01    0x11    0x30 0x44                   Start spindle counter-clockwise

    Return values are
    0 = run
    1 = jog
    2 = r/f
    3 = running
    4 = jogging
    5 = r/f
    6 = Braking
    7 = Track start

    ==========================================================================

    Setting RPM
    ADDR    CMD     LEN     DATA        CRC
    0x01    0x05    0x02    0x09 0xC4   0xBF 0x0F               Write Frequency (0x9C4 = 2500 = 25.00HZ)

    Response is same as data sent

    ==========================================================================

    Setting registers
    Addr    Read    Len     Reg     DataH   DataL   CRC     CRC
    0x01    0x01    0x03    5       0x00    0x00    CRC     CRC     //  PD005
    0x01    0x01    0x03    11      0x00    0x00    CRC     CRC     //  PD011
    0x01    0x01    0x03    143     0x00    0x00    CRC     CRC     //  PD143
    0x01    0x01    0x03    144     0x00    0x00    CRC     CRC     //  PD144

    Message is returned with requested value = (DataH * 16) + DataL (see decimal offset above)

    ==========================================================================

    Status registers
    Addr    Read    Len     Reg     DataH   DataL   CRC     CRC
    0x01    0x04    0x03    0x00    0x00    0x00    CRC     CRC     //  Set Frequency * 100 (25Hz = 2500)
    0x01    0x04    0x03    0x01    0x00    0x00    CRC     CRC     //  Ouput Frequency * 100
    0x01    0x04    0x03    0x02    0x00    0x00    CRC     CRC     //  Ouput Amps * 10
    0x01    0x04    0x03    0x03    0x00    0x00    0xF0    0x4E    //  Read RPM (example CRC shown)
    0x01    0x04    0x03    0x0     0x00    0x00    CRC     CRC     //  DC voltage
    0x01    0x04    0x03    0x05    0x00    0x00    CRC     CRC     //  AC voltage
    0x01    0x04    0x03    0x06    0x00    0x00    CRC     CRC     //  Cont
    0x01    0x04    0x03    0x07    0x00    0x00    CRC     CRC     //  VFD Temp
    
    Message is returned with requested value = (DataH * 16) + DataL (see decimal offset above)

    ==========================================================================

    The math:

        PD005   400  Maximum frequency Hz (Typical for spindles)
        PD011   120  Min Speed (Recommend Aircooled=120 Water=100)
        PD143   2    Poles most are 2 (I think this is only used for RPM calc from Hz)
        PD144   3000 Max rated motor revolution at 50 Hz => 24000@400Hz = 3000@50HZ

    During initialization these 4 are pulled from the VFD registers. It then sets min and max RPM
    of the spindle. So:

        MinRPM = PD011 * PD144 / 50 = 120 * 3000 / 50 = 7200 RPM min
        MaxRPM = PD005 * PD144 / 50 = 400 * 3000 / 50 = 24000 RPM max

    If you then set 12000 RPM, it calculates the frequency:

        int targetFrequency = targetRPM * PD005 / MaxRPM = targetRPM * PD005 / (PD005 * PD144 / 50) = 
                              targetRPM * 50 / PD144 = 12000 * 50 / 3000 = 200

    If the frequency is -say- 25 Hz, Huanyang wants us to send 2500 (eg. 25.00 Hz).
*/

#include "HuanyangSpindle.h"

#include <algorithm>  // std::max

namespace Spindles {
    Huanyang::Huanyang() : VFD() {
        // Baud rate is set in the PD164 setting.  If it is not 9600, add, for example,
        // _baudrate = 19200;
    }

    void Huanyang::direction_command(SpindleState mode, ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 4;
        data.rx_length = 4;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x03;
        data.msg[2] = 0x01;

        switch (mode) {
            case SpindleState::Cw:
                data.msg[3] = 0x01;
                break;
            case SpindleState::Ccw:
                data.msg[3] = 0x11;
                break;
            default:  // SpindleState::Disable
                data.msg[3] = 0x08;
                break;
        }
    }

    void IRAM_ATTR Huanyang::set_speed_command(uint32_t dev_speed, ModbusCommand& data) {
        // The units for setting Huanyang speed are Hz * 100.  For a 2-pole motor,
        // RPM is Hz * 60 sec/min.  The maximum possible speed is 400 Hz so
        // 400 * 60 = 24000 RPM.

        // There is a configuration register PD144 that scales the display to show the
        // RPMs.  It is nominally set to 3000 (= 50Hz * 60) for a 2-pole motor or
        // 1500 for a 4-pole motor, but it can be set slightly lower to account for
        // slip - the torque-limited difference between actual spindle speed and the
        // the frequency delivered to the motor windings.

        // Frequency comes from a conversion of revolutions per second to revolutions per minute
        // (factor of 60) and a factor of 2 from counting the number of poles. E.g. rpm * 120 / 100.

        // int targetFrequency = targetRPM * PD005 / MaxRPM
        // = targetRPM * PD005 / (PD005 * PD144 / 50)
        // = targetRPM * 50 / PD144
        //
        // Huanyang wants a factor 100 bigger numbers. So, 1500 rpm -> 25 HZ. Send 2500.

        // The conversion from RPM as requested by the GCode program and the speed number
        // (nominally Hz * 100) is done in mapping code that is shared across all spindles.
        // The dev_speed argument is precomputed by that code.

        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 5;
        data.rx_length = 5;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x05;  // Set register command
        data.msg[2] = 0x02;  // Register PD002 - main frequency in units of 0.01 Hz
        data.msg[3] = dev_speed >> 8;
        data.msg[4] = dev_speed & 0xFF;
    }

    VFD::response_parser Huanyang::initialization_sequence(int index, ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 6;
        data.rx_length = 6;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x01;  // Read setting
        data.msg[2] = 0x03;  // Len
        //      [3] = set below...
        data.msg[4] = 0x00;
        data.msg[5] = 0x00;

        if (index == -1) {
            // Max frequency
            data.msg[3] = 5;  // PD005: max frequency the VFD will allow. Normally 400.

            return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                uint16_t value = (response[4] << 8) | response[5];
#ifdef DEBUG_VFD
                log_debug("VFD: Max frequency = " << value);
#endif

                // Set current RPM value? Somewhere?
                auto huanyang           = static_cast<Huanyang*>(vfd);
                huanyang->_maxFrequency = value;
                return true;
            };

        } else if (index == -2) {
            // Min Frequency
            data.msg[3] = 11;  // PD011: frequency lower limit. Normally 0.

            return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                uint16_t value = (response[4] << 8) | response[5];

#ifdef DEBUG_VFD
                debug_all("VFD: Min frequency set to " << value);
#endif

                // Set current RPM value? Somewhere?
                auto huanyang           = static_cast<Huanyang*>(vfd);
                huanyang->_minFrequency = value;
                return true;
            };
        } else if (index == -3) {
            // Max rated revolutions @ 50Hz

            data.msg[3] = 144;  // PD144: max rated motor revolution at 50Hz => 24000@400Hz = 3000@50HZ

            return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                uint16_t value = (response[4] << 8) | response[5];
#ifdef DEBUG_VFD
                log_debug("VFD: Max rated revolutions @ 50Hz = " << value);
#endif
                // Set current RPM value? Somewhere?
                auto huanyang           = static_cast<Huanyang*>(vfd);
                huanyang->_maxRpmAt50Hz = value;

                // Regarding PD144, the 2 versions of the manuals both say "This is set according to the
                // actual revolution of the motor. The displayed value is the same as this set value. It
                // can be used as a monitoring parameter, which is convenient to the user. This set value
                // corresponds to the revolution at 50Hz".

                // Calculate the VFD settings:
                huanyang->updateRPM();

                return true;
            };
        }
        /*
        The number of poles seems to be over constrained information with PD144. If we're wrong, here's how 
        to get this information. Note that you probably have to call 'updateRPM' here then:
        --

        else if (index == -4) {
            // Number Poles

            data.msg[3] = 143;  // PD143: 4 or 2 poles in motor. Default is 4. A spindle being 24000RPM@400Hz implies 2 poles

            return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                uint8_t value = response[4]; // Single byte response.

                // Sanity check. We expect something like 2 or 4 poles.
                if (value <= 4 && value >= 2) {
#ifdef DEBUG_VFD
                    // Set current RPM value? Somewhere?
                    log_debug("VFD: Number of poles set to " << value);
#endif

                    auto huanyang = static_cast<Huanyang*>(vfd);
                    huanyang->_numberPoles = value;
                    return true;
                }
                else {
                    error_all("Initialization of Huanyang spindle failed. Number of poles (PD143, expected 2-4, got %d) is not sane.",
                        value);
                    return false;
                }
            };

        }
        */

        // Done.
        return nullptr;
    }

    void Huanyang::updateRPM() {
        /*
        PD005 = 400.00 ; max frequency the VFD will allow
        MaxRPM = PD005 * 60; but see PD176

        Frequencies are expressed in centiHz.
        */

        if (_minFrequency > _maxFrequency) {
            _minFrequency = _maxFrequency;
        }
        if (_speeds.size() == 0) {
            // Convert from Frequency in centiHz (the divisor of 100) to RPM (the factor of 60)
            SpindleSpeed minRPM = _minFrequency * 60 / 100;
            SpindleSpeed maxRPM = _maxFrequency * 60 / 100;
            shelfSpeeds(minRPM, maxRPM);
        }
        setupSpeeds(_maxFrequency);
        _slop = std::max(_maxFrequency / 40, 1);

        // log_info("VFD: VFD settings read: RPM Range(" << _min_rpm << " , " << _max_rpm << ")]");
    }

    VFD::response_parser Huanyang::get_status_ok(ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 6;
        data.rx_length = 6;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x04;
        data.msg[2] = 0x03;
        data.msg[3] = reg;
        data.msg[4] = 0x00;
        data.msg[5] = 0x00;

        if (reg < 0x03) {
            reg++;
        } else {
            reg = 0x00;
        }
        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool { return true; };
    }

    VFD::response_parser Huanyang::get_current_speed(ModbusCommand& data) {
        // NOTE: data length is excluding the CRC16 checksum.
        data.tx_length = 6;
        data.rx_length = 6;

        // data.msg[0] is omitted (modbus address is filled in later)
        data.msg[1] = 0x04;
        data.msg[2] = 0x03;
        data.msg[3] = 0x01;  // Output frequency
        data.msg[4] = 0x00;
        data.msg[5] = 0x00;

        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
            uint16_t frequency = (response[4] << 8) | response[5];

            // Store speed for synchronization
            vfd->_sync_dev_speed = frequency;
            return true;
        };
    }

    // Configuration registration
    namespace {
        SpindleFactory::InstanceBuilder<Huanyang> registration("Huanyang");
    }
}
