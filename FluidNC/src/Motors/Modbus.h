
#pragma once

#include "../Uart.h"

namespace MotorDrivers {
    //extern Uart _uart;

    class ModbusHandler {
        public:
            uint8_t _encoder_modbus_id;
            virtual void onSetCurrentAngle(int32_t);
            virtual void update();
    };

    class Modbus {
        protected:
            static const int RS485_MAX_MSG_SIZE     = 16;  // more than enough for a modbus message
            static const int MAX_RETRIES            = 2;   // otherwise the spindle is marked 'unresponsive'

            static ModbusHandler* _firstHandler;

            struct ModbusCommand {
                bool critical;  // TODO SdB: change into `uint8_t critical : 1;`: We want more flags...

                uint8_t tx_length;
                uint8_t rx_length;
                uint8_t msg[RS485_MAX_MSG_SIZE];
            };

            using response_parser = bool (*)(const uint8_t* response, ModbusHandler* handler);

            static response_parser get_encoder_pos(ModbusCommand& data);

        private:
            static QueueHandle_t uart_cmd_queue;
            static TaskHandle_t  uart_cmdTaskHandle;
            static void          uart_cmd_task(void* pvParameters);

            static void reportParsingErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length);
            static void reportCmdErrors(ModbusCommand cmd, uint8_t* rx_message, size_t read_length, uint8_t id);

            static uint16_t ModRTU_CRC(uint8_t* buf, int msg_len);
        
        public:
            static Uart *_uart;
            
            static void init(ModbusHandler* handler);
    };
}