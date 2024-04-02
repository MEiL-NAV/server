#pragma once

#include <modbus/modbus.h>
#include <optional>
#include <string>
#include "../Loggers/Logger.h"

class ModbusClient
{
public:
    ModbusClient(std::string ip, uint16_t port = 502U, uint8_t slave_id = 1U);
    ModbusClient(const ModbusClient&) = delete;
    ModbusClient& operator=(const ModbusClient&) = delete;
    ModbusClient(ModbusClient&& other);
    ModbusClient& operator=(ModbusClient&& other);
    ~ModbusClient();

    std::optional<float> read_real(uint16_t address);
    std::vector<float> read_reals(uint16_t address, uint16_t n);
    void write_bit(uint16_t address, bool bit);

private:
    modbus_t* mb;
    std::string ip;
    uint16_t port;
    uint8_t slave_id;
    bool connected;
    Logger logger;

    void connect();
    void disconnect();
};
