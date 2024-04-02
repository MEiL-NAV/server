#include "ModbusClient.h"
#include <iostream>

ModbusClient::ModbusClient(std::string ip, uint16_t port, uint8_t slave_id)
    : ip{ip}, port{port}, slave_id{slave_id}, connected{false}, logger(LogType::MODBUS)
{
    connect();
}

ModbusClient::ModbusClient(ModbusClient &&other)
    : mb{other.mb}, ip{other.ip}, port{other.port}, slave_id{other.slave_id}, connected{other.connected}, logger(LogType::MODBUS)
{
    other.mb = nullptr;
    other.connected = false;
}

ModbusClient &ModbusClient::operator=(ModbusClient &&other)
{
    if(this == &other)
    {
        return *this;
    }
    mb = other.mb;
    ip = other.ip;
    port = other.port;
    slave_id = other.slave_id;
    connected = other.connected;
    logger = Logger(LogType::MODBUS);
    other.mb = nullptr;
    other.connected = false;
    return *this;
}

ModbusClient::~ModbusClient()
{
    disconnect();
}

std::optional<float> ModbusClient::read_real(uint16_t address)
{
    if(!connected)
    {
        return std::nullopt;
    }
    float val;
    uint16_t tab_reg[2];
    modbus_read_registers(mb, address, 2, tab_reg);
    int rc = modbus_read_registers(mb, address, 2, tab_reg);
    if (rc == -1) {
         std::cerr << modbus_strerror(errno) << std::endl;
        return std::nullopt;
    }
    val = modbus_get_float_cdab(tab_reg);
    return val;
}

std::vector<float> ModbusClient::read_reals(uint16_t address, uint16_t n)
{
    if(!connected)
    {
        return std::vector<float>();
    }
    uint16_t* tab_reg = new uint16_t[2 * n];
    if (modbus_read_registers(mb, address, 2*n, tab_reg) == -1) {
         std::cerr << modbus_strerror(errno) << std::endl;
        return std::vector<float>();
    }
    std::vector<float> vals;
    for(int i = 0; i < n; i++)
    {
        vals.push_back(modbus_get_float_cdab(tab_reg + 2*i));
    }
    delete[] tab_reg;
    return vals;
}

void ModbusClient::write_bit(uint16_t address, bool bit) 
{
    if(!connected)
    {
        return;
    }
    int rc = modbus_write_bit(mb, address, bit);
    if (rc == -1) {
         std::cerr << modbus_strerror(errno) << std::endl;
    }
}

void ModbusClient::connect() 
{
    mb = modbus_new_tcp(ip.c_str(), port);
    if(NULL == mb)
    {
        logger(std::string("Can not create: ") + modbus_strerror(errno));
        return;
    }
    if (modbus_set_slave(mb, slave_id) == -1) {
        logger("Invalid slave ID");
        modbus_free(mb);
        return;
    }
    if (modbus_connect(mb) == -1) 
    {
        logger(std::string("Connection failed: ") + modbus_strerror(errno));
        modbus_free(mb);
        return;
    }
    connected = true;
    logger("Connected to " + ip);
}

void ModbusClient::disconnect() 
{
    if (!connected)
    {
        return;
    }
    modbus_close(mb);
    modbus_free(mb);
}
