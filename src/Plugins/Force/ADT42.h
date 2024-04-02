#pragma once

#include "../../Utilities/Modbus/ModbusClient.h"
#include <eigen3/Eigen/Dense>

class ADT42
{
public:
    ADT42(std::string ip);
    ADT42(const ADT42&) = delete;
    ADT42& operator=(const ADT42&) = delete;
    ADT42(ADT42&& other);
    ADT42& operator=(ADT42&& other);

    Eigen::Vector<float,4> get_force();
    void zero();

private:
    ModbusClient modbus_client;
};
