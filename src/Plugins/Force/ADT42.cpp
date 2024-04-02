#include "ADT42.h"



ADT42::ADT42(std::string ip)
    : modbus_client{ip}
{

}

ADT42::ADT42(ADT42 &&other)
    : modbus_client{std::move(other.modbus_client)}
{

}

ADT42 &ADT42::operator=(ADT42 &&other)
{
    modbus_client = std::move(other.modbus_client);
    return *this;
}

Eigen::Vector<float, 4> ADT42::get_force()
{
    auto val = modbus_client.read_reals(29,4);
    if (val.size() != 4)
    {
        return Eigen::Vector<float, 4>::Zero();
    }
    return Eigen::Vector<float, 4>(val.data());
}

void ADT42::zero()
{
    modbus_client.write_bit(4010, true);
}
