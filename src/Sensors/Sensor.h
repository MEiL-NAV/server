#pragma once

#include "../Protocol/Message.h"
#include "../TimeSynchronizer.h"
#include "../Utilities/Millis.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

template <typename T>
class Sensor
{
public:
    Sensor(TimeSynchronizer& time_synchronizer) 
        :   time_synchronizer{time_synchronizer},
            last_update{0U}
    {}

    Sensor(TimeSynchronizer& time_synchronizer) requires(std::is_same_v<T, Eigen::Vector3f>)
        :   time_synchronizer{time_synchronizer},
            last_update{0U},
            raw_value{Eigen::Vector3f::Zero()},
            value{Eigen::Vector3f::Zero()}
    {}
    
    virtual ~Sensor() {}

    virtual void consumeMessage(const Message& msg) = 0;

    bool healthy()
    {
        return last_update > 0U && Millis::get() - last_update < timeout;
    }

    std::pair<uint32_t,T> get_raw_value()
    {
        return {last_update, raw_value};
    }

    std::pair<uint32_t,T> get_value()
    {
        return {last_update, value};
    }

    static constexpr uint32_t timeout = 200;

protected:
    TimeSynchronizer& time_synchronizer;
    uint32_t last_update;

    T raw_value;
    T value;
};
