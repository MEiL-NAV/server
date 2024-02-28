#pragma once

#include "../Protocol/Message.h"
#include "../TimeSynchronizer.h"
#include "../Utilities/Millis.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "../Utilities/Loggers/LoggerCSV.h"

template <typename T>
class Sensor
{
public:
    Sensor(TimeSynchronizer& time_synchronizer, std::string csv_filename, std::string csv_header = "") 
        :   time_synchronizer{time_synchronizer},
            last_update{0U},
            logger{csv_filename, csv_header}
    {}

    Sensor(TimeSynchronizer& time_synchronizer) requires(std::is_same_v<T, Eigen::Vector3f>)
        :   time_synchronizer{time_synchronizer},
            last_update{0U},
            raw_value{Eigen::Vector3f::Zero()},
            value{Eigen::Vector3f::Zero()}
    {}
    
    virtual ~Sensor() {}

    virtual void consumeMessage([[maybe_unused]]const Message& msg) {};

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

    void log()
    {
        Eigen::VectorXf log(7);
        log(0) = last_update;
        log.segment<3>(1) =  value;
        log.segment<3>(4) =  raw_value; 
        logger << log;
    }

    static constexpr uint32_t timeout = 200;

protected:
    TimeSynchronizer& time_synchronizer;
    uint32_t last_update;

    T raw_value;
    T value;
    LoggerCSV logger;
};
