#pragma once

#include "../Protocol/Message.h"
#include "../TimeSynchronizer.h"
#include "../Utilities/Millis.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "../Utilities/Loggers/LoggerCSV.h"
#include <mutex>

template <typename T>
class Sensor
{
public:
    Sensor(TimeSynchronizer& time_synchronizer, std::string csv_filename, std::string csv_header = "") 
        :   time_synchronizer{time_synchronizer},
            last_update{0U},
            sem{false},
            logger{csv_filename, csv_header}
    {}

    virtual ~Sensor() {}

    virtual void consumeMessage([[maybe_unused]]const Message& msg) {};

    bool healthy()
    {
        std::scoped_lock lock(value_mutex);
        return last_update > 0U && Millis::get() - last_update < timeout;
    }

    bool has_new_value()
    {
        std::scoped_lock lock(value_mutex);
        return sem;
    }

    std::pair<uint32_t,T> get_raw_value(bool reset_sem = false)
    {
        std::scoped_lock lock(value_mutex);
        if (reset_sem)
        {
            sem = false;
        }
        return {last_update, raw_value};
    }

    std::pair<uint32_t,T> get_value(bool reset_sem = false)
    {
        std::scoped_lock lock(value_mutex);
        if (reset_sem)
        {
            sem = false;
        }
        return {last_update, value};
    }

    virtual void log()
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
    bool sem;
    std::mutex value_mutex;
    LoggerCSV logger;
};
