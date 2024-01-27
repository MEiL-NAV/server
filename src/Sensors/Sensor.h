#pragma once

#include "../Protocol/Message.h"
#include "../TimeSynchronizer.h"
#include "../Utilities/Millis.h"
#include <iostream>

class Sensor
{
public:
    Sensor(TimeSynchronizer& time_synchronizer) 
        :   time_synchronizer{time_synchronizer},
            last_update{0U}
    {}
    
    virtual ~Sensor() {}

    virtual void consumeMessage(const Message& msg) = 0;

    bool healthy()
    {
        return last_update > 0U && Millis::get() - last_update < timeout;
    }

    static constexpr uint32_t timeout = 200;

protected:
    TimeSynchronizer& time_synchronizer;
    uint32_t last_update;
};