#pragma once

#include <iostream>

enum LogType : int 
{
    INFO          = 1 << 0,
    DEBUG         = 1 << 1,
    ERROR         = 1 << 2,
    CALIBRATION   = 1 << 3,
    SYNC          = 1 << 4,
};

class Logger
{
public:
    Logger(LogType log_type)
        :   log_type{log_type}
    {}

    template<typename T>
    Logger& operator<<(const T& obj)
    {
        if(log_mask & log_type)
        {
            std::cout << obj;
        }
        return *this;
    }

    static void set_mask(int new_mask)
    {
        log_mask = new_mask;
    }


private:
    LogType log_type;

    static int log_mask;
};
