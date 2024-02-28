#include <iostream>
#include <csignal>
#include <semaphore>
#include "NaviSystem.hpp"
#include "Utilities/Millis.h"
#include "Utilities/Logger.h"

namespace
{
    std::binary_semaphore shutting_down(0);
}

int main()
{
    Logger::set_mask(LogType::CALIBRATION | LogType::INFO);
    Logger(LogType::INFO).prefix() << "Starting!\n";
    Millis::start();
    std::signal(SIGINT, 
        [](int sig) {
            if(sig == SIGINT)
            {
                shutting_down.release();
            }
        }
    );
    NaviSystem navi_sys;
    shutting_down.acquire();
    Logger(LogType::INFO).prefix() << "Bye!\n";
    return 0;
}
