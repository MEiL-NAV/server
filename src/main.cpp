#include <iostream>
#include <csignal>
#include <semaphore>
#include <zmq.hpp>
#include "NaviSystem.hpp"
#include "Utilities/Millis.h"
#include "Utilities/Loggers/Logger.h"

namespace
{
    std::binary_semaphore shutting_down(0);
}

int main()
{
    zmq::context_t ctx;
    Logger::set_ctx(ctx);
    Logger::set_mask(LogType::CALIBRATION | LogType::INFO);
    Logger logger(LogType::INFO);
    logger("Starting!");
    Millis::start();
    std::signal(SIGINT, 
        [](int sig) {
            if(sig == SIGINT)
            {
                shutting_down.release();
            }
        }
    );
    NaviSystem navi_sys{ctx};
    shutting_down.acquire();
    logger("Bye!");
    Logger::deconstruct();
    return 0;
}
