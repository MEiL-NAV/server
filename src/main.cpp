#include <iostream>
#include <csignal>
#include <semaphore>
#include <zmq.hpp>
#include "NaviSystem.hpp"
#include "Utilities/Millis.h"
#include "Utilities/Loggers/Logger.h"
#include "Utilities/Config/Config.h"

namespace
{
    std::binary_semaphore shutting_down(0);
}

int main()
{
    zmq::context_t ctx;
    Logger::set_ctx(ctx);
    Logger logger(LogType::INFO);
    logger("Starting!");
    Millis::start();
    auto config = Config::get_singleton();
    std::signal(SIGINT, 
        [](int sig) {
            if(sig == SIGINT)
            {
                shutting_down.release();
            }
        }
    );
    NaviSystem navi_sys{ctx, config};
    shutting_down.acquire();
    // std::cout << config.log_dir_path << std::endl;
    logger("Bye!");
    Logger::deconstruct();
    return 0;
}
