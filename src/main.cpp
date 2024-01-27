#include <iostream>
#include <csignal>
#include <semaphore>
#include "NaviSystem.hpp"
#include "Utilities/Millis.h"

namespace
{
    std::binary_semaphore shutting_down(0);
}

int main()
{
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
    std::cout << "Bye!" << std::endl;
    return 0;
}
