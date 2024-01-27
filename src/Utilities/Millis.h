#pragma once

#include <chrono>

class Millis
{
public:
    static void start();
    static uint32_t get();

private:
    static std::chrono::time_point<std::chrono::steady_clock> start_time;
};