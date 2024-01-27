#include "Millis.h"

std::chrono::time_point<std::chrono::steady_clock> Millis::start_time = std::chrono::steady_clock::now();

void Millis::start() 
{
    Millis::start_time = std::chrono::steady_clock::now();
}

uint32_t Millis::get() 
{ 
   const auto end_time = std::chrono::steady_clock::now();
   const auto diff = end_time - start_time;
   return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
}
