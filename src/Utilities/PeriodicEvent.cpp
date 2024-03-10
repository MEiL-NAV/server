#include "PeriodicEvent.h"
#include <iostream>

PeriodicEvent::PeriodicEvent(uint32_t period_millis, bool self_start)
    : should_exit(false), period_millis(period_millis) 
{
    if (self_start)
    {
       start_periodic_task();
    }
}

PeriodicEvent::~PeriodicEvent() 
{
    stop_periodic_task();
}

void PeriodicEvent::start_periodic_task() 
{
    should_exit.store(false);
    if(!sender_thread.joinable())
    {
        sender_thread = std::thread(&PeriodicEvent::event, this);
    }
}

void PeriodicEvent::stop_periodic_task() 
{
    should_exit.store(true);
    cv.notify_all();
    if (sender_thread.joinable()) {
        sender_thread.join();
    }
}

void PeriodicEvent::event() {
    while (!should_exit.load()) {
        periodic_event();
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait_for(lock, std::chrono::milliseconds(period_millis), [this] { return should_exit.load(); });
    }
}
