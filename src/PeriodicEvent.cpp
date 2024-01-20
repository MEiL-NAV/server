#include "PeriodicEvent.h"

PeriodicEvent::PeriodicEvent(uint32_t period_millis)
    : should_exit(false), period_millis(period_millis) 
{
    sender_thread = std::thread(&PeriodicEvent::event, this);
}

PeriodicEvent::~PeriodicEvent() 
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
