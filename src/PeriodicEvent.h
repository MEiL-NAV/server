#pragma once

#include <thread>
#include <chrono>
#include <atomic>
#include <condition_variable>

class PeriodicEvent {
public:
    PeriodicEvent(uint32_t period_millis);
    virtual ~PeriodicEvent();

protected:
    virtual void periodic_event() = 0;

private:
    std::thread sender_thread;
    std::atomic<bool> should_exit;
    uint32_t period_millis;
    std::condition_variable cv;
    std::mutex mutex;

    void event();
};
