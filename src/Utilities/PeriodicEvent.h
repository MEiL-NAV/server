#pragma once

#include <thread>
#include <chrono>
#include <atomic>
#include <condition_variable>

class PeriodicEvent {
public:
    PeriodicEvent(uint32_t period_millis, bool self_start = true);
    virtual ~PeriodicEvent();

protected:
    void start_periodic_task();
    void stop_periodic_task();
    virtual void periodic_event() {};

private:
    std::thread sender_thread;
    std::atomic<bool> should_exit;
    uint32_t period_millis;
    std::condition_variable cv;
    std::mutex mutex;

    void event();
};
