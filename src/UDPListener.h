#pragma once

#include <thread>
#include <functional>
#include "Protocol/Message.h"

class UDPListener {
public:
    UDPListener(uint16_t port);
    ~UDPListener();

    void set_message_event(std::function<void(const Message&)> func);

private:
    void initialize_socket();
    void start_listening_thread();
    void stop_listening_thread();
    void receive();

private:
    uint16_t port;
    int sockfd;
    std::atomic_bool is_listening;

    std::function<void(const Message&)> on_message;

    std::thread listen_thread;
};