#pragma once

#include <thread>
#include <functional>
#include "../Protocol/Message.h"

class UDPListener {
public:
    UDPListener(std::string multicast_address, uint16_t port, std::string local_interface = "");
    ~UDPListener();

    void stop_listening_thread();

    void set_message_event(std::function<void(const Message&)> func);

private:
    void initialize_socket();
    void start_listening_thread();
    void receive();

private:
    std::string multicast_address;
    uint16_t port;
    std::string local_interface;
    int sockfd;
    std::atomic_bool is_listening;

    std::function<void(const Message&)> on_message;

    std::thread listen_thread;
};