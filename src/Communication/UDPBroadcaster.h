#pragma once

#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <arpa/inet.h>

class UDPBroadcaster {
public:
    UDPBroadcaster(uint16_t broadcast_port);
    ~UDPBroadcaster();

    void send(std::string message);
    void send(std::vector<uint8_t> bytes);

    template <size_t T>
    void send(std::array<uint8_t, T> bytes);

private:
    int socket_fd;
    uint16_t port;
    struct sockaddr_in broadcast_address;

    void initializeSocket();
    void send(const uint8_t* data, size_t size);
};

template <size_t T>
void UDPBroadcaster::send(std::array<uint8_t, T> bytes) {
    send(bytes.data(), bytes.size());
}