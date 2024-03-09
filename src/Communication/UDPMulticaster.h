#pragma once

#include <string>
#include <vector>
#include <array>
#include <cstdint>
#include <arpa/inet.h>

class UDPMulticaster {
public:
    UDPMulticaster(std::string multicast_address, uint16_t multicast_port, std::string local_interface = "");
    ~UDPMulticaster();

    void send(std::string message);
    void send(std::vector<uint8_t> bytes);

    template <size_t T>
    void send(std::array<uint8_t, T> bytes);

private:
    int socket_fd;
    std::string multicast_address;
    uint16_t port;
    std::string local_interface;
    struct sockaddr_in multicast_group;
    int ttl;

    void initializeSocket();
    void send(const uint8_t* data, size_t size);
};

template <size_t T>
void UDPMulticaster::send(std::array<uint8_t, T> bytes) {
    send(bytes.data(), bytes.size());
}