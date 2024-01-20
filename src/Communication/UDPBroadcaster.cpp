#include "UDPBroadcaster.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UDPBroadcaster::UDPBroadcaster(uint16_t broadcast_port) : port(broadcast_port) {
    initializeSocket();
}

UDPBroadcaster::~UDPBroadcaster() {
    close(socket_fd);
}

void UDPBroadcaster::initializeSocket() {
    if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Enable broadcast
    int broadcastEnable = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        perror("setsockopt(SO_BROADCAST) failed");
        close(socket_fd);
        exit(EXIT_FAILURE);
    }

    // Address information
    memset(&broadcast_address, 0, sizeof(broadcast_address));
    broadcast_address.sin_family = AF_INET;
    broadcast_address.sin_addr.s_addr = INADDR_BROADCAST;
    broadcast_address.sin_port = 0;

    // Bind the socket
    if (bind(socket_fd, reinterpret_cast<const sockaddr*>(&broadcast_address), sizeof(broadcast_address)) < 0) {
        perror("Socket bind failed");
        close(socket_fd);
        exit(EXIT_FAILURE);
    }
    broadcast_address.sin_port = htons(port);
}

void UDPBroadcaster::send(std::string message) {
    send(reinterpret_cast<const uint8_t*>(message.c_str()), message.size());
}

void UDPBroadcaster::send(std::vector<uint8_t> bytes) {
    send(bytes.data(), bytes.size());
}

template <size_t T>
void UDPBroadcaster::send(std::array<uint8_t, T> bytes) {
    send(bytes.data(), bytes.size());
}

void UDPBroadcaster::send(const uint8_t* data, size_t size) {
    std::cout << broadcast_address.sin_port << std::endl;
    sendto(socket_fd, data, size, 0, reinterpret_cast<const sockaddr*>(&broadcast_address), sizeof(broadcast_address));
}
