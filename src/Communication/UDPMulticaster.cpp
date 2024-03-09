#include "UDPMulticaster.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

UDPMulticaster::UDPMulticaster(std::string multicast_address, uint16_t multicast_port, std::string local_interface) 
    : multicast_address{multicast_address}, port(multicast_port), local_interface{local_interface}, ttl{3}
{
    initializeSocket();
}

UDPMulticaster::~UDPMulticaster() {
    close(socket_fd);
}

void UDPMulticaster::initializeSocket() 
{
    if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Set the TTL for multicast packets
    if (setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_TTL, (void *)&ttl, sizeof(ttl)) < 0) {
        perror("setsockopt(IP_MULTICAST_TTL) failed");
        close(socket_fd);
        exit(EXIT_FAILURE);
    }

    // Set the interface for outgoing multicast packets
    if (!local_interface.empty()) {
        struct in_addr local_interface_addr;
        local_interface_addr.s_addr = inet_addr(local_interface.c_str());
        if (setsockopt(socket_fd, IPPROTO_IP, IP_MULTICAST_IF, (void *)&local_interface_addr, sizeof(local_interface_addr)) < 0) {
            perror("setsockopt(IP_MULTICAST_IF) failed");
            close(socket_fd);
            exit(EXIT_FAILURE);
        }
    }

    // Address information
    memset(&multicast_group, 0, sizeof(multicast_group));
    multicast_group.sin_family = AF_INET;
    multicast_group.sin_addr.s_addr = inet_addr(multicast_address.c_str());
    multicast_group.sin_port = htons(port);
}

void UDPMulticaster::send(std::string message) {
    send(reinterpret_cast<const uint8_t*>(message.c_str()), message.size());
}

void UDPMulticaster::send(std::vector<uint8_t> bytes) {
    send(bytes.data(), bytes.size());
}

void UDPMulticaster::send(const uint8_t* data, size_t size) {
    sendto(socket_fd, data, size, 0, reinterpret_cast<const sockaddr*>(&multicast_group), sizeof(multicast_group));
}
