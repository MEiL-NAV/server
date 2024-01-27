#include "UDPListener.h"
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../Protocol/MessageDecoder.h"

UDPListener::UDPListener(uint16_t port) 
    : port(port), sockfd(-1), is_listening(true) {
    initialize_socket();
    start_listening_thread();
}

UDPListener::~UDPListener() {
    stop_listening_thread();
    if (sockfd != -1) {
        close(sockfd);
    }
}

void UDPListener::set_message_event(std::function<void(const Message &)> func) 
{
    on_message = func;
}

void UDPListener::initialize_socket() {
    // Create UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("Socket creation failed");
        return;
    }

    // Set timeout on socket
    struct timeval tv{0, 100000};
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
        perror("Timeout set rrror");
    }

    // Set up server address
    struct sockaddr_in serverAddress;
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(port);

    // Bind the socket
    if (bind(sockfd, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        perror("Socket binding failed");
        close(sockfd);
        sockfd = -1;
        return;
    }
}

void UDPListener::start_listening_thread() {
    listen_thread = std::thread([this]() {
        while (is_listening) {
            receive();
        }
    });
}

void UDPListener::stop_listening_thread() {
    is_listening = false;
    if (listen_thread.joinable()) {
        listen_thread.join();
    }
}

void UDPListener::receive() {

    MessageDecoder decoder;

    uint8_t buffer[MessageDecoder::max_datagram_size];

    ssize_t bytesRead = recvfrom(sockfd, buffer, sizeof(buffer), 0, NULL, NULL);

    if (bytesRead <= 0)
    {
        return;
    }
    
    // for (ssize_t i = 0; i < bytesRead; ++i) {
    //     printf("%02X ", buffer[i]);
    // }
    // std::cout << std::endl;

    auto message = decoder.decode(buffer,bytesRead);

    if(message == nullptr || !on_message)
    {
        return;
    }
    on_message(*message);
}
