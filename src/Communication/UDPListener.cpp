#include "UDPListener.h"
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../Protocol/MessageDecoder.h"

UDPListener::UDPListener(std::string multicast_address, uint16_t port, std::string local_interface) 
    : multicast_address{multicast_address}, port(port), local_interface{local_interface}, sockfd(-1), is_listening(true) {
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
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("Socket creation failed");
        return;
    }

    // Set timeout on socket
    struct timeval tv{0, 100000};
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("Timeout set error");
    }

    // Allow multiple sockets to use the same port number
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        perror("SO_REUSEADDR failed");
    }

    // Set up server address
    struct sockaddr_in serverAddress;
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(port);

    // Bind the socket
    if (bind(sockfd, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1) {
        perror("Socket binding failed");
        close(sockfd);
        sockfd = -1;
        return;
    }

    // Join the multicast group
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_address.c_str());
    if  (local_interface.empty())
    {
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    }
    else
    {
        mreq.imr_interface.s_addr = inet_addr(local_interface.c_str());
    }
    if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0) {
        perror("Joining multicast group failed");
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
    if(!is_listening)
    {
        return;
    }
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

    auto message = decoder.decode(buffer,bytesRead);

    if(message == nullptr || !on_message)
    {
        return;
    }
    on_message(*message);
}
