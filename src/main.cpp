#include <iostream>
#include <csignal>
#include <semaphore>
#include "UDPListener.h"
#include "Protocol/Message.h"

namespace
{
    std::binary_semaphore shutting_down(0);
}

int main()
{
    std::signal(SIGINT, 
        [](int sig) {
            if(sig == SIGINT)
            {
                shutting_down.release();
            }
        }
    );

    int port = 1234;

    UDPListener udpListener(port);

    udpListener.set_message_event([] (const Message& msg)
    {
        if(msg.command == Command::DUMMY)
        {
            std::cout << std::get<DummySensorPacket>(msg.payload).counter << std::endl;
        }
    });

    shutting_down.acquire();

    std::cout << "Bye!" << std::endl;

    return 0;
}
