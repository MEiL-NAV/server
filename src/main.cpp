#include <iostream>
#include "UDPListener.h"
#include "Protocol/Message.h"

int main()
{
    int port = 1234;

    UDPListener udpListener(port);

    udpListener.set_message_event([] (const Message& msg)
    {
        if(msg.command == Command::DUMMY)
        {
            std::cout << std::get<DummySensorPacket>(msg.payload).counter << std::endl;
        }
    });

    // Let the listener run in the background
    std::this_thread::sleep_for(std::chrono::seconds(30));

    return 0;
}
