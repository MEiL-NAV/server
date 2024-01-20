#include "Communication/UDPListener.h"
#include "Protocol/Message.h"
#include "TimeSynchronizer.h"

class NaviSystem
{
public:
    NaviSystem();
    ~NaviSystem();

private:
    UDPListener udp_listener;
    TimeSynchronizer time_syncronizer;

    void messageHandler(const Message& msg);
};