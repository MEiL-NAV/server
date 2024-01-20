#include "NaviSystem.hpp"
#include <functional>

NaviSystem::NaviSystem()
    : udp_listener{1234}, time_syncronizer(2000,50000)
{
    udp_listener.set_message_event(std::bind(&NaviSystem::messageHandler, this, std::placeholders::_1));
}

NaviSystem::~NaviSystem() 
{

}

void NaviSystem::messageHandler(const Message &msg) 
{

}
