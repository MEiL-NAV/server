#include "TimeSynchronizer.h"
#include <iostream>

TimeSynchronizer::TimeSynchronizer(uint32_t period_millis,
                                 uint16_t broadcast_port) 
    : PeriodicEvent(period_millis), udp_broadcaster(broadcast_port)
{

}

void TimeSynchronizer::periodic_event() 
{
    udp_broadcaster.send("HELLOW");
}