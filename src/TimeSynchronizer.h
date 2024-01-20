#include "PeriodicEvent.h"
#include "Communication/UDPBroadcaster.h"

class TimeSynchronizer : public PeriodicEvent
{
public:
    TimeSynchronizer(uint32_t period_millis, uint16_t broadcast_port = 50000U);
    virtual ~TimeSynchronizer() {}

protected:
    void periodic_event() override;

private:
    UDPBroadcaster udp_broadcaster;
};