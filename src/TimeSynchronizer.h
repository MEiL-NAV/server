#pragma once

#include <map>
#include <mutex>
#include "Utilities/PeriodicEvent.h"
#include "Communication/UDPBroadcaster.h"
#include "Protocol/Message.h"
#include <optional>

class TimeSynchronizer : public PeriodicEvent
{
public:
    TimeSynchronizer(uint32_t period_millis, uint16_t broadcast_port = 50000U);
    virtual ~TimeSynchronizer() {}

    std::optional<int32_t> get_offset(uint8_t node_id);
    void consumeMessage(const Message& msg);

protected:
    void periodic_event() override;

private:
    UDPBroadcaster udp_broadcaster;
    std::tuple<uint8_t, uint32_t> last_sync_time;
    uint8_t sync_id;

    std::map<uint8_t, int32_t> offset_table;
    std::mutex mtx;
};