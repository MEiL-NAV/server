#include "PositionProvider.h"

PositionProvider::PositionProvider(TimeSynchronizer &time_synchronizer)
    :   Sensor(time_synchronizer, "pos_provider", "time,X,Y,Z")
{
}

void PositionProvider::consumeMessage(const Message &msg)
{
    auto payload = std::get<VectorPacket>(msg.payload);
    auto offset = time_synchronizer.get_offset(msg.node_id);
    if (!offset.has_value())
    {
        return;
    }
    last_update =  payload.time + offset.value();
    raw_value = Eigen::Vector3f(payload.X, payload.Y, payload.Z);
    value = raw_value;
    log();
}
