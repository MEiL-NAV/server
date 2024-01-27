#include "Accelerometer.h"
#include <iostream>

Accelerometer::Accelerometer(TimeSynchronizer &time_synchronizer)
    : Sensor(time_synchronizer)
{
    last_update = 0U;
}

void Accelerometer::consumeMessage(const Message &msg)
{

    auto offset = time_synchronizer.get_offset(msg.node_id);
    if (!offset.has_value())
    {
        return;
    }
    auto payload = std::get<VectorPacket>(msg.payload);
    last_update =  payload.time + offset.value();
    value = Eigen::Vector3f(payload.X, payload.Y, payload.Z);
}
