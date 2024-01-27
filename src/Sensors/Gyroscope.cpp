#include "Gyroscope.h"
#include <iostream>

Gyroscope::Gyroscope(TimeSynchronizer &time_synchronizer)
    :   Sensor(time_synchronizer),
        initialized{false},
        value{Eigen::Vector3f::Zero()}
{
    last_update = 0U;
}

void Gyroscope::consumeMessage(const Message &msg)
{
    auto payload = std::get<VectorPacket>(msg.payload);
    if(!initialized)
    {
        calibrate(Eigen::Vector3f(payload.X, payload.Y, payload.Z));
        return;
    }
    auto offset = time_synchronizer.get_offset(msg.node_id);
    if (!offset.has_value())
    {
        return;
    }
    last_update =  payload.time + offset.value();
    Eigen::Vector3f new_value = Eigen::Vector3f(payload.X, payload.Y, payload.Z) - bias;
    value = LPF * value + new_value * (1.0f - LPF);
}

void Gyroscope::calibrate(Eigen::Vector3f sample) 
{
    static int counter = 0;
    static Eigen::Vector3f sum = Eigen::Vector3f::Zero();

    sum += sample;
    counter++;
    if(counter == 1000)
    {
        bias = sum / counter;
        initialized = true;
    }
}
