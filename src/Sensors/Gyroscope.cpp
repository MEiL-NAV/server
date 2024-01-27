#include "Gyroscope.h"
#include <iostream>
#include "../Utilities/Statistics.h"

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
    static Statistics<Eigen::Vector3f> statistic(1000);

    if(statistic.push_sample(sample))
    {
        auto sd = statistic.sd();
        if(sd.maxCoeff() > sd_limit)
        {
            std::cout << "Gyroscope calibration failed! sd: " << sd.maxCoeff() << " > " << sd_limit << std::endl;
            statistic.reset();
            return;
        }

        bias = statistic.mean();
        initialized = true;
        std::cout << "Gyroscope calibrated! Bias: " << bias.transpose() << ", sd: " << sd.transpose() << std::endl;
    }
}
