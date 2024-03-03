#include "Gyroscope.h"
#include <iostream>
#include "../Utilities/Statistics.h"
#include "../Utilities/Loggers/Logger.h"
#include "../Utilities/Filters/LPF.h"

Gyroscope::Gyroscope(TimeSynchronizer &time_synchronizer, bool skip_calibration)
    :   Sensor(time_synchronizer, "gyro", "time,X,Y,Z,raw_X,raw_Y,raw_Z"),
        initialized{skip_calibration},
        bias{Eigen::Vector3f::Zero()},
        filter{std::make_unique<LPF<Eigen::Vector3f>>(LPF_cufoff_freq)}
{

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
    auto new_last_update = payload.time + offset.value();
    std::scoped_lock lock(value_mutex);
    raw_value = Eigen::Vector3f(payload.X, payload.Y, payload.Z);
    auto delta_time = static_cast<float>(new_last_update - last_update)/1000.0f;
    value = filter->update(raw_value - bias, delta_time);
    last_update = new_last_update;
    log();
}

void Gyroscope::calibrate(Eigen::Vector3f sample) 
{
    static Statistics<Eigen::Vector3f> statistic(1000);
    static Logger logger(LogType::CALIBRATION);
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    ss.precision(3);

    if(statistic.push_sample(sample))
    {
        auto sd = statistic.sd();
        if(sd.maxCoeff() > sd_limit)
        {
            logger("Gyroscope calibration failed! sd: " + std::to_string(sd.maxCoeff()) + " > " + std::to_string(sd_limit));
            statistic.reset();
            return;
        }

        bias = statistic.mean();
        initialized = true;

        ss << "Gyroscope calibrated! Bias: " << bias.format(commaFormat) << ", sd: " << sd.format(commaFormat);
        logger(ss.str());
    }
}
