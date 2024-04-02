#include "ForceLogger.h"
#include "../../Utilities/Millis.h"
#include "../../Utilities/Config/Config.h"


ForceLogger::ForceLogger()
    :   PeriodicEvent(10),
        logger("force", "time, forces...")
{
    forces.setZero(0);
    for(const auto &ip: Config::get_singleton().force_sensor_ips)
    {
        sources.push_back(ip);
    }
}

Eigen::VectorXf ForceLogger::get_forces()
{
    std::scoped_lock<std::mutex> lock(force_mtx);
    return forces;
}

void ForceLogger::zero() 
{
    for(auto &source: sources)
    {
        source.zero();
    }
}

void ForceLogger::periodic_event()
{
    std::scoped_lock<std::mutex> lock(force_mtx);
    forces.resize(1 + 4 * sources.size());
    forces(0) = Millis::get();
    for(size_t i = 0; i < sources.size(); i++)
    {
        forces.segment<4>(1 + 4 * i) = sources[i].get_force();
    }
    logger << forces;
}
