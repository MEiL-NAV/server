#include "ForceLogger.h"
#include "Utilities/Millis.h"


ForceLogger::ForceLogger()
    :   PeriodicEvent(100),
        logger("force", "time, forces...")
{
    forces.setZero(0);
}

Eigen::VectorXf ForceLogger::get_forces()
{
    std::scoped_lock<std::mutex> lock(force_mtx);
    return forces;
}

void ForceLogger::periodic_event()
{
    std::scoped_lock<std::mutex> lock(force_mtx);
    forces.resize(1 + sources.size());
    forces(0) = Millis::get();
    for(auto& source : sources)
    {
        // Read force from source
    }

    logger << forces;
}
