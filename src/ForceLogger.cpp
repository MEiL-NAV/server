#include "ForceLogger.h"
#include "Utilities/Millis.h"


ForceLogger::ForceLogger()
    :   PeriodicEvent(20),
        logger("force.csv", "time, forces...")
{

}

Eigen::VectorXf ForceLogger::get_forces()
{
    std::scoped_lock<std::mutex> lock(force_mtx);
    return forces;
}

void ForceLogger::periodic_event()
{
    std::scoped_lock<std::mutex> lock(force_mtx);

    for(auto& source : sources)
    {
        // Read force from source
    }

    Eigen::VectorXf log;
    log << Millis::get(), forces;
    logger << log;
}
