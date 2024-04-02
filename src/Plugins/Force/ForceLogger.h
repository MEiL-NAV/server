#pragma once

#include "../../Utilities/PeriodicEvent.h"
#include "../../Utilities/Loggers/LoggerCSV.h"
#include <eigen3/Eigen/Dense>
#include <mutex>
#include "ADT42.h"

class ForceLogger : protected PeriodicEvent
{
public:
    ForceLogger();
    Eigen::VectorXf get_forces();
    void zero();

protected:
    void periodic_event() override;

private:
    std::vector<ADT42> sources;

    std::mutex force_mtx;
    Eigen::VectorXf forces;
    LoggerCSV logger;
};