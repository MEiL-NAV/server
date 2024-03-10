#include "Utilities/PeriodicEvent.h"
#include "Utilities/Loggers/LoggerCSV.h"
#include <eigen3/Eigen/Dense>
#include <mutex>

struct ForceSource
{
    std::string address;
    uint16_t port;
    uint16_t modbus_register;
};

class ForceLogger : protected PeriodicEvent
{
public:
    ForceLogger();
    Eigen::VectorXf get_forces();

protected:
    void periodic_event() override;

private:
    std::vector<ForceSource> sources;

    std::mutex force_mtx;
    Eigen::VectorXf forces;
    LoggerCSV logger;
};