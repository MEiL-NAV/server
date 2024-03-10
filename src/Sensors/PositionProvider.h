#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>
#include <memory>
#include "../Utilities/Loggers/LoggerCSV.h"

class PositionProvider : public Sensor<Eigen::Vector3f>
{
public:
    PositionProvider(TimeSynchronizer& time_synchronizer);
    virtual ~PositionProvider() {}

    void consumeMessage(const Message& msg) override;
};