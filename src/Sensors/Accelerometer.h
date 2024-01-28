#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>

class Accelerometer : public Sensor<Eigen::Vector3f>
{
public:
    Accelerometer(TimeSynchronizer& time_synchronizer);

    void consumeMessage(const Message& msg) override;
};