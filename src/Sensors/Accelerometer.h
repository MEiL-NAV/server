#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>

class Accelerometer : public Sensor
{
public:
    Accelerometer(TimeSynchronizer& time_synchronizer);

    void consumeMessage(const Message& msg) override;

    std::pair<uint32_t,Eigen::Vector3f> get_raw_value()
    {
        return {last_update, value};
    }

private:
    Eigen::Vector3f value;
};