#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>

class Gyroscope : public Sensor
{
public:
    Gyroscope(TimeSynchronizer& time_synchronizer);

    void consumeMessage(const Message& msg) override;

    std::pair<uint32_t,Eigen::Vector3f> get_raw_value()
    {
        return {last_update, value};
    }

private:
    bool initialized;

    Eigen::Vector3f value;
    Eigen::Vector3f bias;

    void calibrate(Eigen::Vector3f sample);

    static constexpr float LPF = 0.3f;
};