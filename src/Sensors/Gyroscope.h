#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>
#include <memory>
#include "../Utilities/Filters/Filter.h"

class Gyroscope : public Sensor<Eigen::Vector3f>
{
public:
    Gyroscope(TimeSynchronizer& time_synchronizer, bool skip_calibration = false);

    void consumeMessage(const Message& msg) override;

private:
    bool initialized;

    Eigen::Vector3f bias;

    std::unique_ptr<Filter<Eigen::Vector3f>> filter;

    void calibrate(Eigen::Vector3f sample);

    static constexpr float LPF_cufoff_freq = 20.0f;
    static constexpr float sd_limit = 300.0f;
};