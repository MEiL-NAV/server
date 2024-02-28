#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>
#include "../Utilities/Loggers/Logger.h"
#include "../Utilities/Statistics.h"
#include <cstdint>

class AccelerometerCalibration
{
public:
    AccelerometerCalibration();

    bool calibrate(Eigen::Vector3f sample);

    Eigen::Vector3f calculate_bias();
    Eigen::Vector3f calculate_scalers();

private:
    Eigen::Vector3f max;
    Eigen::Vector3f min;
    Statistics<Eigen::Vector3f> statistic;
    Logger logger;
    uint8_t checked_sides;


    void save_mean(Eigen::Vector3f mean);
    uint8_t calc_sign(Eigen::Vector3f mean);
    uint8_t calc_side(uint8_t signs);

    enum sign : uint8_t
    {
        ZERO            = 0, // 00
        PLUS            = 1, // 01
        MINUS           = 2, // 10
        INVALID_SIGN    = 3  // 11
    };

    enum side: uint8_t
    {
        INVALID_SIDE    = 0,
        TOP             = 1,
        BUTTOM          = 2,
        FRONT           = 4,
        BACK            = 8,
        RIGHT           = 16,
        LEFT            = 32,
    };

    static constexpr float G = 9.805f;
    static constexpr float sd_limit = 0.1f;
};

class Accelerometer : public Sensor<Eigen::Vector3f>
{
public:
    Accelerometer(TimeSynchronizer& time_synchronizer, bool skip_calibration = false);
    virtual ~Accelerometer() {}

    void consumeMessage(const Message& msg) override;

private:
    bool initialized;

    Eigen::Vector3f bias;
    Eigen::Vector3f scalers;

    AccelerometerCalibration calibration;

    void calibrate(Eigen::Vector3f sample);
};