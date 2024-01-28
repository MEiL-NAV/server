#include "Accelerometer.h"
#include <iostream>

Accelerometer::Accelerometer(TimeSynchronizer &time_synchronizer, bool skip_calibration)
    :   Sensor(time_synchronizer),
        initialized{skip_calibration},
        bias{Eigen::Vector3f::Zero()},
        scalers{Eigen::Vector3f::Ones()}
{
}

void Accelerometer::consumeMessage(const Message &msg)
{
    auto payload = std::get<VectorPacket>(msg.payload);
    if(!initialized)
    {
        calibrate(Eigen::Vector3f(payload.X, payload.Y, payload.Z));
        return;
    }
    auto offset = time_synchronizer.get_offset(msg.node_id);
    if (!offset.has_value())
    {
        return;
    }
    last_update =  payload.time + offset.value();
    raw_value = Eigen::Vector3f(payload.X, payload.Y, payload.Z);
    value = (raw_value - bias).array() * scalers.array();
}

void Accelerometer::calibrate(Eigen::Vector3f sample) 
{
    if (!calibration.calibrate(sample))
    {
        return;
    }
    bias = calibration.calculate_bias();
    scalers = calibration.calculate_scalers();
    Logger(LogType::CALIBRATION) << "Accelerometer calibrated! Bias: " 
        << bias.transpose() << ", scalers: " << scalers.transpose() << "\n";
    initialized = true;
}

AccelerometerCalibration::AccelerometerCalibration()
    :   max{Eigen::Vector3f::Zero()},
        min{Eigen::Vector3f::Zero()},
        statistic(500),
        logger(LogType::CALIBRATION),
        checked_sides{0}
{}

bool AccelerometerCalibration::calibrate(Eigen::Vector3f sample)
{
    if(statistic.push_sample(sample))
    {
        auto sd_max = statistic.sd().maxCoeff();
        if(sd_max < sd_limit)
        {
            save_mean(statistic.mean());
        }
        else
        {
            logger << "Accelerometer sd: " << statistic.sd().transpose() << "\n";
        }
        statistic.reset();
    }
    return 0b00111111 == checked_sides; // All sides checked
}

Eigen::Vector3f AccelerometerCalibration::calculate_bias()
{
    return (max + min) / 2.0f;
}

Eigen::Vector3f AccelerometerCalibration::calculate_scalers()
{
    return  (G * Eigen::Vector3f::Ones()).array() / ((max - min) / 2.0f).array();
}

uint8_t AccelerometerCalibration::calc_sign(Eigen::Vector3f mean)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < 3; i++)
    {
        int sign_val = INVALID_SIGN;
        if (mean[i] > 0.85f*G)
        {
            sign_val = PLUS;
        }
        else if (mean[i] < -0.85f*G)
        {
            sign_val = MINUS;
        }
        else if (std::abs(mean[i]) < 0.05f*G )
        {
            sign_val = ZERO;
        }
        sum += (sign_val << 2 * i);
    }
    return sum;
}

uint8_t AccelerometerCalibration::calc_side(uint8_t signs)
{
    switch(signs)
    {
        case 0b010000:
            return side::TOP;
        case 0b100000:
            return side::BUTTOM;
        case 0b000001:
            return side::FRONT;
        case 0b000010:
            return side::BACK;
        case 0b000100:
            return side::LEFT;
        case 0b001000:
            return side::RIGHT;
        default:
            return side::INVALID_SIDE;
    }
}

void AccelerometerCalibration::save_mean(Eigen::Vector3f mean)
{
    auto signs = calc_sign(mean);
    auto side = calc_side(signs);

    switch(side)
    {
        case side::INVALID_SIDE:
            return;
        case side::FRONT:
            max.x() = mean.x();
            logger << "Detected FRONT side\n";
            break;
        case side::BACK:
            min.x() = mean.x();
            logger << "Detected BACK side\n";
            break;
        case side::LEFT:
            max.y() = mean.y();
            logger << "Detected LEFT side\n";
            break;
        case side::RIGHT:
            min.y() = mean.y();
            logger << "Detected RIGHT side\n";
            break;
        case side::TOP:
            max.z() = mean.z();
            logger << "Detected TOP side\n";
            break;
        case side::BUTTOM:
            min.z() = mean.z();
            logger << "Detected BUTTOM side\n";
            break;
    }

    checked_sides |= side;
}
