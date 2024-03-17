#include "Accelerometer.h"
#include <iostream>
#include "../Utilities/Config/Config.h"

Accelerometer::Accelerometer(TimeSynchronizer &time_synchronizer, bool skip_calibration)
    :   Sensor(time_synchronizer, "accel", "time,X,Y,Z,raw_X,raw_Y,raw_Z"),
        initialized{skip_calibration}, was_calibrated{false}
{
    auto config = Config::get_singleton();
    coefficients.R = config.accelerometer_R;
    coefficients.b = config.accelerometer_bias;
}

Accelerometer::~Accelerometer() 
{
    if(was_calibrated)
    {
        Logger(LogType::CALIBRATION)("Saving accelerometer calibration coefficients!");
        auto config = Config::get_singleton_mut();
        config.accelerometer_R = coefficients.R;
        config.accelerometer_bias = coefficients.b;
        config.save();
    }
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
    std::scoped_lock lock(value_mutex);
    last_update =  payload.time + offset.value();
    raw_value = Eigen::Vector3f(payload.X, payload.Y, payload.Z);
    value = coefficients.R * raw_value + coefficients.b;
    sem = true;
    log();
}

void Accelerometer::calibrate(Eigen::Vector3f sample) 
{
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    ss.precision(3);

    if (!calibration.calibrate(sample))
    {
        return;
    }
    coefficients = calibration.calculate();
    ss << "Accelerometer calibrated! Bias: " 
        << coefficients.b.format(commaFormat);

    Logger(LogType::CALIBRATION)(ss.str());
    was_calibrated = true;
    initialized = true;
}

AccelerometerCalibration::AccelerometerCalibration()
    :   readings{Eigen::Matrix<float,3,6>::Zero()},
        expected{Eigen::Matrix<float,3,6>::Zero()},
        statistic(500),
        logger(LogType::CALIBRATION),
        checked_sides{0}
{}

bool AccelerometerCalibration::calibrate(Eigen::Vector3f sample)
{
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    std::stringstream ss;
    ss.precision(3);

    if(statistic.push_sample(sample))
    {
        auto sd_max = statistic.sd().maxCoeff();
        if(sd_max < sd_limit)
        {
            save_mean(statistic.mean());
        }
        else
        {
            ss << "Accelerometer sd: " << statistic.sd().format(commaFormat);
            logger(ss.str());
        }
        statistic.reset();
    }
    return 0b00111111 == checked_sides; // All sides checked
}

AccelerometerCalibrationCoefficients AccelerometerCalibration::calculate()
{
    Eigen::Vector<float, 18> Y;
    Y << expected.col(0), expected.col(1), expected.col(2), expected.col(3), expected.col(4), expected.col(5);
    Eigen::MatrixXf X = Eigen::MatrixXf::Zero(18,12);
    for (size_t i = 0; i < 6; i++) 
    {
        X(i * 3, 0) = readings(0, i);
        X(i * 3, 1) = readings(1, i);
        X(i * 3, 2) = readings(2, i);
        X(i * 3, 3) = 1.0f;
        X(i * 3 + 1, 4) = readings(0, i);
        X(i * 3 + 1, 5) = readings(1, i);
        X(i * 3 + 1, 6) = readings(2, i);
        X(i * 3 + 1, 7) = 1.0f;
        X(i * 3 + 2, 8) = readings(0, i);
        X(i * 3 + 2, 9) = readings(1, i);
        X(i * 3 + 2, 10) = readings(2, i);
        X(i * 3 + 2, 11) = 1.0f;
    }
    Eigen::Vector<float, 12> THETA = (X.transpose() * X).ldlt().solve(X.transpose() * Y);
    Eigen::Matrix<float, 3, 4> A;
    A.row(0) = THETA.segment<4>(0).transpose();
    A.row(1) = THETA.segment<4>(4).transpose();
    A.row(2) = THETA.segment<4>(8).transpose();

    AccelerometerCalibrationCoefficients coefficients;
    coefficients.R = A.block<3,3>(0,0);
    coefficients.b = A.block<3,1>(0,3);
    return coefficients;
}

uint8_t AccelerometerCalibration::calc_sign(Eigen::Vector3f mean)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < 3; i++)
    {
        int sign_val = INVALID_SIGN;
        if (mean[i] > 0.8f*G)
        {
            sign_val = PLUS;
        }
        else if (mean[i] < -0.8f*G)
        {
            sign_val = MINUS;
        }
        else if (std::abs(mean[i]) < 0.1f*G )
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

    if (side == side::INVALID_SIDE)
    {
        Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
        std::stringstream ss;
        ss.precision(3);
        ss << "Accelerometer mean: " << mean.format(commaFormat);
        logger(ss.str());
        return;
    }

    switch(side)
    {
        case side::FRONT:
            readings.col(0) = mean;
            expected.col(0) = Eigen::Vector3f(G,0.0f, 0.0f);
            logger("Detected FRONT side");
            break;
        case side::BACK:
            readings.col(1) = mean;
            expected.col(1) = Eigen::Vector3f(-G,0.0f, 0.0f);
            logger("Detected BACK side");
            break;
        case side::LEFT:
            readings.col(2) = mean;
            expected.col(2) = Eigen::Vector3f(0.0f, G, 0.0f);
            logger("Detected LEFT side");
            break;
        case side::RIGHT:
            readings.col(3) = mean;
            expected.col(3) = Eigen::Vector3f(0.0f, -G, 0.0f);
            logger("Detected RIGHT side");
            break;
        case side::TOP:
            readings.col(4) = mean;
            expected.col(4) = Eigen::Vector3f(0.0f, 0.0f, G);
            logger("Detected TOP side");
            break;
        case side::BUTTOM:
            readings.col(5) = mean;
            expected.col(5) = Eigen::Vector3f(0.0f, 0.0f, -G);
            logger("Detected BUTTOM side");
            break;
        default:
            break;
    }

    checked_sides |= side;
}
