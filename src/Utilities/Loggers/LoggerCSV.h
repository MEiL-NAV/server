#pragma once

#include <eigen3/Eigen/Dense>
#include <fstream>

class LoggerCSV
{
public:
    LoggerCSV(std::string filename, std::string header);
    ~LoggerCSV();

    void operator<<(const Eigen::VectorXf& vec);
    void operator()(uint32_t time, const Eigen::VectorXf& vec);

private:
    std::ofstream ofs;

    static Eigen::IOFormat commaFormat;
};