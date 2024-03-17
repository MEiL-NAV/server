#include "LoggerCSV.h"
#include "Logger.h"

Eigen::IOFormat LoggerCSV::commaFormat = Eigen::IOFormat{Eigen::FullPrecision, Eigen::DontAlignCols," ",","};

LoggerCSV::LoggerCSV(std::string filename, std::string header)
    : ofs(Logger::get_log_dir() + filename + ".csv", std::ofstream::out | std::ofstream::trunc)
{
    ofs << header << std::endl;
}

LoggerCSV::~LoggerCSV() 
{
    ofs.close();
}

void LoggerCSV::operator<<(const Eigen::VectorXf &vec)
{
    if (ofs.is_open())
    {
        ofs << vec.format(commaFormat) << std::endl;
    }
}

void LoggerCSV::operator()(uint32_t time, const Eigen::VectorXf &vec) 
{
    if (ofs.is_open())
    {
        ofs << time << ',' << vec.format(commaFormat) << std::endl;
    }
}
