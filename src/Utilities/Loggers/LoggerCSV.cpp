#include "LoggerCSV.h"
#include "Logger.h"

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
    static Eigen::IOFormat commaFormat(Eigen::FullPrecision, Eigen::DontAlignCols," ",",");
    if (ofs.is_open())
    {
        ofs << vec.format(commaFormat) << std::endl;
    }
}
