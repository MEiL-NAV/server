#include "Logger.h"
#include <filesystem>

int Logger::log_mask = -1;
std::unique_ptr<Logger::FileAccessor> Logger::file_accessor = nullptr;
std::string Logger::log_path = "../logs/";
std::string Logger::run_counter_path = "../logs/run_counter";

Logger &Logger::prefix()
{
    auto prefix = get_prefix();
    if(log_mask & log_type)
    {
        std::cout << prefix;
    }
    std::scoped_lock lck(file_accessor->mtx);
    if(file_accessor->file.is_open())
    {
        file_accessor->file << prefix;
    }
    return *this;
}

const char *Logger::get_prefix()
{
    switch (log_type)
    {
    case LogType::CALIBRATION:
        return "[CALIBRATION] ";
    case LogType::INFO:
        return "[INFO] ";
    case LogType::DEBUG:
        return "[DEBUG] ";
    case LogType::ERROR:
        return "[ERROR] ";
    case LogType::SYNC:
        return "[SYNC] ";
    default:
        return "";
    }
}

void Logger::init_file() 
{
    if (file_accessor != nullptr)
    {
        return;
    }

    int run_number = 1;

    std::error_code ec;
    std::filesystem::create_directory(log_path, ec);

    if (std::filesystem::exists(run_counter_path))
    {
        std::ifstream ifs(run_counter_path);
        ifs >> run_number;
        ifs.close();
    }
    std::ofstream ofs(run_counter_path, std::ofstream::trunc);
    ofs << (run_number + 1);
    ofs.close();

    std::string filename = log_path + std::to_string(run_number) + ".log";
    file_accessor = std::make_unique<FileAccessor>();
    file_accessor->file.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
}
