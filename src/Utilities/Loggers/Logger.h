#pragma once

#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>

enum LogType : int 
{
    INFO          = 1 << 0,
    DEBUG         = 1 << 1,
    ERROR         = 1 << 2,
    CALIBRATION   = 1 << 3,
    SYNC          = 1 << 4,
};

class Logger
{
public:
    Logger(LogType log_type)
        :   log_type{log_type}
    {
        init_file();
    }

    Logger& prefix();

    template<typename T>
    Logger& operator<<(const T& obj)
    {
        if(log_mask & log_type)
        {
            std::cout << obj;
        }
        std::scoped_lock lck(file_accessor->mtx);
        if(file_accessor->file.is_open())
        {
            file_accessor->file << obj;
        }
        return *this;
    }

    static void set_mask(int new_mask)
    {
        log_mask = new_mask;
    }

    static std::string get_log_dir();

private:
    LogType log_type;

    struct FileAccessor
    {
        std::mutex mtx;
        std::string log_dir_path;
        std::ofstream file;
    };

    static std::string log_path;
    static std::string run_counter_path;

    static int log_mask;
    static std::unique_ptr<FileAccessor> file_accessor;

    const char* get_prefix();
    static void init_file();
};
