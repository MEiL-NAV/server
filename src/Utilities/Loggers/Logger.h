#pragma once

#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>
#include <zmq.hpp>

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
    
    void operator()(const std::string& msg);
    
    static void set_ctx(zmq::context_t& ctx);
    static void set_mask(int new_mask);
    static void deconstruct();
    static std::string get_log_dir();

private:
    LogType log_type;

    struct FileAccessor
    {
        std::mutex mtx;
        std::string log_dir_path;
        std::ofstream file;
    };
    static int log_mask;
    static std::unique_ptr<FileAccessor> file_accessor;

    static std::unique_ptr<zmq::socket_t> logger_socket;

    const char* get_prefix();
    static void init_file();
};
