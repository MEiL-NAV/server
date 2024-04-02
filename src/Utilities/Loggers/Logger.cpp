#include "Logger.h"
#include <filesystem>
#include <thread>
#include "../Config/Config.h"

int Logger::log_mask = -1;
std::unique_ptr<Logger::FileAccessor> Logger::file_accessor = nullptr;
std::unique_ptr<zmq::socket_t> Logger::logger_socket = nullptr;

void Logger::operator()(const std::string &msg)
{
    std::string msg_with_prefix = get_prefix() + msg;
    if(log_mask & log_type)
    {
        std::cout << msg_with_prefix << std::endl;
    }

    if(logger_socket != nullptr)
    {
        std::string message_zmq = std::string("c:") + msg_with_prefix;
        zmq::message_t zmq_msg(message_zmq.data(),message_zmq.size());
        logger_socket->send(zmq_msg, zmq::send_flags::none);
    }
    
    std::scoped_lock lck(file_accessor->mtx);
    if(file_accessor->file.is_open())
    {
        file_accessor->file << msg_with_prefix << std::endl;;
    }
}

void Logger::set_ctx(zmq::context_t &ctx)
{
    auto config = Config::get_singleton();
    logger_socket = std::make_unique<zmq::socket_t>(ctx, zmq::socket_type::pub);
    logger_socket->bind(config.logger_address);
    logger_socket->send(zmq::message_t(), zmq::send_flags::none);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void Logger::set_mask(int new_mask)
{
    log_mask = new_mask;
}

void Logger::deconstruct()
{
    if(logger_socket != nullptr)
    {
        logger_socket->close();
    }
    file_accessor->file.close();
}


std::string Logger::get_log_dir() {
  init_file();
  std::scoped_lock lck(file_accessor->mtx);
  return file_accessor->log_dir_path;
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
    case LogType::FANUC:
        return "[FANUC] ";
    case LogType::MODBUS:
        return "[MODBUS] ";
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

    auto config = Config::get_singleton();

    int run_number = 1;

    std::error_code ec;
    std::filesystem::create_directory(config.log_path, ec);

    if (std::filesystem::exists(config.run_counter_path))
    {
        std::ifstream ifs(config.run_counter_path);
        ifs >> run_number;
        ifs.close();
    }
    std::ofstream ofs(config.run_counter_path, std::ofstream::trunc);
    ofs << (run_number + 1);
    ofs.close();

    std::filesystem::create_directory(config.log_path + std::to_string(run_number), ec);

    file_accessor = std::make_unique<FileAccessor>();
    std::scoped_lock lck(file_accessor->mtx);
    file_accessor->log_dir_path = config.log_path + std::to_string(run_number) + "/";
    file_accessor->file.open((file_accessor->log_dir_path + "server.log").c_str(), std::ofstream::out | std::ofstream::trunc);

    set_mask(config.log_mask);
}
