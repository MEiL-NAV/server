#include "FanucPosition.h"
#include <future>
#include <chrono>
#include <semaphore>

FanucPosition::FanucPosition(TimeSynchronizer & time_synchronizer, std::string ip_address, uint16_t port)
    :   Sensor(time_synchronizer, "fanuc_position", "time,X,Y,Z,W,P,R"),
        PeriodicEvent(200,false),
        socket(io_service)  ,
        msg_logger(LogType::FANUC)
{
    connect(ip_address, port);
}

FanucPosition::~FanucPosition() 
{
    std::scoped_lock lock(socket_mtx);
    close();
}

void FanucPosition::connect(std::string ip_address, uint16_t port)
{
    std::scoped_lock lock(socket_mtx);
    close();
    try
    {
        socket.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ip_address), port));
        auto reply = recv_async();
        if (!reply.has_value() || reply.value() != "success")
        {
            throw std::runtime_error("No reply on connect");
        }
        PeriodicEvent::start_periodic_task();
        msg_logger("Connected to fanucpy");
    }
    catch(const boost::system::system_error& e)
    {
        close();
        msg_logger(e.what());
    }   
}

void FanucPosition::log() 
{
    Eigen::VectorXf log(7);
    log(0) = last_update;
    log.segment<3>(1) =  value;
    log.segment<3>(4) =  orientation; 
    logger << log;
}

void FanucPosition::periodic_event()
{
    io_service.reset();
    std::scoped_lock lock(socket_mtx);
    if(!socket.is_open())
    {
        msg_logger("Socket is not open");
        return;
    }

    msg_logger("Requesting current position");
    if(!send_async("curpos"))
    {
        return;
    }

    msg_logger("Waiting for reply");
    auto reply = recv_async();
    if (reply.has_value())
    {
        parse_message(reply.value());
    }
}

void FanucPosition::close() 
{
    PeriodicEvent::stop_periodic_task();
    if (socket.is_open())
    {
        msg_logger("Closing connection to fanucpy");
        socket.close();
    }
}

bool FanucPosition::send_async(std::string msg)
{
    if (msg.empty())
    {
        return false;
    }

    if (msg.back() != '\n')
    {
        msg.push_back('\n');
    }
    
    auto send_future = std::async(std::launch::async, [&] () -> bool
        {
            try
            {
                //send message
                boost::asio::write(socket, boost::asio::buffer(msg));
            }
            catch(const boost::system::system_error& e)
            {
                msg_logger(e.what());
                close();
                return false;
            }
            return true;
        }
    );
    
    switch (auto status = send_future.wait_for(std::chrono::milliseconds{ 200 }); status)
    {
        case std::future_status::deferred:
        case std::future_status::timeout:
            close();
            return false;
        case std::future_status::ready:
            break;
    }

    if(!send_future.get())
    {
        close();
        return false;
    }

    return true;
}

std::optional<std::string> FanucPosition::recv_async()
{
    auto recv_future = std::async(std::launch::async, [&] () -> std::string
        {
            try
            {
                //read next message
                boost::asio::streambuf response;
                std::binary_semaphore sem{0};
                boost::asio::async_read(socket, response,
                    boost::asio::transfer_at_least(1),
                    [&]
                    (const boost::system::error_code& ec,
                     [[maybe_unused]] std::size_t bytes_transferred) 
                    {
                        if (ec)
                        {
                            msg_logger(ec.message()); // Print the human-readable error message
                        }
                        sem.release();
                    });
                io_service.run_one();
                sem.acquire();
                std::string message = boost::asio::buffer_cast<const char*>(response.data());
                return message;
            }
            catch(const boost::system::system_error& e)
            {
                if(e.code() == boost::asio::error::eof)
                {
                    msg_logger("Connection closed by fanucpy");
                    close();
                    return "";
                }
                msg_logger(e.what());
                return "";
            }
        }
    );
    
    switch (auto status = recv_future.wait_for(std::chrono::milliseconds{ 200 }); status)
    {
        case std::future_status::deferred:
        case std::future_status::timeout:
            msg_logger("Timeout");
            socket.cancel();
            break;
        case std::future_status::ready:
            auto msg = recv_future.get();
            if (msg.empty())
            {
                close();
                return std::nullopt;
            }
            if (msg[0] == '0' && msg[1] == ':')
            {
                return msg.substr(2);
            }
            break;
    }
    return std::nullopt;
}

void FanucPosition::parse_message(std::string message)
{
    std::vector<std::string> result;
    std::stringstream s_stream(message);
    while(s_stream.good()) {
        std::string substr;
        getline(s_stream, substr, ',');
        result.push_back(substr);
    }
    if (result.size() != 6)
    {
        msg_logger("Invalid message: " + message);
        return;
    }
    char prefix[] = {'x', 'y', 'z', 'w', 'p', 'r'};
    Eigen::Vector<float, 6> values;

    for (size_t i = 0; i < 6; i++)
    {
        if(result[i].empty() || result[i][0] != prefix[i] || result[i][1] != '=')
        {
            msg_logger("Invalid message: " + message);
            return;
        }

        values[i] = std::stof(result[i].substr(2));
    }
    
    std::scoped_lock lock(value_mutex);
    raw_value = values.head<3>();
    value = values.head<3>();
    orientation = values.tail<3>();
    sem = true;
    log();
}
