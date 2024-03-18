#include "FanucPosition.h"
#include <future>
#include <chrono>
#include <semaphore>

FanucPosition::FanucPosition(TimeSynchronizer & time_synchronizer)
    :   Sensor(time_synchronizer, "fanuc_position", "time,X,Y,Z,raw_X,raw_Y,raw_Z"),
        PeriodicEvent(1000,false),
        socket(io_service)  ,
        logger(LogType::FANUC)
{
    connect("127.0.0.1", 18736);
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
        logger("Connected to fanucpy");
    }
    catch(const boost::system::system_error& e)
    {
        close();
        logger(e.what());
    }   
}

void FanucPosition::periodic_event()
{
    io_service.reset();
    std::scoped_lock lock(socket_mtx);
    if(!socket.is_open())
    {
        logger("Socket is not open");
        return;
    }

    auto send_future = std::async(std::launch::async, [&] () -> bool
        {
            try
            {
                //send message "curpos"
                std::string message = "curpos";
                boost::asio::write(socket, boost::asio::buffer(message));
            }
            catch(const boost::system::system_error& e)
            {
                logger(e.what());
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
            return;
        case std::future_status::ready:
            break;
    }

    if(!send_future.get())
    {
        close();
        return;
    }

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
        socket.close();
    }
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
                    [&sem]
                    ([[maybe_unused]] const boost::system::error_code& ec,
                     [[maybe_unused]] std::size_t bytes_transferred) 
                    {
                        sem.release();
                    });
                io_service.run();
                sem.acquire();
                std::string message = boost::asio::buffer_cast<const char*>(response.data());
                return message;
            }
            catch(const boost::system::system_error& e)
            {
                if(e.code() == boost::asio::error::eof)
                {
                    logger("Connection closed by fanucpy");
                    close();
                    return "";
                }
                logger(e.what());
                return "";
            }
        }
    );
    
    switch (auto status = recv_future.wait_for(std::chrono::milliseconds{ 200 }); status)
    {
        case std::future_status::deferred:
        case std::future_status::timeout:
            logger("Timeout");
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
        logger("Invalid message: " + message);
        return;
    }
    char prefix[] = {'x', 'y', 'z'};
    Eigen::Vector3f position;

    for (size_t i = 0; i < 3; i++)
    {
        if(result[i].empty() || result[i][0] != prefix[i] || result[i][1] != '=')
        {
            logger("Invalid message: " + message);
            return;
        }
        position[i] = std::stof(result[i].substr(2));
    }
    
    std::scoped_lock lock(value_mutex);
    raw_value = position;
    value = position;
    sem = true;
    log();
}
