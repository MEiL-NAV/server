#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>
#include <memory>
#include "../Utilities/PeriodicEvent.h"
#include "../Utilities/Loggers/LoggerCSV.h"
#include <boost/asio.hpp>
#include <optional>

class FanucPosition : public Sensor<Eigen::Vector3f>, protected PeriodicEvent
{
public:
    FanucPosition(TimeSynchronizer& time_synchronizer, std::string ip_address, uint16_t port = 18736);
    virtual ~FanucPosition();

    void consumeMessage([[maybe_unused]] const Message& msg) override {};

    void connect(std::string ip_address, uint16_t port);

    void log() override;

protected:
    void periodic_event() override;
    void close();

    bool send_async(std::string msg);
    std::optional<std::string> recv_async();
    void parse_message(std::string message);

    Eigen::Vector3f orientation;

    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;
    std::mutex socket_mtx;

    Logger msg_logger;
};