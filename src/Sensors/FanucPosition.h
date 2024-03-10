#include "Sensor.h"
#include "../TimeSynchronizer.h"
#include <eigen3/Eigen/Dense>
#include <memory>
#include "../Utilities/PeriodicEvent.h"
#include "../Utilities/Loggers/LoggerCSV.h"
#include <boost/asio.hpp>
#include <optional>

class FanucPosition : public Sensor<Eigen::Vector3f>, public PeriodicEvent
{
public:
    FanucPosition(TimeSynchronizer& time_synchronizer);
    virtual ~FanucPosition();

    void consumeMessage([[maybe_unused]] const Message& msg) override {};

    void connect(std::string ip_address, uint16_t port);

protected:
    void periodic_event() override;
    void close();

    std::optional<std::string> recv_async();
    void parse_message(std::string message);

    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;
    std::mutex socket_mtx;

    Logger logger;
};