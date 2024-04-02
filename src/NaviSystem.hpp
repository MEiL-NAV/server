#include "Communication/UDPListener.h"
#include "Protocol/Message.h"
#include "TimeSynchronizer.h"
#include "Sensors/Accelerometer.h"
#include "Sensors/Gyroscope.h"
#include "Sensors/FanucPosition.h"
#include "Sensors/PositionProvider.h"
#include "Utilities/PeriodicEvent.h"
#include <zmq.hpp>
#include "EKF/EKF_IMU.h"
#include "Utilities/Config/Config.h"
#include "Plugins/Force/ForceLogger.h"

class NaviSystem : public PeriodicEvent
{
public:
    NaviSystem(zmq::context_t& ctx, const Config& config);
    virtual ~NaviSystem();

protected:
    void periodic_event() override;

private:
    UDPListener udp_listener;
    TimeSynchronizer time_synchronizer;
    EKF_IMU ekf;

    // Sensors
    Accelerometer accelerometer;
    Gyroscope gyroscope;
    PositionProvider position_provider;
    FanucPosition fanuc_position;
    ForceLogger force_logger;

    // ZMQ
    zmq::context_t& ctx;
    zmq::socket_t status_sock;
    static constexpr const char* status_address = "tcp://*:5555";

    void messageHandler(const Message& msg);
    void send_status();
    void set_EKF_parameters();
};