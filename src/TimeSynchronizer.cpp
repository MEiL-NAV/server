#include "TimeSynchronizer.h"
#include <iostream>
#include <cassert>
#include "Utilities/Millis.h"
#include "Utilities/Loggers/Logger.h"

TimeSynchronizer::TimeSynchronizer(uint32_t period_millis,
                                 uint16_t broadcast_port) 
    : PeriodicEvent(period_millis), udp_broadcaster(broadcast_port)
{

}

std::optional<int32_t> TimeSynchronizer::get_offset(uint8_t node_id)
{ 
  std::lock_guard<std::mutex> lck(mtx);
  if(offset_table.contains(node_id))
  {
    return offset_table.at(node_id);
  }
  return std::nullopt;
}

void TimeSynchronizer::consumeMessage(const Message &msg) 
{
	static Logger logger(LogType::SYNC);

    assert(msg.command == Command::TIMESYNC);
    auto millis = Millis::get();
    uint32_t ping;
    auto payload = std::get<TimeSyncPacket>(msg.payload);
    {
        std::lock_guard<std::mutex> lck(mtx);
        if (payload.sync_id != std::get<0>(last_sync_time)) 
        {
            return;
        }
        ping = millis - std::get<1>(last_sync_time);
        offset_table[msg.node_id] =
        static_cast<int32_t>((millis + std::get<1>(last_sync_time)) / 2U) -
        static_cast<int32_t>(payload.time);
    }
    logger(std::string("Node ") + std::to_string(msg.node_id) + " offset set to "
      	+ std::to_string(offset_table[msg.node_id]) + " ms. Ping: " + std::to_string(ping) + "ms");
}

void TimeSynchronizer::periodic_event() 
{
    auto millis = Millis::get();
    {
        std::lock_guard<std::mutex> lck(mtx);
        last_sync_time = {sync_id, millis};
    }
    std::array<uint8_t,2> message;
    message[0] = 'S';
    message[1] = sync_id;
    udp_broadcaster.send(message);
    sync_id++;
}