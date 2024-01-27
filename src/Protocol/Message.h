#pragma once

#include <cstdint>
#include <variant>
#include "Payload.h"


enum Command : uint8_t
{
    TIMESYNC                = 1,
    DUMMY                   = 2,
    ACCELEROMETER_READING   = 3,
    GYROSCOPE_READING       = 4
};

typedef std::variant<TimeSyncPacket,DummySensorPacket, VectorPacket> Payload;

struct Message
{
    uint8_t node_id;
    Command command;
    Payload payload;
};
