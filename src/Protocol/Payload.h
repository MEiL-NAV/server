#pragma once

#include <cstdint>

struct __attribute__((__packed__)) TimeSyncPacket
{
    uint8_t sync_id;
    uint32_t time;
};

struct __attribute__((__packed__)) DummySensorPacket
{
    uint32_t counter;
};

struct __attribute__((__packed__)) VectorPacket
{
    uint32_t time;
    float X;
    float Y;
    float Z;
};