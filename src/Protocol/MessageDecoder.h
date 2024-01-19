#pragma once

#include <cstdint>
#include <unistd.h>
#include "Message.h"
#include <memory>
#include "CRC16/CRC16.h"

class MessageDecoder
{
public:
    MessageDecoder();

    std::unique_ptr<Message> decode(uint8_t* buffer, ssize_t buffer_size);
    
    static constexpr ssize_t max_datagram_size = 20U;

private:
    Payload parsePayload(Command command, uint8_t* buffer, size_t buffer_size);

    CRC16 crc16;
};
