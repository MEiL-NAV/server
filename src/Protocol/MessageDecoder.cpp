#include "MessageDecoder.h"
#include <iostream>
#include <cstring>

MessageDecoder::MessageDecoder() 
{
}

std::unique_ptr<Message> MessageDecoder::decode(uint8_t *buffer,
                                                ssize_t buffer_size) {
  uint16_t crc = (buffer[buffer_size - 1] << 8) | (buffer[buffer_size - 2]);
  std::memcpy(&crc,&(buffer[buffer_size - 2]), sizeof(uint16_t));
  if (!crc16.checkCRC(buffer, buffer_size - sizeof(uint16_t), crc)) {
    std::cerr << "Invalid CRC!" << std::endl;
    return nullptr;
  }

  auto message = std::make_unique<Message>();
  message->node_id = buffer[0];
  message->command = static_cast<Command>(buffer[1]);
  message->payload = parsePayload(
    message->command,
    buffer + 2 * sizeof(uint8_t), 
    buffer_size - 2 * sizeof(uint8_t) - sizeof(uint16_t)
    );
  return message;
}

template<typename T>
T parse(uint8_t *buffer, size_t buffer_size)
{
    T payload;
    std::memcpy(&payload, buffer,
        std::min(buffer_size, static_cast<size_t>(sizeof(T))));
    return payload;
}

Payload MessageDecoder::parsePayload(Command command, uint8_t *buffer,
                                     size_t buffer_size) 
{
    switch(command)
    {
        case Command::TIMESYNC:
            return parse<TimeSyncPacket>(buffer,buffer_size); 
        break;
        case Command::DUMMY:
            return parse<DummySensorPacket>(buffer,buffer_size);
        break;
        case Command::ACCELEROMETER_READING:
        case Command::GYROSCOPE_READING:
        case Command::POSITION_READING:
            return parse<VectorPacket>(buffer,buffer_size);
        break;
    }
    throw std::runtime_error("Unknown command");
}
