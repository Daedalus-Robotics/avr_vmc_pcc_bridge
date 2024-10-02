#pragma once

#include <cstdint>
#include <functional>

#include "crc15can.hpp"
#include "endianness.hpp"

#define MESSAGE_DATA_LEN 32
#define MESSAGE_BUF_LEN (static_cast<int>(sizeof(message_t)))
#define SOL_NUM 0b1111111111111111

#pragma pack(1)
struct message_t {
    uint16_t sol;
    uint16_t identifier; // The id or topic of the message
    uint8_t data[MESSAGE_DATA_LEN]; // The data payload of the message (32 bytes)
    uint16_t crc; // The CRC 15 checksum of the data for error checking with an extra bit because why not
} __attribute__((packed));
#pragma pack()

inline uint16_t get_crc(const message_t *message) {
    return crc15can_byte(0, message->data, MESSAGE_DATA_LEN);
}

inline void append_crc(message_t *message) {
    message->crc = get_crc(message);
}

inline bool checkCrc(const message_t *message) {
    printf("topic: %u\n", message->identifier, get_crc(message));
    printf("in-message: %u, checked: %u\n", message->crc, get_crc(message));
    return message->crc == get_crc(message);
}
