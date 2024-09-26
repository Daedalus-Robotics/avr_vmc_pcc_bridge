#pragma once

#include <cstdint>
#include <functional>

#include "crc15can.hpp"
#include "endianness.hpp"

#define MESSAGE_BUF_LEN sizeof(message_t)
#define SOL_NUM 0b1111111111111111

#pragma pack(1)
struct message_t {
    uint16_t sol;
    uint16_t identifier; // The id or topic of the message
    uint8_t data[32]; // The data payload of the message (32 bytes)
    uint16_t crc; // The CRC 15 checksum of the data for error checking with an extra bit because why not
} __attribute__((packed));
#pragma pack()

inline uint16_t get_crc(const message_t *message) {
    return byte_swap<host_endian, little_endian>(
        crc15can_byte(0, message->data, MESSAGE_BUF_LEN)
    );
}

inline void append_crc(message_t *message) {
    message->crc = get_crc(message);
}
