#pragma once

#include "endianness.hpp"

#define TOPIC_CONVERT(TOPIC) byte_swap<host_endian, little_endian>((uint16_t) TOPIC)

// Request topic (host-mcu)
#define TOPIC_RESET 0
#define TOPIC_ESTOP 1

#define TOPIC_ONBOARD_LED_SET 2

#define TOPIC_LED_STRIP_SET 3
#define TOPIC_LED_STRIP_FILL 4
#define TOPIC_LED_STRIP_MODE 5

#define TOPIC_SERVO_ENABLE 6
#define TOPIC_SERVO_SET 7
#define TOPIC_SERVO_SPAN 8

// Response topic (mcu-host)
#define TOPIC_ERROR 0
#define TOPIC_STATUS 1

#define TOPIC_THERMAL_ROW_0 2
#define TOPIC_THERMAL_ROW_1 3
#define TOPIC_THERMAL_ROW_2 4
#define TOPIC_THERMAL_ROW_3 5
#define TOPIC_THERMAL_ROW_4 6
#define TOPIC_THERMAL_ROW_5 7
#define TOPIC_THERMAL_ROW_6 8
#define TOPIC_THERMAL_ROW_7 9

// Error Codes
#define ERROR_BAD_TOPIC 0
#define ERROR_COMPONENT_FAILED 1

// Status Codes
#define STATUS_READY 0
#define STATUS_RESET 1
#define STATUS_HALT 2

// Component Codes
#define COMPONENT_THERMAL 0
#define COMPONENT_SERVOS 1

// Error Types
#define ERROR_TYPE 0
#define ERROR_TYPE_RESOLVED 1
