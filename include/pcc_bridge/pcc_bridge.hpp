#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include "pcc_bridge/comm.hpp"

#include "pcc_bridge/led_component.hpp"

namespace pcc_bridge
{
    class PCCBridgeNode : public rclcpp::Node
    {
    public:
        explicit PCCBridgeNode(const rclcpp::NodeOptions &);
        void setup();

        void readLoop();

    private:
        serial::Serial port;
        std::vector<uint8_t> dataQueue;

        rclcpp::TimerBase::SharedPtr readTimer;

        LedComponent ledComponent;

        void openPort();
        void sendMessage(message_t *message);
    };
}
