#pragma once

#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pcc_bridge/comm.hpp"

#include "pcc_bridge/led_component.hpp"
#include "pcc_bridge/servo_component.hpp"

namespace pcc_bridge
{
    class PCCBridgeNode : public rclcpp::Node
    {
    public:
        explicit PCCBridgeNode(const rclcpp::NodeOptions &);
        void setup();

        void readLoop();

    private:
        std::atomic<bool> connected;

        serial::Serial port;
        std::vector<uint8_t> dataQueue;

        rclcpp::TimerBase::SharedPtr readTimer;

        LedComponent ledComponent;
        ServoComponent servoComponent;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;

        void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> _,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> _);

        void openPort();
        void sendMessage(message_t *message);
    };
}
