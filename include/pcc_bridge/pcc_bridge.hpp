#pragma once

#include <mutex>
#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pcc_bridge/comm.hpp"

#include "pcc_bridge/led_component.hpp"
#include "pcc_bridge/servo_component.hpp"
#include "pcc_bridge/thermal_component.hpp"

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
        std::mutex isRunningMutex;
        std::mutex serialSendMutex;

        serial::Serial port;
        std::vector<uint8_t> dataQueue;

        rclcpp::TimerBase::SharedPtr readTimer;
        rclcpp::TimerBase::SharedPtr syncTimer;

        LedComponent ledComponent;
        ServoComponent servoComponent;
        ThermalComponent thermalComponent;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr eStopSubscriber;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;

        void eStopCallback(const std::shared_ptr<std_msgs::msg::Empty> _msg);
        void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> _rq,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> _rs);

        void openPort();
        void sendMessage(message_t *message);
        void sendFullUpdate();
    };
}
