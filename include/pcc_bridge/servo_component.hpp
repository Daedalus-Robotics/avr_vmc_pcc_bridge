#pragma once

#include <memory>
#include <string>
#include <functional>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <avr_vmc_pcc_interfaces/srv/set_servo.hpp>
#include <avr_vmc_pcc_interfaces/srv/span_servos.hpp>

#include "pcc_bridge/comm.hpp"

namespace pcc_bridge {

    class ServoComponent {
    public:
        ServoComponent();
        void setup(rclcpp::Node *node, std::function<void(message_t *)> sendMessageFunc);

        void sendEnabledUpdate();
        void sendSingleUpdate(uint8_t num);
        void sendUpdate();

    private:
        bool enabled;
        uint16_t microsecondsArr[8];

        std::function<void(message_t *)> sendMessage;

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enabledService;
        rclcpp::Service<avr_vmc_pcc_interfaces::srv::SetServo>::SharedPtr setService;
        rclcpp::Service<avr_vmc_pcc_interfaces::srv::SpanServos>::SharedPtr spanService;

        void enabledCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> _);
        void setCallback(const std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetServo::Request> request,
                         std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetServo::Response> _);
        void spanCallback(const std::shared_ptr<avr_vmc_pcc_interfaces::srv::SpanServos::Request> request,
                          std::shared_ptr<avr_vmc_pcc_interfaces::srv::SpanServos::Response> _);
    };

}
