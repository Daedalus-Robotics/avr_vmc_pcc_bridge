#pragma once

#include <memory>
#include <string>
#include <functional>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <avr_common_interfaces/srv/set_color.hpp>
#include <avr_vmc_pcc_interfaces/srv/set_led_effect.hpp>

#include "pcc_bridge/comm.hpp"

namespace pcc_bridge {

    class LedComponent {
    public:
        LedComponent();
        void setup(rclcpp::Node *node, std::function<void(message_t *)> sendMessageFunc);

        void sendUpdate();

    private:
        uint8_t red;
        uint8_t green;
        uint8_t blue;

        std::function<void(message_t *)> sendMessage;

        rclcpp::Service<avr_common_interfaces::srv::SetColor>::SharedPtr setService;
        rclcpp::Service<avr_vmc_pcc_interfaces::srv::SetLedEffect>::SharedPtr effectService;

        void setCallback(const std::shared_ptr<avr_common_interfaces::srv::SetColor::Request> request,
                         std::shared_ptr<avr_common_interfaces::srv::SetColor::Response> _);
        void effectCallback(const std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetLedEffect::Request> request,
                            std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetLedEffect::Response> _);
    };

}
