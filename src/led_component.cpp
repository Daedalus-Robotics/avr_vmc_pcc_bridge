#include "pcc_bridge/led_component.hpp"

#include "pcc_bridge/topics.h"

namespace pcc_bridge {
    LedComponent::LedComponent() : red(), green(), blue(),
                                   sendMessage() {}

    void LedComponent::setup(rclcpp::Node *node, std::function<void(message_t *)> sendMessageFunc) {
        sendMessage = sendMessageFunc;
        setService = node->create_service<avr_common_interfaces::srv::SetColor>(
            "set_onboard_led",
            [this](std::shared_ptr<avr_common_interfaces::srv::SetColor::Request> request, std::shared_ptr<avr_common_interfaces::srv::SetColor::Response> response) {
                setCallback(request, response);
            }
        );
        effectService = node->create_service<avr_vmc_pcc_interfaces::srv::SetLedEffect>(
            "set_led_effect",
            [this](std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetLedEffect::Request> request, std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetLedEffect::Response> response) {
                effectCallback(request, response);
            }
        );
    }

    void LedComponent::sendUpdate() {
        message_t message;
        message.identifier = TOPIC_ONBOARD_LED_SET;
        message.data[0] = red;
        message.data[1] = green;
        message.data[2] = blue;
        sendMessage(&message);
    }

    void LedComponent::setCallback(const std::shared_ptr<avr_common_interfaces::srv::SetColor::Request> request,
                                   __attribute__((unused)) std::shared_ptr<avr_common_interfaces::srv::SetColor::Response> _) {
        red = request->color.r;
        green = request->color.g;
        blue = request->color.b;

        sendUpdate();
    }

    void LedComponent::effectCallback(const std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetLedEffect::Request> request,
                                      __attribute__((unused)) std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetLedEffect::Response> _) {
        message_t message;
        message.identifier = TOPIC_LED_STRIP_MODE;
        message.data[0] = request->mode;
        message.data[1] = request->r;
        message.data[2] = request->g;
        message.data[3] = request->b;
        message.data[4] = request->arg;
        sendMessage(&message);
    }
}