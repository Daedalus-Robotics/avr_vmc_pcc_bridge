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
}