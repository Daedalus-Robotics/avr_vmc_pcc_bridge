#include "pcc_bridge/servo_component.hpp"

#include "pcc_bridge/topics.h"

namespace pcc_bridge {
    ServoComponent::ServoComponent() : enabled(true), microsecondsArr(),
                                       sendMessage() {}

    void ServoComponent::setup(rclcpp::Node *node, std::function<void(message_t *)> sendMessageFunc) {
        sendMessage = sendMessageFunc;
        enabledService = node->create_service<std_srvs::srv::SetBool>(
            "set_servo_enabled",
            [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                enabledCallback(request, response);
            }
        );
        setService = node->create_service<avr_vmc_pcc_interfaces::srv::SetServo>(
            "set_servo",
            [this](std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetServo::Request> request, std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetServo::Response> response) {
                setCallback(request, response);
            }
        );
    }

    void ServoComponent::sendEnabledUpdate() {
        message_t message;
        message.identifier = TOPIC_CONVERT(TOPIC_SERVO_ENABLE);
        message.data[0] = enabled;

        sendMessage(&message);
    }

    void ServoComponent::sendSingleUpdate(uint8_t num) {
        message_t message;
        message.identifier = TOPIC_CONVERT(TOPIC_SERVO_SET);
        message.data[0] = num;

        auto microsecondsPtr = (uint16_t *) &message.data[1];
        printf("Value: %ul \n", microsecondsArr[num]);
        *microsecondsPtr = byte_swap<host_endian, little_endian>(microsecondsArr[num]);
        //*microsecondsPtr = microsecondsArr[num];

        sendMessage(&message);
    }

    void ServoComponent::sendUpdate() {
        sendEnabledUpdate();
        for (size_t i = 0; i < 8; i++) {
            sendSingleUpdate(i);
        }
    }

    void ServoComponent::enabledCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        enabled = request->data;
        sendEnabledUpdate();

        response->success = true;
        response->message = "Success";
    }

    void ServoComponent::setCallback(const std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetServo::Request> request,
                                     std::shared_ptr<avr_vmc_pcc_interfaces::srv::SetServo::Response> _) {
        microsecondsArr[request->servo] = request->microseconds;
        sendSingleUpdate(request->servo);
    }
}
