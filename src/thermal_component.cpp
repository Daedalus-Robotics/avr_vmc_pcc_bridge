#include "pcc_bridge/thermal_component.hpp"

namespace pcc_bridge {
    ThermalComponent::ThermalComponent() : frameMessage() {}

    void ThermalComponent::setup(rclcpp::Node *node, __attribute__((unused)) std::function<void(message_t *)> sendMessageFunc) {
        framePublisher = node->create_publisher<avr_vmc_pcc_interfaces::msg::ThermalFrame>("thermal/raw", 10);

        frameMessage.step = THERMAL_FRAME_SIZE;
        frameMessage.data.resize(THERMAL_FRAME_SIZE * THERMAL_FRAME_SIZE, 0);
    }

    void ThermalComponent::onRowUpdate(uint8_t row, float data[THERMAL_FRAME_SIZE]) {
        size_t offset = row * THERMAL_FRAME_SIZE;
        for (size_t i = 0; i < THERMAL_FRAME_SIZE; i++) {
            frameMessage.data[offset + i] = data[i];
        }
    }
}
