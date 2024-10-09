#pragma once

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <avr_vmc_pcc_interfaces/msg/thermal_frame.hpp>

#include "pcc_bridge/comm.hpp"

#define THERMAL_FRAME_SIZE 8

namespace pcc_bridge {

    class ThermalComponent {
    public:
        ThermalComponent();
        void setup(rclcpp::Node *node, std::function<void(message_t *)> sendMessageFunc);

        void onRowUpdate(uint8_t row, float data[THERMAL_FRAME_SIZE]);

        private:
            rclcpp::Publisher<avr_vmc_pcc_interfaces::msg::ThermalFrame>::SharedPtr framePublisher;
            avr_vmc_pcc_interfaces::msg::ThermalFrame frameMessage;
    };

}
