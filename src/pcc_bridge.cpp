#include "pcc_bridge/pcc_bridge.hpp"

#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

#include "pcc_bridge/topics.h"

using namespace boost::filesystem;
using namespace std::chrono_literals;


namespace pcc_bridge {
    PCCBridgeNode::PCCBridgeNode(const rclcpp::NodeOptions &options) : Node("pcc_bridge", "pcc", options),
                                                                       connected(false),
                                                                       port("", 115200), dataQueue(),
                                                                       ledComponent(), servoComponent() {
        readTimer = this->create_wall_timer(5ms, [this] { readLoop(); });

        resetService = node->create_service<std_srvs::srv::Trigger>(
            "reset",
            [this](std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                resetCallback(request, response);
            }
        );

        openPort();

        setup();
        sendFullUpdate();
    }

    void PCCBridgeNode::setup() {
        ledComponent.setup(reinterpret_cast<rclcpp::Node *>(this), [this](message_t *message) { sendMessage(message); });
        servoComponent.setup(reinterpret_cast<rclcpp::Node *>(this), [this](message_t *message) { sendMessage(message); });
    }

    void PCCBridgeNode::sendFullUpdate() {
        RCLCPP_INFO(get_logger(), "Syncing PCC state");

        ledComponent.sendUpdate();
        servoComponent.sendUpdate();
    }

    void PCCBridgeNode::openPort() {
        connected = false;
        port.close();

        do {
            RCLCPP_INFO_THROTTLE(get_logger(), get_clock(), 100, "Searching for a serial port...");

            for (directory_entry& entry : directory_iterator("/dev")) {
                std::string name = entry.path().filename().string();

                if (name.rfind("ttyACM", 0) == 0) {
                    try {
                        port.setPort(entry.path().c_str());
                        port.open();
                        break;
                    } catch (...) {}
                }
            }
        } while (!port.isOpen());

        connected = true;
        RCLCPP_INFO(get_logger(), "Port Opened");
    }

    void PCCBridgeNode::readLoop() {
        readTimer->cancel();
        try {
            if (port.isOpen()) {
                // Read another byte and cycle the queue
                uint8_t currentByte;
                if (port.available() > 0) {
                    if (dataQueue.size() >= MESSAGE_BUF_LEN) {
                        dataQueue.erase(dataQueue.cbegin());
                    }
                    
                    port.read(&currentByte, 1);
                    dataQueue.push_back(currentByte);
                }

                // Check if the queue size is correct, if so, then validate it's crc to ensure it's a complete message.
                if (dataQueue.size() == MESSAGE_BUF_LEN && dataQueue[0] == 255 && dataQueue[1] == 255) {
                    const auto message = reinterpret_cast<message_t *>(dataQueue.data());

                    if (checkCrc(message)) {
                        // Clear the old message to remove the possibility of misinterpreting messages
                        dataQueue.clear();

                        // Check the topic identifier
                        switch (message->identifier) {
                          case TOPIC_STATUS:
                              switch (message->data[0]) {
                                  case STATUS_READY:
                                      RCLCPP_INFO(get_logger(), "PCC Ready");
                                      sendFullUpdate();
                                      break;
                                  case STATUS_RESET:
                                      RCLCPP_INFO(get_logger(), "PCC Resetting");
                                      break;
                                  case STATUS_HALT:
                                      RCLCPP_ERROR(get_logger(), "PCC Halted");
                              }
                              break;
                          case TOPIC_ERROR:
                              RCLCPP_ERROR(get_logger(), "Got error: %u", message->data[0]);
                              switch (message->data[0]) {
                                  case COMPONENT_THERMAL:
                                      if (message->data[1] == ERROR_TYPE_RESOLVED) {
                                          RCLCPP_INFO(get_logger(), "Thermal camera has connected");
                                      } else {
                                          RCLCPP_ERROR(get_logger(), "Thermal camera has disconnected");
                                      }
                                      break;
                                  case COMPONENT_SERVOS:
                                      if (message->data[1] == ERROR_TYPE_RESOLVED) {
                                          RCLCPP_INFO(get_logger(), "Servo controller has connected");
                                      } else {
                                          RCLCPP_ERROR(get_logger(), "Servo controller has disconnected");
                                      }
                              }
                              break;
                          default:
                              RCLCPP_WARN(get_logger(), "Invalid topic: %ul", message->identifier);
                        }
                    }
                }
            } else {
                // Just to fall over to the outer catch block as well
                throw std::exception("Port not open");
            }
        } catch (...) {
            RCLCPP_WARN(get_logger(), "Port not open. Searching...");
            openPort();

            sendFullUpdate();
        }
        readTimer->reset();
    }

    void PCCBridgeNode::sendMessage(message_t *message) {
        message->sol = SOL_NUM;
        message->identifier = TOPIC_CONVERT(message->identifier);
        append_crc(message);

        if (connected.load()) {
            try {
                port.write(reinterpret_cast<uint8_t *>(message), MESSAGE_BUF_LEN);
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "Failed to send message using serial port");
            }
        }
    }

    void PCCBridgeNode::resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> _,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> _) {
        message_t message;
        message->identifier = TOPIC_RESET;

        sendMessage(&message);
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;
    std::shared_ptr<pcc_bridge::PCCBridgeNode> node = std::make_shared<pcc_bridge::PCCBridgeNode>(options);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
