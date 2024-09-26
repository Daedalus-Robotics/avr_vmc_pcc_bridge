#include "pcc_bridge/pcc_bridge.hpp"

#include <string>
#include <chrono>
#include <boost/filesystem.hpp>

#include "pcc_bridge/topics.h"

using namespace boost::filesystem;
using namespace std::chrono_literals;


namespace pcc_bridge {
    PCCBridgeNode::PCCBridgeNode(const rclcpp::NodeOptions &options) : Node("pcc_bridge", "pcc", options),
                                                                       port("", 115200), dataQueue(),
                                                                       ledComponent(), servoComponent() {
        openPort();

        readTimer = this->create_wall_timer(5ms, [this] { readLoop(); });
    }

    void PCCBridgeNode::setup() {
        ledComponent.setup(reinterpret_cast<rclcpp::Node *>(this), [this](message_t *message) { sendMessage(message); });
        servoComponent.setup(reinterpret_cast<rclcpp::Node *>(this), [this](message_t *message) { sendMessage(message); });
    }

    void PCCBridgeNode::openPort() {
        port.close();

        for (directory_entry& entry : directory_iterator("/dev")) {
            std::string name = entry.path().filename().string();

            if (name.rfind("ttyACM", 0) == 0) {
                port.setPort(entry.path().c_str());
                port.open();
                break;
            }
        }
    }

    void PCCBridgeNode::readLoop() {
    	if (port.isOpen()) {
    	     uint8_t currentByte;
             if (port.available() > 0) {
                 if (dataQueue.size() >= MESSAGE_BUF_LEN) {
                     dataQueue.erase(dataQueue.cbegin());
                 }
                 port.read(&currentByte, 1);
                 dataQueue.push_back(currentByte);
             }

    	    if (dataQueue.size() == MESSAGE_BUF_LEN && dataQueue[0] == 255 && dataQueue[1] == 255) {
    	        const auto message = reinterpret_cast<message_t *>(dataQueue.data());

    	        if (true) {//(!checkCrc(message)) {
    	            dataQueue.clear();
    	            switch (message->identifier) {
    	              case TOPIC_STATUS:
                          switch (message->data[0]) {
                              case STATUS_READY:
                                  ledComponent.sendUpdate();
                                  servoComponent.sendUpdate();
                                  RCLCPP_INFO(get_logger(), "PCC Ready");
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
                          break;
                      default:
                          RCLCPP_WARN(get_logger(), "Invalid topic: %ul", message->identifier);
                    }
    	        }
    	    }
    	} else {
    		port.close();
    		openPort();
    	}
    }

    void PCCBridgeNode::sendMessage(message_t *message) {
        message->sol = SOL_NUM;
        message->identifier = TOPIC_CONVERT(message->identifier);
        append_crc(message);

        try {
            port.write(reinterpret_cast<uint8_t *>(message), MESSAGE_BUF_LEN);
        } catch(serial::SerialException) {
            RCLCPP_ERROR(get_logger(), "Failed to send message using serial port");
        }
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;
    std::shared_ptr<pcc_bridge::PCCBridgeNode> node = std::make_shared<pcc_bridge::PCCBridgeNode>(options);

    node->setup();

    rclcpp::spin(node);

    printf("Hello");

    rclcpp::shutdown();
    return 0;
}
