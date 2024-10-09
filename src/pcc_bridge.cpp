#include "pcc_bridge/pcc_bridge.hpp"

#include <string>
#include <thread>
#include <chrono>
#include <boost/filesystem.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "pcc_bridge/topics.h"

using namespace boost::filesystem;
using namespace std::chrono_literals;

std::shared_ptr<pcc_bridge::PCCBridgeNode> node;


namespace pcc_bridge {
    PCCBridgeNode::PCCBridgeNode(const rclcpp::NodeOptions &options) : Node("pcc_bridge", "pcc", options), onStateUpdate(),
                                                                       connected(false), lastState(), thermalCameraConnected(), servoControllerConnected(),
                                                                       isRunningMutex(), serialSendMutex(),
                                                                       port("", 115200), dataQueue(),
                                                                       ledComponent(), servoComponent(), thermalComponent() {
        readTimer = create_wall_timer(1ms, [this] { readLoop(); });
        syncTimer = create_wall_timer(50ms, [this] { sendFullUpdate(); });
        syncTimer->cancel();

        eStopSubscriber = this->create_subscription<std_msgs::msg::Empty>(
            "/estop",
            10,
            [this](const std::shared_ptr<std_msgs::msg::Empty> message) {
            	eStopCallback(message);
            }
        );
        resetService = create_service<std_srvs::srv::Trigger>(
            "reset",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                resetCallback(request, response);
            }
        );

        openPort();

        setup();

        RCLCPP_INFO(get_logger(), "Setup done");
    }

    void PCCBridgeNode::setup() {
        ledComponent.setup(reinterpret_cast<rclcpp::Node *>(this), [this](message_t *message) { sendMessage(message); });
        servoComponent.setup(reinterpret_cast<rclcpp::Node *>(this), [this](message_t *message) { sendMessage(message); });
        thermalComponent.setup(reinterpret_cast<rclcpp::Node *>(this), [this](message_t *message) { sendMessage(message); });
    }

    void PCCBridgeNode::sendFullUpdate() {
        syncTimer->cancel();
        RCLCPP_INFO(get_logger(), "Syncing PCC state");

        ledComponent.sendUpdate();
        std::this_thread::sleep_for(50ms);
        servoComponent.sendUpdate();
    }

    void PCCBridgeNode::openPort() {
        connected = false;
        port.close();
        onStateUpdate();

        auto& clk = *get_clock();

        RCLCPP_INFO(get_logger(), "Searching for a serial port...");

        for (int i = 0; i < 5; i++) {
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

            if (port.isOpen()) {
            	connected = true;
                RCLCPP_INFO(get_logger(), "Port Opened");
                onStateUpdate();
                break;
            } else {
            	RCLCPP_WARN_THROTTLE(get_logger(), clk, 250, "Failed to find port. Retrying...");
            }
        }
    }

    void PCCBridgeNode::readLoop() {
        bool couldLock = isRunningMutex.try_lock();

        if (couldLock) {
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
	                                      syncTimer->reset();
	                                      thermalCameraConnected = true;
	                                      servoControllerConnected = true;
	                                      break;
	                                  case STATUS_RESET:
	                                      RCLCPP_INFO(get_logger(), "PCC Resetting");
	                                      break;
	                                  case STATUS_HALT:
	                                      RCLCPP_ERROR(get_logger(), "PCC Halted");
	                              }
	                              lastState = message->data[0];
	                              onStateUpdate();
	                              break;
	                          case TOPIC_ERROR:
                                  if (message->data[1] != ERROR_TYPE_RESOLVED) {
	                                  RCLCPP_ERROR(get_logger(), "Got error: %u", message->data[0]);
                                  }
	                              switch (message->data[0]) {
	                                  case COMPONENT_THERMAL:
	                                      if (message->data[1] == ERROR_TYPE_RESOLVED) {
	                                          RCLCPP_INFO(get_logger(), "Thermal camera has connected");
	                                          thermalCameraConnected = true;
	                                      } else {
	                                          RCLCPP_ERROR(get_logger(), "Thermal camera has disconnected");
	                                          thermalCameraConnected = false;
	                                      }
	                                      break;
	                                  case COMPONENT_SERVOS:
	                                      if (message->data[1] == ERROR_TYPE_RESOLVED) {
	                                          RCLCPP_INFO(get_logger(), "Servo controller has connected");
	                                          servoControllerConnected = true;
	                                      } else {
	                                          RCLCPP_ERROR(get_logger(), "Servo controller has disconnected");
	                                          servoControllerConnected = false;
	                                      }
	                              }
	                              onStateUpdate();
	                              break;
                              case TOPIC_THERMAL_ROW_0:
                                  thermalComponent.onRowUpdate(0, reinterpret_cast<float *>(message->data));
                                  break;
	                          case TOPIC_THERMAL_ROW_1:
	                           	  thermalComponent.onRowUpdate(1, reinterpret_cast<float *>(message->data));
                                  break;
	                          case TOPIC_THERMAL_ROW_2:
	                              thermalComponent.onRowUpdate(2, reinterpret_cast<float *>(message->data));
                                  break;
	                          case TOPIC_THERMAL_ROW_3:
	                        	  thermalComponent.onRowUpdate(3, reinterpret_cast<float *>(message->data));
                                  break;
	                          case TOPIC_THERMAL_ROW_4:
	                        	  thermalComponent.onRowUpdate(4, reinterpret_cast<float *>(message->data));
                                  break;
	                          case TOPIC_THERMAL_ROW_5:
	                        	  thermalComponent.onRowUpdate(5, reinterpret_cast<float *>(message->data));
                                  break;
	                          case TOPIC_THERMAL_ROW_6:
	                        	  thermalComponent.onRowUpdate(6, reinterpret_cast<float *>(message->data));
                                  break;
	                          case TOPIC_THERMAL_ROW_7:
	                        	  thermalComponent.onRowUpdate(7, reinterpret_cast<float *>(message->data));
                                  break;
	                          default:
	                              RCLCPP_WARN(get_logger(), "Invalid topic: %ul", message->identifier);
	                        }
	                    }
	                }
	            } else {
	                // Just to fall over to the outer catch block as well
	                throw std::exception();
	            }
	        } catch (...) {
	            RCLCPP_WARN(get_logger(), "Port not open. Searching...");
	            openPort();
	        }

	        isRunningMutex.unlock();
	    }
    }

    void PCCBridgeNode::sendMessage(message_t *message) {
        std::lock_guard<std::mutex> guard(serialSendMutex);

        message->sol = SOL_NUM;
        append_crc(message);

        if (connected.load()) {
            try {
                port.write(reinterpret_cast<uint8_t *>(message), MESSAGE_BUF_LEN);
                std::this_thread::sleep_for(50ms);
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "Failed to send message using serial port");
            }
        }
    }

    void PCCBridgeNode::eStopCallback(__attribute__((unused)) const std::shared_ptr<std_msgs::msg::Empty> _msg) {
    	message_t message;
        message.identifier = TOPIC_ESTOP;

        sendMessage(&message);
    }

    void PCCBridgeNode::resetCallback(__attribute__((unused)) const std::shared_ptr<std_srvs::srv::Trigger::Request> _rq,
                                      __attribute__((unused)) std::shared_ptr<std_srvs::srv::Trigger::Response> _rs) {
        message_t message;
        message.identifier = TOPIC_RESET;

        sendMessage(&message);
    }
}

void pcc_diagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat) {
    bool connected = node->connected.load();
    uint8_t lastState = node->lastState.load();
    bool thermalCameraConnected = node->thermalCameraConnected;
    bool servoControllerConnected = node->servoControllerConnected;

	bool error = !(connected && (lastState != STATUS_HALT) && thermalCameraConnected && servoControllerConnected);
	bool warn = node->lastState != STATUS_READY;

	if (error) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Something isn\'t great");
	} else if (warn) {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "The current state has a warning");
	} else {
		stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All is good");
	}

	stat.add("connected", connected);
	stat.add("state", lastState);
	stat.add("thermal_connected", thermalCameraConnected);
	stat.add("servo_connected", servoControllerConnected);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const rclcpp::NodeOptions options;
    node = std::make_shared<pcc_bridge::PCCBridgeNode>(options);

    diagnostic_updater::Updater updater(node);
    updater.setHardwareID("pcc");

    node->onStateUpdate = [&]{
    	updater.force_update();
    };
    updater.add("PCC Status", pcc_diagnostic);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
