cmake_minimum_required(VERSION 3.5)
project(avr_vmc_pcc_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Boost COMPONENTS filesystem  REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(avr_common_interfaces REQUIRED)
find_package(avr_vmc_pcc_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_executable(pcc_bridge_node src/pcc_bridge.cpp src/crc15can.cpp src/led_component.cpp src/servo_component.cpp src/thermal_component.cpp)
target_link_libraries(pcc_bridge_node ${Boost_FILESYSTEM_LIBRARY})
ament_target_dependencies(pcc_bridge_node rclcpp serial std_msgs std_srvs avr_common_interfaces avr_vmc_pcc_interfaces)
target_include_directories(pcc_bridge_node PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS pcc_bridge_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
