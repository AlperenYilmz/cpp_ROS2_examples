cmake_minimum_required(VERSION 3.8)
project(cppROSexamples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)

# **** MODIFIED PART ****
add_executable(cppNodeSimple src/firstCppNode.cpp)
ament_target_dependencies(cppNodeSimple rclcpp)

add_executable(cppRoboNode src/cppPublisher.cpp)
ament_target_dependencies(cppRoboNode rclcpp example_interfaces)

add_executable(cppSubsNode src/cppSubsPhone.cpp)
ament_target_dependencies(cppSubsNode rclcpp example_interfaces)

add_executable(int_adder_server_node src/cppAdderServer.cpp)
ament_target_dependencies(int_adder_server_node rclcpp example_interfaces)

add_executable(int_adder_client_noop src/cppAdderClientNOOP.cpp)
ament_target_dependencies(int_adder_client_noop rclcpp example_interfaces)

add_executable(int_adder_client_oop src/cppAdderClientOOP.cpp)
ament_target_dependencies(int_adder_client_oop rclcpp example_interfaces)

add_executable(cppHwStatPubNode src/cppHardwareStPublisher.cpp)
ament_target_dependencies(cppHwStatPubNode rclcpp custom_interfaces)

install(TARGETS
cppNodeSimple
cppRoboNode
cppSubsNode
int_adder_server_node
int_adder_client_noop
int_adder_client_oop
cppHwStatPubNode
DESTINATION lib/${PROJECT_NAME})
# **** MODIFIED PART ****

ament_package()