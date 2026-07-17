cmake_minimum_required(VERSION 3.8)
project(ipe_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ipe_cst_impedance_controller SHARED
  src/cst_impedance_controller.cpp)
target_compile_features(ipe_cst_impedance_controller PUBLIC cxx_std_17)
target_include_directories(ipe_cst_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ipe_cst_impedance_controller
  control_msgs controller_interface hardware_interface pluginlib rclcpp
  rclcpp_lifecycle realtime_tools sensor_msgs)

pluginlib_export_plugin_description_file(
  controller_interface ipe_controllers_plugin.xml)

install(TARGETS ipe_cst_impedance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(impedance_law_test test/impedance_law_test.cpp)
  target_compile_features(impedance_law_test PUBLIC cxx_std_17)
  target_include_directories(impedance_law_test PRIVATE include)
  add_test(NAME impedance_law_test COMMAND impedance_law_test)
endif()

ament_export_dependencies(
  control_msgs controller_interface hardware_interface pluginlib rclcpp
  rclcpp_lifecycle realtime_tools sensor_msgs)
ament_package()
