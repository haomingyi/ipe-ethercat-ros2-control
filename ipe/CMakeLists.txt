cmake_minimum_required(VERSION 3.10)
project(ethercat_master)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOEM_ROOT ./SOEM)

include_directories(
    ${SOEM_ROOT}/soem
    ${SOEM_ROOT}/osal
    ${SOEM_ROOT}/oshw/linux
    ${SOEM_ROOT}/osal/linux/
    ${CMAKE_SOURCE_DIR}/includes
)

set(SOEM_SRC
    ${SOEM_ROOT}/soem/ethercatbase.c
    ${SOEM_ROOT}/soem/ethercatmain.c
    ${SOEM_ROOT}/soem/ethercatcoe.c
    ${SOEM_ROOT}/soem/ethercatfoe.c
    ${SOEM_ROOT}/soem/ethercateoe.c
    ${SOEM_ROOT}/oshw/linux/nicdrv.c
    ${SOEM_ROOT}/osal/linux/osal.c
    ${SOEM_ROOT}/oshw/linux/oshw.c
    ${SOEM_ROOT}/soem/ethercatdc.c
    ${SOEM_ROOT}/soem/ethercatconfig.c
    ${SOEM_ROOT}/soem/ethercatsoe.c
    ${SOEM_ROOT}/soem/ethercatprint.c
)

set(IPE_ETHERCAT_CORE_SRC
    includes/ethercat_common.c
    includes/ecat_motor_master.c
)

add_executable(single_joint_lab
    src/single_joint_lab.cpp ${IPE_ETHERCAT_CORE_SRC} ${SOEM_SRC})
target_link_libraries(single_joint_lab pthread)

add_executable(ipe_joint_units_test tests/ipe_joint_units_test.cpp)

enable_testing()
add_test(NAME ipe_joint_units_test COMMAND ipe_joint_units_test)

# ROS 2 is optional for the standalone EtherCAT commissioning tool.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    find_package(rclcpp REQUIRED)
    find_package(hardware_interface REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(controller_manager REQUIRED)
    find_package(realtime_tools REQUIRED)

    # A file capability puts glibc's loader in secure-execution mode, where
    # LD_LIBRARY_PATH is ignored. Use legacy DT_RPATH so the ROS library path
    # also applies to transitive dependencies such as
    # liblibstatistics_collector.so.
    get_filename_component(
        IPE_ROS_PREFIX "${rclcpp_DIR}/../../.." ABSOLUTE)
    set(IPE_ROS_LIBRARY_DIR "${IPE_ROS_PREFIX}/lib")
    set(IPE_ROS_SECURE_RPATH
        "${IPE_ROS_LIBRARY_DIR}"
        "${IPE_ROS_LIBRARY_DIR}/x86_64-linux-gnu"
        "${IPE_ROS_PREFIX}/opt/sdformat_vendor/lib"
        "${IPE_ROS_PREFIX}/opt/gz_math_vendor/lib"
        "${IPE_ROS_PREFIX}/opt/gz_utils_vendor/lib")

    add_library(ipe_ros2_control_hardware SHARED
        src/ipe_three_mode_ros2_control_hardware.cpp
        ${IPE_ETHERCAT_CORE_SRC} ${SOEM_SRC})
    add_executable(ipe_ros2_control_node
        src/ipe_ros2_control_node.cpp)
    ament_target_dependencies(ipe_ros2_control_hardware
        hardware_interface pluginlib rclcpp)
    ament_target_dependencies(ipe_ros2_control_node
        controller_manager rclcpp realtime_tools)
    target_link_libraries(ipe_ros2_control_hardware pthread)
    target_link_libraries(ipe_ros2_control_node pthread)
    target_link_options(ipe_ros2_control_node PRIVATE
        "-Wl,--disable-new-dtags")
    set_target_properties(ipe_ros2_control_node PROPERTIES
        BUILD_RPATH "${IPE_ROS_SECURE_RPATH}"
        INSTALL_RPATH "${IPE_ROS_SECURE_RPATH}")
    pluginlib_export_plugin_description_file(
        hardware_interface ipe_ros2_control_plugin.xml)

    install(TARGETS ipe_ros2_control_node
        DESTINATION lib/${PROJECT_NAME})
    install(TARGETS ipe_ros2_control_hardware
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
    ament_package()
endif()
