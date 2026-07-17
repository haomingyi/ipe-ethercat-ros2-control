cmake_minimum_required(VERSION 3.16)
project(ipe_three_mode_adapter LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(IPE_ROOT "${PROJECT_ROOT}/ipe")
set(SOEM_ROOT "${IPE_ROOT}/SOEM")

include_directories(
    "${IPE_ROOT}/includes"
    "${SOEM_ROOT}/soem"
    "${SOEM_ROOT}/osal"
    "${SOEM_ROOT}/osal/linux"
    "${SOEM_ROOT}/oshw/linux")

set(SOEM_SRC
    "${SOEM_ROOT}/soem/ethercatbase.c"
    "${SOEM_ROOT}/soem/ethercatmain.c"
    "${SOEM_ROOT}/soem/ethercatcoe.c"
    "${SOEM_ROOT}/soem/ethercatfoe.c"
    "${SOEM_ROOT}/soem/ethercateoe.c"
    "${SOEM_ROOT}/soem/ethercatdc.c"
    "${SOEM_ROOT}/soem/ethercatconfig.c"
    "${SOEM_ROOT}/soem/ethercatsoe.c"
    "${SOEM_ROOT}/soem/ethercatprint.c"
    "${SOEM_ROOT}/osal/linux/osal.c"
    "${SOEM_ROOT}/oshw/linux/nicdrv.c"
    "${SOEM_ROOT}/oshw/linux/oshw.c")

add_executable(ipe_three_mode_lab
    src/ipe_three_mode_lab.cpp
    "${IPE_ROOT}/includes/ecat_motor_master.c"
    "${IPE_ROOT}/includes/ethercat_common.c"
    ${SOEM_SRC})

target_link_libraries(ipe_three_mode_lab PRIVATE pthread)
