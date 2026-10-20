cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME CtrlBoard-H7_IMU)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# ---------------------------------------------------------------------------
# 固件选项（编译期 target 选择）
#   cmake --preset Debug -DROBOT_TARGET=sentry_2026
# 可选值：
#   sentry_2026           2026 哨兵，CAN 电机（默认）
#   infantry_steer_2025   2025 舵轮步兵，CAN 电机
#   board_only_dm02       DM02 光板板载资源测试，不启动 CAN/电机
# ---------------------------------------------------------------------------
set(ROBOT_TARGET "sentry_2026" CACHE STRING "Firmware option / robot target")
if(ROBOT_TARGET STREQUAL "sentry_2026")
    set(ROBOT_TARGET_DEF "ROBOT_TARGET_SENTRY_2026")
elseif(ROBOT_TARGET STREQUAL "sentry_2026_bringup")
    set(ROBOT_TARGET_DEF "ROBOT_TARGET_SENTRY_2026_BRINGUP")
elseif(ROBOT_TARGET STREQUAL "sentry_2026_chassis")
    set(ROBOT_TARGET_DEF "ROBOT_TARGET_SENTRY_2026_CHASSIS")
elseif(ROBOT_TARGET STREQUAL "infantry_steer_2025")
    set(ROBOT_TARGET_DEF "ROBOT_TARGET_INFANTRY_STEER_2025")
elseif(ROBOT_TARGET STREQUAL "board_only_dm02")
    set(ROBOT_TARGET_DEF "ROBOT_TARGET_BOARD_ONLY_DM02")
elseif(ROBOT_TARGET STREQUAL "can1_dmh6215_test")
    set(ROBOT_TARGET_DEF "ROBOT_TARGET_CAN1_DMH6215_TEST")
elseif(ROBOT_TARGET STREQUAL "can1_dmh6215_angle_test")
    set(ROBOT_TARGET_DEF "ROBOT_TARGET_CAN1_DMH6215_ANGLE_TEST")
else()
    message(FATAL_ERROR
        "Unknown ROBOT_TARGET '${ROBOT_TARGET}'. "
        "Choose sentry_2026, infantry_steer_2025 or board_only_dm02.")
endif()
message(STATUS "Firmware option: ${ROBOT_TARGET}")

# CAN 测试构建变体：
#   APP_CAN_TEST_STOP_ONLY=1     只发停止/失能命令
#   APP_CAN_TEST_SCAN_ONLY=1     只扫描电机 ID，不驱动
#   APP_CAN_TEST_CONSERVATIVE=1  指定 ID 的低速、限时、低增益 MIT 保守驱动
set(APP_CAN_TEST_STOP_ONLY "0" CACHE STRING "Build CAN test as stop-only firmware")
set(APP_CAN_TEST_SCAN_ONLY "0" CACHE STRING "Build CAN test as scan-only firmware (no drive)")
set(APP_CAN_TEST_TARGET_ID "0" CACHE STRING "Drive only this motor ID (0 = first detected)")
set(APP_CAN_TEST_VEL_RAD_S "3.0F" CACHE STRING "CAN test speed target in rad/s")
set(APP_CAN_TEST_RUN_MS "0" CACHE STRING "CAN test drive duration in ms (0 = run forever)")
set(APP_CAN_TEST_CONSERVATIVE "0" CACHE STRING "Build CAN test as conservative low-speed timed drive")
set(APP_CAN_TEST_SAFE_NATIVE_SPEED "0" CACHE STRING "Conservative test uses native speed mode instead of MIT Kd")
set(APP_CAN_TEST_SAFE_VEL_RAD_S "0.5F" CACHE STRING "Conservative test speed target in rad/s")
set(APP_CAN_TEST_SAFE_KD "0.5F" CACHE STRING "Conservative MIT velocity damping")
set(APP_CAN_TEST_SAFE_SETTLE_MS "300" CACHE STRING "Conservative zero-torque settle time in ms")
set(APP_CAN_TEST_SAFE_RAMP_MS "500" CACHE STRING "Conservative speed ramp time in ms")
set(APP_CAN_TEST_SAFE_HOLD_MS "1000" CACHE STRING "Conservative hold time in ms")
set(APP_CAN_TEST_SAFE_TIMEOUT_MS "3000" CACHE STRING "Conservative hard timeout in ms")
set(APP_CAN_TEST_SAFE_START_FB_TIMEOUT_MS "2000" CACHE STRING "Wait-for-first-feedback timeout in ms")
set(APP_CAN_TEST_SAFE_FB_TIMEOUT_MS "200" CACHE STRING "Feedback watchdog timeout in ms")
set(APP_CAN_TEST_SAFE_POS_LIMIT_RAD "1.0F" CACHE STRING "Conservative position change limit in rad")
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ROBOT_TARGET=${ROBOT_TARGET_DEF}
    APP_CAN_TEST_STOP_ONLY=${APP_CAN_TEST_STOP_ONLY}
    APP_CAN_TEST_SCAN_ONLY=${APP_CAN_TEST_SCAN_ONLY}
    APP_CAN_TEST_TARGET_ID=${APP_CAN_TEST_TARGET_ID}
    APP_CAN_TEST_VEL_RAD_S=${APP_CAN_TEST_VEL_RAD_S}
    APP_CAN_TEST_RUN_MS=${APP_CAN_TEST_RUN_MS}
    APP_CAN_TEST_CONSERVATIVE=${APP_CAN_TEST_CONSERVATIVE}
    APP_CAN_TEST_SAFE_NATIVE_SPEED=${APP_CAN_TEST_SAFE_NATIVE_SPEED}
    APP_CAN_TEST_SAFE_VEL_RAD_S=${APP_CAN_TEST_SAFE_VEL_RAD_S}
    APP_CAN_TEST_SAFE_KD=${APP_CAN_TEST_SAFE_KD}
    APP_CAN_TEST_SAFE_SETTLE_MS=${APP_CAN_TEST_SAFE_SETTLE_MS}
    APP_CAN_TEST_SAFE_RAMP_MS=${APP_CAN_TEST_SAFE_RAMP_MS}
    APP_CAN_TEST_SAFE_HOLD_MS=${APP_CAN_TEST_SAFE_HOLD_MS}
    APP_CAN_TEST_SAFE_TIMEOUT_MS=${APP_CAN_TEST_SAFE_TIMEOUT_MS}
    APP_CAN_TEST_SAFE_START_FB_TIMEOUT_MS=${APP_CAN_TEST_SAFE_START_FB_TIMEOUT_MS}
    APP_CAN_TEST_SAFE_FB_TIMEOUT_MS=${APP_CAN_TEST_SAFE_FB_TIMEOUT_MS}
    APP_CAN_TEST_SAFE_POS_LIMIT_RAD=${APP_CAN_TEST_SAFE_POS_LIMIT_RAD}
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ../Apps/app_board_test.c
    ../Apps/app_communication.c
    ../Apps/app_can_test.c
    ../Apps/app_angle_test.c
    ../Apps/app_board_link.c
    ../Apps/app_calibration.c
    ../Apps/app_observe.c
    ../Apps/app_input.c
    ../Apps/app_magazine.c
    ../Apps/app_gimbal.c
    ../Apps/app_chassis.c
    ../Apps/app_shoot.c
    ../Apps/app_referee.c
    ../Apps/app_daemon.c
    ../Apps/app_alarm.c
    ../Tasks/main.c
    ../Tasks/Src/task_main.c
    ../Algorithm/Src/common.c
    ../Algorithm/Src/pid_core.c
    ../Algorithm/Src/pid_linear.c
    ../Algorithm/Src/pid_angle.c
    ../Algorithm/Src/angle_pid.c
    ../Algorithm/Src/pid.c
    ../Algorithm/Src/kalman.c
    ../Algorithm/Src/extended_kalman.c
    ../Algorithm/Src/mahony.c
    ../Algorithm/Src/ballistic.c
    ../ThirdParty/CMSIS-DSP/Source/CommonTables/arm_common_tables.c
    ../ThirdParty/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c
    ../ThirdParty/CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.c
    ../ThirdParty/CMSIS-DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    ../ThirdParty/CMSIS-DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    ../ThirdParty/CMSIS-DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    ../ThirdParty/CMSIS-DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
    ../Component/Observer/observer.c
    ../Component/Observer/Src/observer_common.c
    ../Component/Observer/Src/observer_ins.c
    ../Component/Observer/Src/observer_chassis.c
    ../Component/Observer/Src/observer_balance.c
    ../Component/Aim/aim.c
    ../Component/Controller/Power/power_ctrl.c
    ../Component/Controller/Thermal/thermal.c
    ../Component/Controller/Chassis/chassis.c
    ../Component/Controller/Chassis/Src/chassis_mecanum.c
    ../Component/Controller/Chassis/Src/chassis_omni.c
    ../Component/Controller/Chassis/Src/chassis_steering.c
    ../Component/Controller/Chassis/Src/chassis_balance.c
    ../Component/Controller/Gimbal/gimbal.c
    ../Component/Controller/Gimbal/Src/gimbal_direct_yaw_pitch.c
    ../Component/Controller/Gimbal/Src/gimbal_double_yaw_pitch.c
    ../Component/Controller/Gimbal/Src/gimbal_linkage_yaw_pitch.c
    ../Component/Controller/Shoot/shoot.c
    ../Component/Controller/Shoot/Src/shoot_none.c
    ../Component/Controller/Shoot/Src/shoot_mono.c
    ../Component/Controller/Shoot/Src/shoot_double.c
    ../Component/Input/input.c
    ../Component/Input/Src/input_common.c
    ../Component/Input/Src/input_gimbal_rc.c
    ../Component/Input/Src/input_gimbal_vision.c
    ../Component/Input/Src/input_gimbal_planner.c
    ../Component/Input/Src/input_chassis_rc.c
    ../Component/Input/Src/input_chassis_slam.c
    ../Component/Input/Src/input_chassis_planner.c
    ../Component/Input/Src/input_cali_rc.c
    ../Component/Input/Src/input_shoot_rc.c
    ../Component/Input/Src/input_shoot_planner.c
    ../Component/Debug/debug.c
    ../Component/Debug/Src/debug_log.c
    ../Component/Debug/Src/debug_platform_stm32.c
    ../Component/Debug/Src/debug_probe.c
    ../Component/Debug/Src/debug_profiler.c
    ../Component/Communication/communication.c
    ../Component/Communication/Src/communication_board.c
    ../Component/Communication/Src/communication_usb.c
    ../Component/Communication/Src/communication_debug.c
    ../Component/Communication/Src/communication_host.c
    ../Component/Communication/Src/communication_time.c
    ../Component/Communication/Src/communication_vofa.c
    ../Component/Communication/Src/Generate/proto2_rt.c
    ../Component/Ui/ui.c
    ../Component/Ui/Src/ui_common.c
    ../Component/Ui/Src/ui_referee.c
    ../Component/Ui/Src/ui_hud_g.c
    ../Bsp/bsp.c
    ../Bsp/Src/bsp_cdc.c
    ../Bsp/Src/bsp_adc.c
    ../Bsp/Src/bsp_crc.c
    ../Bsp/Src/bsp_dts.c
    ../Bsp/Src/bsp_dwt.c
    ../Bsp/Src/bsp_fdcan.c
    ../Bsp/Src/bsp_gpio.c
    ../Bsp/Src/bsp_ospi.c
    ../Bsp/Src/bsp_rng.c
    ../Bsp/Src/bsp_spi.c
    ../Bsp/Src/bsp_tim.c
    ../Bsp/Src/bsp_timebase.c
    ../Bsp/Src/bsp_uart.c
    ../Bsp/Src/bsp_usb.c
    ../Devices/Src/buzzer.c
    ../Devices/Src/magazine.c
    ../Devices/Src/motor.c
    ../Devices/Src/flash.c
    ../Devices/Src/flash_w25q64.c
    ../Devices/Src/power.c
    ../Devices/Src/ws2812.c
    ../Devices/Src/imu.c
    ../Devices/Src/imu_heat.c
    ../Devices/Src/imu_bmi088.c
    ../Devices/Src/rc.c
    ../Devices/Src/rc_dbus.c
    ../Devices/Src/rc_sbus.c
    ../Devices/Src/referee_protocol.c
    ../Devices/Src/referee.c
    ../Devices/Src/tfmini.c
    ../Devices/Src/motor_dji.c
    ../Devices/Src/motor_dm.c
    ../Devices/Src/motor_mit.c
    ../Devices/Src/motor_rs.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
    ../Apps
    ../Tasks
    ../Tasks/Src
    ../Robot
    ../Robot/Infantry
    ../Robot/Sentry
    ../Robot/Test
    ../Algorithm
    ../Algorithm/Src
    ../ThirdParty/CMSIS-DSP/Include
    ../ThirdParty/CMSIS-DSP/PrivateInclude
    ../Component/Observer
    ../Component/Observer/Src
    ../Component/Aim
    ../Component/Controller/Power
    ../Component/Controller/Thermal
    ../Component/Debug
    ../Component/Debug/Src
    ../Component/Communication
    ../Component/Communication/Src
    ../Component/Communication/Src/Generate
    ../Component/Controller/Chassis
    ../Component/Controller/Chassis/Src
    ../Component/Controller/Gimbal
    ../Component/Controller/Gimbal/Src
    ../Component/Controller/Shoot
    ../Component/Controller/Shoot/Src
    ../Component/Input
    ../Component/Input/Src
    ../Component/Ui
    ../Component/Ui/Src
    ..
    ../Bsp
    ../Bsp/Src
    ../Devices
    ../Devices/Src
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
