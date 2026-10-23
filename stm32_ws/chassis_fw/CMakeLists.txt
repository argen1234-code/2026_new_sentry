cmake_minimum_required(VERSION 3.22)

# 全向轮底盘固件：复用 sentinel 由 .ioc 生成的外设代码与 HAL，
# 只补充 App/ 下的底盘、USB 协议与任务代码。
project(chassis_fw C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(SENTINEL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel)
set(HAL_DIR      ${SENTINEL_DIR}/Drivers/STM32H7xx_HAL_Driver)
set(CMSIS_DIR    ${SENTINEL_DIR}/Drivers/CMSIS)
set(USB_LIB_DIR  ${SENTINEL_DIR}/Middlewares/ST/STM32_USB_Device_Library)
set(FREERTOS_DIR ${SENTINEL_DIR}/Middlewares/Third_Party/FreeRTOS/Source)

# ---------------------------------------------------------------- 源文件 ----

# sentinel 生成的 Core 代码，去掉 main.c / freertos.c（由本工程 App/ 替代）
file(GLOB SENTINEL_CORE_SRC ${SENTINEL_DIR}/Core/Src/*.c)
list(FILTER SENTINEL_CORE_SRC EXCLUDE REGEX "/(main|freertos)\\.c$")

# HAL 驱动全量编译：stm32h7xx_it.c 引用了所有外设句柄
file(GLOB HAL_SRC ${HAL_DIR}/Src/*.c)

file(GLOB APP_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/App/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/chassis/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/bsp/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/algorithm/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/comm/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/stubs/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/port/*.c
)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32h723xx.s

    ${SENTINEL_CORE_SRC}

    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_cdc_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target/usbd_conf.c

    ${USB_LIB_DIR}/Core/Src/usbd_core.c
    ${USB_LIB_DIR}/Core/Src/usbd_ctlreq.c
    ${USB_LIB_DIR}/Core/Src/usbd_ioreq.c
    ${USB_LIB_DIR}/Class/CDC/Src/usbd_cdc.c

    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/CMSIS_RTOS_V2/cmsis_os2.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c

    ${HAL_SRC}
    ${APP_SRC}
)

# ---------------------------------------------------------------- 头文件 ----

# App/stubs 必须排在 Core/Inc 之前：它提供 gimbal_task.h、hwt_imu.h 等
# 上游头文件的最小替代。
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/App
    ${CMAKE_CURRENT_SOURCE_DIR}/App/common
    ${CMAKE_CURRENT_SOURCE_DIR}/App/chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/App/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/App/algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/App/comm
    ${CMAKE_CURRENT_SOURCE_DIR}/App/port

    ${SENTINEL_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target

    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${CMSIS_DIR}/Device/ST/STM32H7xx/Include
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/RTOS2/Include

    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/CMSIS_RTOS_V2

    ${USB_LIB_DIR}/Core/Inc
    ${USB_LIB_DIR}/Class/CDC/Inc
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32H723xx
    USE_PWR_LDO_SUPPLY
    $<$<CONFIG:Debug>:DEBUG>
)

# ---------------------------------------------------------------- 链接 ----
# 链接参数（-T 链接脚本、nano.specs、-Map、--gc-sections）由
# cmake/gcc-arm-none-eabi.cmake 中的 CMAKE_C_LINK_FLAGS 提供，不要重复添加。
# 数学库必须放在目标文件之后。
target_link_libraries(${PROJECT_NAME} PRIVATE m)

# 生成 hex/bin，便于 OpenOCD 烧录
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating ${PROJECT_NAME}.hex / .bin"
)
