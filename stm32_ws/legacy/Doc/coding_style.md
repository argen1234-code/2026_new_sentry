# Printk_Prj 编码规范

## 1. 目标

本文档定义 `Printk_Prj` 单片机工程的代码风格、目录组织、命名规则、注释规范、实现约束和工程维护原则。

本工程以 **C 语言实现为事实来源**，允许使用少量 C++17 头文件进行零运行时成本包装，但不接受复杂运行时 C++ 抽象。

本规范的目标是：

1. 保证代码风格统一。
2. 降低长期维护成本。
3. 避免过度封装、过度兼容和过度防御。
4. 保证单片机实时性、可读性和可审查性。
5. 避免 CubeMX / Keil 工程自动生成内容被破坏。

---

## 2. 基本原则

### 2.1 工程定位

本工程仅服务于当前达妙 DM02 控制板相关开发，不以兼容多种板卡为目标。

实现时可以保持必要的可移植性，但不得为了“未来可能兼容”引入复杂抽象层、兼容层或运行时分支。

当前仓库是母仓库。具体车辆通过 `Robot` target 表达，不在底层模块中写死“某兵种一定如何接线、某设备一定服务某控制语义”。

`old_stm32_code_reference/` 只作为旧工程参考来源，不参与当前工程构建，不允许从新工程源码 include 其中的头文件。

### 2.2 实现原则

1. 优先使用 C 语言。
2. 非必要不使用动态内存。
3. 非必要不引入运行时多态。
4. 非必要不增加全局状态。
5. 非必要不增加多层目录。
6. 不接受冗余回退逻辑。
7. 不接受“为了保险”而堆叠门控、兜底、兼容路径。
8. 只保留必要的安全钳位和参数检查。
9. 发现架构臃肿时，优先重写，而不是继续打补丁。

---

## 3. 命名规范

### 3.1 类型命名

类型、结构体、枚举类型、模块类型使用 **大驼峰命名法**。

示例：

```c
typedef struct
{
    float kp;
    float ki;
    float kd;
} PidParam;

typedef enum
{
    MotorStateStop = 0,
    MotorStateRun,
    MotorStateFault,
} MotorState;
````

### 3.2 函数命名

函数使用 **小写 + 下划线命名法**。

命名应简洁、紧凑、能反映意图。

示例：

````c
pid_init();
pid_calc();
motor_set_current();
imu_update();
````

不推荐：

````c
controller_motor_speed_loop_pid_calculate_output_value();
````

### 3.3 文件命名

`.c`、`.h` 文件使用 **小写 + 下划线命名法**。

示例：

````text
pid.c
pid.h
motor_dm.c
motor_dm.h
imu_bmi088.c
imu_bmi088.h
````

### 3.4 文件夹命名

文件夹使用 **大驼峰命名法**。

示例：

````text
Component/
Controller/
Bsp/
ThirdParty/
````

### 3.5 下划线数量限制

小写下划线命名原则上不应超过三个下划线。

允许：

````c
motor_dm_set_current();
imu_bmi088_update();
ctrl_yaw_calc();
````

不推荐：

````c
robot_gimbal_yaw_motor_current_loop_update();
````

命名过长时，应优先反思模块边界是否过深，而不是继续加前缀。

---

## 4. 文件组织规范

### 4.1 平铺优先

本工程优先采用平铺式组织。

如果某一类子模块文件数量不超过 4 个，原则上不应单独新建子文件夹。

推荐：

````text
Bsp/
  bsp_can.c
  bsp_can.h
  bsp_uart.c
  bsp_uart.h
````

不推荐：

````text
Bsp/
  CAN/
    bsp_can.c
    bsp_can.h
  UART/
    bsp_uart.c
    bsp_uart.h
````

### 4.2 分支实现

如果一个模块存在多个具体实现分支，应采用如下结构：

````text
Component/
  Controller/
    controller.h
    controller.c
    controller.hpp
    Src/
      controller_pid.c
      controller_pid.h
      controller_lqr.c
      controller_lqr.h
````

要求：

1. 对外暴露的统一行为放在 `<module_name>.h / .c`。
2. 具体分支实现放在 `Src/` 中。
3. 分支文件使用 `<module_name>_<branch>.h/c` 命名。
4. 可选提供 `<module_name>.hpp` 作为 C++ 零成本转发层。
5. 不允许为了保留旧接口而引入兼容层。

### 4.3 Apps 平铺规则

`Apps` 目录使用平铺结构。每个 App 使用统一前缀的三件套文件，不再为每个 App 单独建文件夹。

推荐：

````text
Apps/
  app_observe.c
  app_observe.h
  app_observe_config.h
  app_input.c
  app_input.h
  app_input_config.h
````

不推荐：

````text
Apps/Observe/app_observe.c
Apps/Input/app_input.c
````

这样做可以让 App 层保持统一排序，减少 Keil/CMake include path 数量，也符合“小模块不额外嵌套”的规则。

### 4.4 Tasks 入口规则

`Tasks` 目录只保留一层 `Src`。根目录放真正固件入口，`Src` 放 RTOS app 启动编排。

固定入口如下：
```text
Tasks/
  main.c
  main.h
  Src/
    task_main.c
    task_main.h
```

要求：
1. `Tasks/main.c/.h` 持有当前工程自己的 `main()`、系统时钟配置和 target 外设初始化分支。
2. 不使用 CubeMX 生成的 `Boards/Core/Src/main.c` 或 `main.cpp` 作为最终入口。
3. `Tasks/Src/task_main.c/.h` 只负责按 `ROBOT_APP_ENABLE_*` 编排 app 启动顺序。
4. target 外设初始化通过 `TASK_MAIN_TARGET` 在编译期选择，不做运行时 target 切换。
5. 不为旧 `main_task` 名称保留 wrapper 或兼容入口。

### 4.5 Robot Target 组织

具体车辆配置使用“一台车一组完整 target 文件”的方式维护。命名推荐：

````text
Robot/<Role>/<role_chassis_year>.h
Robot/<Role>/<role_chassis_year>_map.h
Robot/<Role>/<role_chassis_year>_param.h
````

要求：

1. `Robot/robot.h` 只负责选择当前 `ROBOT_TARGET`。
2. `<target>.h` 聚合身份、App 启动选择、机构构型和参数文件。
3. `<target>_map.h` 只写硬件接线、外设绑定、电机 bus/id 和 GPIO/TIM/SPI/UART 映射。
4. `<target>_param.h` 只写输入映射、控制参数、观测参数、通信和调试策略。
5. 不做兵种继承，不做模糊 profile 叠加，不把具体车辆参数写进 `Bsp`、`Devices` 或 `Component`。
6. 允许 target 文件中存在重复宏，优先保证“这台车是谁、怎么接、用什么参数”一眼明确。

---

## 5. 头文件规范

### 5.1 头文件职责

头文件只放置对外可见的声明，包括：

1. 类型定义。
2. 枚举定义。
3. 宏定义。
4. 函数声明。
5. 必要的内联函数。
6. 必要的 C++ 包装接口。

头文件中出现的所有内容都必须使用标准 Doxygen 风格中文注释。

### 5.2 头文件模板

````c
/**
 ******************************************************************************
 * @file    xxx.h
 * @author  GPT
 * @version Vx.x.x
 * @date    <Date>
 * @brief   <模块简要说明>
 *
 * @note    <补充说明>
 *
 * @note    <补充说明>
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_<MODULE_NAME>_<SUBMODULE_NAME>_H
#define PRINTK_<MODULE_NAME>_<SUBMODULE_NAME>_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief <函数简要说明>
 *
 * <函数详细说明>
 *
 * @param para1 <参数说明>
 * @param para2 <参数说明>
 * @return <返回值说明>
 */
Type function_name(Type para1, Type para2);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_<MODULE_NAME>_<SUBMODULE_NAME>_H */
````

### 5.3 Include Guard

头文件保护宏格式：

````c
PRINTK_<MODULE_NAME>_<SUBMODULE_NAME>_H
````

示例：

````c
#ifndef PRINTK_CONTROLLER_PID_H
#define PRINTK_CONTROLLER_PID_H
````

### 5.4 Include 写法

源码依赖工程 include path，不允许在源码中写跨模块路径。

推荐：

````c
#include "rc.h"
#include "gimbal.h"
#include "shoot.h"
````

禁止：

````c
#include "../../../Devices/Src/rc.h"
#include "Devices/Src/rc.h"
#include "Component/Controller/Gimbal/gimbal.h"
````

模块内部短路径允许保留，例如模块入口包装包含自己的 `Src` 头文件：

````c
#include "Src/referee.h"
````

如果 Keil 或 CMake 找不到头文件，应修工程 include path，不允许用源码相对路径绕过。

---

## 6. C 文件规范

### 6.1 文件头

`.c` 文件必须和 `.h` 文件一样具有文件说明头。

示例：

````c
/**
 ******************************************************************************
 * @file    pid.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   PID 控制器实现
 *
 * @note    本文件实现位置式 PID 控制器。
 *
 * @note    本实现不进行动态内存分配。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
````

### 6.2 内部宏和内部函数

如果 `.c` 文件中出现未在头文件暴露的宏、静态函数、静态变量，需要使用标准 Doxygen 风格注释。

示例：

````c
/**
 * @brief PID 积分项默认限幅值。
 *
 * 该值用于防止积分项在长时间饱和时继续累积。
 */
#define PID_DEFAULT_I_LIMIT 1000.0f

/**
 * @brief 将输入值限制到指定范围。
 *
 * @param value 输入值。
 * @param min 最小值。
 * @param max 最大值。
 * @return 限幅后的值。
 */
static float clampf(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}
````

### 6.3 函数内部注释

函数内部只对重要逻辑进行注释。

必须注释的场景包括：

1. 数值计算中容易出错的地方。
2. 单位转换。
3. 坐标系转换。
4. 固定点 / 浮点混合计算。
5. 饱和、限幅、归一化。
6. 中断、DMA、缓存一致性相关逻辑。
7. 与硬件时序相关的等待或保护。
8. 非显然的优化。

不需要注释显而易见的语句。

不推荐：

````c
i++; /* i 加一 */
````

推荐：

````c
/* 角度误差归一化到 [-pi, pi]，避免跨零点时误差突变。 */
err = angle_norm_rad(ref - fdb);
````

---

## 7. API 设计规范

### 7.1 API 命名

API 名称应短、清晰、稳定。

推荐：

````c
pid_init();
pid_reset();
pid_calc();
pid_set_limit();
````

不推荐：

````c
pid_controller_object_calculate_output_with_limit();
````

### 7.2 API 参数检查

使用输入参数前必须判断参数有效性。

对于指针参数：

````c
if (pid == NULL)
{
    return;
}
````

对于数值参数，应检查必要的范围。

示例：

````c
if (dt <= 0.0f)
{
    return 0.0f;
}
````

### 7.3 参数检查边界

参数检查只做必要安全保护，不允许堆叠复杂防御逻辑。

允许：

````c
if (pid == NULL)
{
    return 0.0f;
}

if (dt <= 0.0f)
{
    return 0.0f;
}
````

不推荐：

````c
if (pid == NULL)
{
    use_backup_pid();
}

if (dt <= 0.0f)
{
    dt = guess_dt_from_system();
}

if (sensor_fault())
{
    fallback_to_legacy_mode();
}
````

---

## 8. C++ 使用规范

### 8.1 C++ 定位

本工程允许使用 C++17，但 C++ 只用于：

1. `.hpp` 头文件包装。
2. 编译期转发。
3. 类型约束。
4. 简洁访问接口。
5. 零运行时成本胶水层。

C 实现仍然是事实来源。

### 8.2 禁止项

禁止使用：

1. 虚函数。
2. RTTI。
3. 异常。
4. 复杂模板元编程。
5. 运行时多态。
6. 隐式动态内存。
7. 为了“优雅”而隐藏实际控制流。

### 8.3 推荐写法

允许 `.hpp` 提供轻量转发。推荐使用与模块一致的命名空间，不在 `.hpp` 中引入新的运行时控制逻辑：

````cpp
#pragma once

#include "pid.h"

namespace Printk
{
class Pid
{
public:
    explicit Pid(PidHandle *handle) : handle_(handle) {}

    float Calc(float ref, float fdb, float dt)
    {
        return pid_calc(handle_, ref, fdb, dt);
    }

private:
    PidHandle *handle_;
};
}
````

要求：

1. 不引入虚函数。
2. 不持有动态资源。
3. 不改变 C 层事实来源。
4. 不隐藏关键运行时开销。
5. 不在 `.hpp` 中重新实现 `.c` 已有的运行时状态机。
6. 可以提供编译期便捷宏、模板包装或 inline 转发，但不得改变 C API 的语义。

---

## 9. 内存与资源管理

### 9.1 动态内存

原则上不使用动态内存。

如必须使用，应优先考虑：

1. 静态数组。
2. 静态内存池。
3. 固定容量缓冲区。
4. 上位机预处理。

如果静态分配仍无法满足需求，应重新评估该功能是否适合放在单片机中实现。

### 9.2 缓冲区

缓冲区大小必须明确。

不允许隐式增长。

推荐：

````c
#define UART_RX_BUF_SIZE 256U

static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
````

### 9.3 全局变量

全局变量应尽量限制在 `.c` 文件内部。

优先使用：

````c
static MotorState motor_state;
````

避免在头文件中定义全局变量。

---

## 10. CMSIS-DSP 使用规范

CMSIS-DSP 库统一使用：

````text
ThirdParty/CMSIS-DSP
````

不得使用 `Boards` 目录下的 CMSIS-DSP 副本。

封装 DSP 算法时，应保持接口风格与全局一致，不得为了单个算法引入特殊命名风格或特殊目录结构。

---

## 11. CubeMX 与 BSP 约束

### 11.1 CubeMX

原则上不允许直接修改 `.ioc` 文件。

原则上不允许直接修改 CubeMX 会覆盖的代码区域。

如果必须修改 CubeMX 生成文件，应优先使用用户代码区：

````c
/* USER CODE BEGIN xxx */

/* USER CODE END xxx */
````

并在提交中报告此项改动。

### 11.2 BSP

原则上不随意更改 BSP。

确实需要修改时，应保证：

1. 修改目的明确。
2. 不破坏现有硬件初始化流程。
3. 不引入无关兼容逻辑。
4. 不改变 CubeMX 生成代码的可维护性。

### 11.3 Devices 与硬件绑定

`Devices` 只表达“设备是什么、协议怎么解析、如何收发”。具体硬件绑定应来自 `Robot` target 的 map 宏或 App 启动配置。

允许：

````text
Devices/Rc 解析 DBUS/SBUS 原始通道。
Component/Input 将 RC 通道映射为 GimbalCmd / ShootCmd。
Robot target 指定 RC 使用 UART5。
````

不允许：

````text
Devices/Rc 直接决定 yaw、pitch、vx、fire。
Devices/Referee 直接改 Shoot 或 Chassis 输出。
Devices/Motor 内写死某车型的控制策略。
````

---

## 12. 工程构建与调试环境

### 12.1 CMake 与 Keil

日常构建以 CMake + Ninja + arm-none-eabi-gcc 为准：

````bash
cd Boards
cmake --preset Debug
cmake --build --preset Debug
````

Keil 工程仍保留在 `Boards/MDK-ARM/CtrlBoard-H7_IMU.uvprojx`，用于对照和应急，不作为主构建入口。

### 12.2 工程文件

Keil、CMake 和 include path 应与源码结构同步维护。

要求：

1. 新增源码后同步加入 Keil/CMake。
2. 新增公共头目录后同步加入 include path。
3. 不通过跨模块相对 include 弥补工程 include path 缺失。
4. 不修改 `.ioc`，不修改 CubeMX 生成区。

### 12.3 调试和烧录

当前默认调试器为 CMSIS-DAP（DAPLink v2），烧录使用 OpenOCD：

````bash
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "program build/Debug/CtrlBoard-H7_IMU.elf verify reset exit"
````

要求：

1. 调试日志优先走 `Component/Debug` 和 `Component/Communication`。
2. 不新增 App 专用调试三件套。
3. 不把 USB/串口调试通道接入控制安全链路。
4. 烧录器型号、串口号和本机路径属于现场环境事实，不应写进底层通用模块。

### 12.4 Git 使用

仓库使用 git 管理提交历史，但源码事实以工作区文件为准。

要求：

1. 需要比较或审查时，直接读取当前工作区文件，不依赖缓存或旧提交猜测现状。
2. 提交前检查 `git status` 和 `git diff`，确认没有把构建产物或本机配置带入。
3. 不使用 `git reset`、`git checkout` 等命令随意丢弃未提交改动。

## 13. 文件编码

所有源文件、头文件、文档文件统一使用 UTF-8 编码。

进行批量修改时，优先使用 Python 等明确指定 UTF-8 的工具。

不推荐使用可能破坏中文编码的方式直接读写文件。尤其是不要使用未明确指定 UTF-8 的 PowerShell 写文件流程。

---

## 14. 推荐代码风格

### 14.1 缩进与括号

使用 4 空格缩进。

大括号单独成行。

````c
if (value > max)
{
    value = max;
}
````

### 14.2 提前返回

参数非法时可以提前返回，减少嵌套。

推荐：

````c
float pid_calc(PidHandle *pid, float ref, float fdb, float dt)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    ...
}
````

### 14.3 常量后缀

浮点常量使用 `f` 后缀。

````c
float alpha = 0.1f;
````

无符号整数使用 `U` 后缀。

````c
#define CAN_TX_TIMEOUT 10U
````

### 14.4 单位明确

涉及物理量时，变量名或注释应明确单位。

推荐：

````c
float gyro_rad_s;
float angle_rad;
float speed_rpm;
````

不推荐：

````c
float gyro;
float angle;
float speed;
````

---

## 15. 迁移与兼容规则

### 15.1 参考工程使用边界

`old_stm32_code_reference/` 只能作为阅读和迁移参考，不作为新工程依赖。

允许：

1. 阅读参考工程对照业务语义。
2. 将有价值的控制逻辑迁移到新分层中的合适位置。
3. 在注释或文档中说明某参数来源于参考工程。

禁止：

1. 新源码 include 参考工程头文件。
2. 保留旧任务式 API 作为兼容入口。
3. 为了兼容旧全局变量而增加 wrapper。
4. 把旧 weak hook、旧全局 getter、旧任务循环原样搬入新工程。

### 15.2 迁移判断

迁移时只保留有价值的业务语义。

明显错误、重复、备份、临时调试、已经被新架构替代的内容不迁移。意义不明但非常突兀、可能是现场 bug 妥协的行为，应原样记录语义并放到新架构中合适的位置，而不是用旧 API 包起来。

### 15.3 安全钳位

允许必要的安全钳位，例如参数检查、输出限幅、离线清零、温度/热量/功率保护。

不允许把安全钳位扩展成多层回退、兼容路径或隐藏控制策略。

---

## 16. Doxygen 注释示例

### 16.1 类型注释

````c
/**
 * @brief PID 控制器参数。
 */
typedef struct
{
    /**
     * @brief 比例系数。
     */
    float kp;

    /**
     * @brief 积分系数。
     */
    float ki;

    /**
     * @brief 微分系数。
     */
    float kd;

    /**
     * @brief 输出限幅。
     */
    float out_limit;

    /**
     * @brief 积分限幅。
     */
    float i_limit;
} PidParam;
````

### 16.2 函数注释

````c
/**
 * @brief 计算 PID 输出。
 *
 * 根据目标值、反馈值和控制周期计算 PID 输出。
 * 本函数内部会对积分项和最终输出进行限幅。
 *
 * @param pid PID 控制器句柄。
 * @param ref 目标值。
 * @param fdb 反馈值。
 * @param dt 控制周期，单位为秒。
 * @return PID 输出值。
 */
float pid_calc(PidHandle *pid, float ref, float fdb, float dt);
````

---

## 17. 审查标准

提交或合并代码前，应至少检查：

1. 命名是否符合规范。
2. 文件组织是否过度嵌套。
3. 是否引入了不必要兼容层。
4. 是否修改了 CubeMX 自动生成区域。
5. 是否错误使用 CMSIS-DSP 路径。
6. 头文件内容是否均有 Doxygen 中文注释。
7. `.c` 文件内部宏、静态函数、重要逻辑是否有注释。
8. 参数是否在使用前完成必要检查。
9. 是否引入动态内存。
10. 是否引入虚函数、RTTI、异常等运行时 C++ 特性。
11. API 是否过长、过深、过度抽象。
12. 是否可以用更少的代码表达同样逻辑。

---

## 18. 总结

本工程追求的是：

````text
简单、直接、可审查、可维护、实时友好。
````

不追求：

````text
大而全、过度兼容、过度抽象、过度防御、形式主义架构。
````

当代码开始变得臃肿时，应优先删除、重构或重写，而不是继续叠加新的抽象层。
