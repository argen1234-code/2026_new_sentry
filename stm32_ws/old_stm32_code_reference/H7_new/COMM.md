# DM-balanceV1 通讯任务与自瞄链路指南 (H7_new)

涵盖板间通信、自瞄数据通道、与应用任务的对接。

## 主要文件与职责
- 通信栈核心：`User/Communication/core/{comm.c, uproto.c, CRC8_CRC16.c, config.h, platform.h, comm_utils.h}`
  - 协议打包/解析、CRC、平台抽象。
- 通道模板：`User/Communication/channel/{camera, gimbal, time_sync}`
- 示例：`User/Communication/example/{host, device, shared}` 提供上位机/设备演示。
- 应用对接：`User/APP/message_task.[ch]`（构造/发送/接收机器人状态）、`User/APP/auto_aim.[ch]`（解析目标）、`User/APP/communication` 相关引用。
- 底层传输：`User/Bsp/bsp_usart.[ch]`、`bsp_usb.[ch]`、CAN（如需要）。

## 数据流
1) 发送路径
   - 数据来源：`message_task` 收集底盘/云台/射击/功率/自瞄状态。
   - 打包：通过 `comm_utils/uproto` 构造帧，附加 CRC8/CRC16。
   - 物理发送：调用串口/USB/CAN 驱动（通常 USART/USB CDC）。
   - 可复用通道：camera/gimbal/time_sync 作为参考，或自定义 channel。

2) 接收路径
   - 接口：串口/USB 中断或轮询 -> 放入缓存 -> `comm.c` 解析帧。
   - 校验：CRC 校验通过后，按协议类型分发。
   - 落地：更新 `comm_app` 或自定义结构，供 `auto_aim`、`gimbal_task`、`chassis_task` 使用。

3) 自瞄链路
   - 目标来源：上位机/视觉板通过通信栈发送目标 yaw/pitch/距离/置信度。
   - 解析：`auto_aim` 读入解析结果，计算云台目标与开火许可。
   - 反馈：可回传锁定状态/发射结果/延迟，用同一栈打包发送。
   - DM-02 兼容：当前使用 `gimbal` 通道的 `GIMBAL_SID_DELTA.status` 传递业务位，约定 `bit0=target_valid`（上位机当前帧有有效目标/控制指令）。下位机 `auto_aim` 依据 target_valid + 云台误差锁定判定 +  自动瞄准射击电平信号：目标锁定时返回1（持续射击），否则返回0，交给 `shoot` 状态机执行（摩擦轮未预热/超热会自动拒绝）。

## 关键配置
- 协议字段/通道 ID：在 `config.h` 或自定义 channel 中定义。
- 波特率/端口：`bsp_usart` 配置；USB CDC 则参考 `usb_task`。
- 缓冲区大小/超时：`comm.c` 内部常量或宏。

## 调试建议
- 先用 `Communication/example/host` 与 `example/device` 回环验证协议与 CRC。
- 打印/抓包：在 `comm.c` 中暂时开启日志或用 USB CDC 输出解析结果。
- 处理粘包：确认协议有长度字段与 CRC 校验，缓存足够大。
- 自瞄延迟：记录发送/接收时间戳（time_sync 通道可参考），必要时补偿。
