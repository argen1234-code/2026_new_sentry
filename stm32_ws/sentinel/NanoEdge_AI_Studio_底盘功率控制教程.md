# NanoEdge AI Studio 底盘功率控制教程

本文面向当前 Hero 工程，目标是用 NanoEdge AI Studio 在 STM32H723 上实现底盘功率预测、功率限幅系数预测，并为后续四轮功率分配 `s[4]` 做准备。

## 1. 工程目标

你的任务属于回归预测。

```text
P_predict      = 连续功率数值
K_Reduction    = 0.01 到 1.00 的连续缩放系数
s0/s1/s2/s3    = 四个连续缩放系数
```

NanoEdge AI Studio 中回归预测对应 `Extrapolation` 项目。

推荐验证顺序：

```text
第一阶段：预测 P_predict
第二阶段：预测 K_Reduction
第三阶段：预测四轮缩放系数 s[4]
```

## 2. 项目配置

Sensor 选择：

```text
Sensor Type：Generic
Data Type：Time Series
```

选择原因：

```text
底盘功率控制数据 = 自定义多变量控制数据
    -> 包含轮速、电流、功率、电压、buffer、模式
    -> Generic 支持自定义输入特征
```

Target 选择：

```text
Target：NUCLEO-H723ZG
```

选择原因：

```text
NUCLEO-H723ZG 和你的 STM32H723VGTx 同属 STM32H723
    -> Cortex-M7
    -> 564KB RAM
    -> 算力和内存评估可参考
```

最终集成时仍使用当前 Keil/CubeMX 工程的 `STM32H723VGTx`。

## 3. 输入数量

第一版建议选择 `24 个输入`。

24 个输入定义：

```text
0   vx_set
1   vy_set
2   wz_set
3   wheel_speed_set_0
4   wheel_speed_set_1
5   wheel_speed_set_2
6   wheel_speed_set_3
7   motor_speed_0
8   motor_speed_1
9   motor_speed_2
10  motor_speed_3
11  model_current_0
12  model_current_1
13  model_current_2
14  model_current_3
15  give_current_0
16  give_current_1
17  give_current_2
18  give_current_3
19  set_power
20  buffer_energy
21  pm01_v_out
22  pm01_i_out
23  pm01_temp
```

后续四轮缩放系数版本建议扩展到 `32 个输入`：

```text
增加 motor_power_0..3
增加 p_math
增加 k_math
增加 chassis_mode
增加 pm01_p_out
```

## 4. 数据采集

每个控制周期记录一行 CSV。

推荐采样频率：

```text
500Hz
```

基础 CSV 字段：

```csv
t_ms,
vx_set,vy_set,wz_set,
wheel_speed_set_0,wheel_speed_set_1,wheel_speed_set_2,wheel_speed_set_3,
motor_speed_0,motor_speed_1,motor_speed_2,motor_speed_3,
model_current_0,model_current_1,model_current_2,model_current_3,
give_current_0,give_current_1,give_current_2,give_current_3,
set_power,buffer_energy,
pm01_v_out,pm01_i_out,pm01_temp,
pm01_p_out,k_label,
s_label_0,s_label_1,s_label_2,s_label_3
```

当前工程已适配 VOFA+ CSV 输出。

启用方式：

```c
#define CHASSIS_AI_LOG_ENABLE      1
#define CHASSIS_AI_LOG_PERIOD_MS   10U
```

宏位置：

```text
User/APP/chassis_task.h
```

串口设置：

```text
USART1
921600 baud
VOFA+ 使用文本/CSV 接收
```

当前工程默认发送 VOFA+ JustFloat 二进制流，VOFA+ 可按通道导出数据，后期按固定通道顺序补 CSV 字段名。

JustFloat 通道顺序：

```text
ch0  t_ms
ch1  vx_set
ch2  vy_set
ch3  wz_set
ch4  wheel_speed_set_0
ch5  wheel_speed_set_1
ch6  wheel_speed_set_2
ch7  wheel_speed_set_3
ch8  motor_speed_0
ch9  motor_speed_1
ch10 motor_speed_2
ch11 motor_speed_3
ch12 model_current_0
ch13 model_current_1
ch14 model_current_2
ch15 model_current_3
ch16 give_current_0
ch17 give_current_1
ch18 give_current_2
ch19 give_current_3
ch20 set_power
ch21 buffer_energy
ch22 pm01_v_out
ch23 pm01_i_out
ch24 pm01_temp
ch25 pm01_p_out
ch26 k_label
ch27 s_label_0
ch28 s_label_1
ch29 s_label_2
ch30 s_label_3
```

CSV 记录频率建议：

```text
100Hz：稳定采集 31 路通道
500Hz：JustFloat 数据量约 64KB/s，921600 串口带宽可覆盖
```

标签定义：

```text
P_predict 标签 = pm01_p_out
K_Reduction 标签 = k_label
s[i] 标签 = s_label_i
```

`k_label` 生成方式：

```text
台架或实车找到稳定缩放系数
    -> 限幅到 0.01 到 1.00
    -> 平滑突变
    -> 得到 k_label
```

`s_label_i` 生成方式：

```text
s_label_i = give_current_i / model_current_i
```

`model_current_i` 接近 0 时跳过该行样本。

## 5. 数据格式

NanoEdge AI Studio 通常需要一个输入 CSV 和一个输出目标列。

训练 `P_predict`：

```text
输入列 = 24 个特征
输出列 = pm01_p_out
```

训练 `K_Reduction`：

```text
输入列 = 24 个特征
输出列 = k_label
```

推荐把 `P_predict` 或 `P_math` 加入 `K_Reduction` 的输入：

```text
原始 24 个特征
    -> 增加 P_predict
    -> 增加 P_math
    -> 训练 K_Reduction
```

运行时链路：

```text
24 个原始输入
    -> P_predict 库输出预测功率
    -> K_Reduction 库输入原始特征 + P_predict + set_power
    -> 输出 K_Reduction
    -> 限幅后下发电流
```

训练四轮缩放系数：

```text
方案 1：建立 4 个 Extrapolation 项目
输出列分别为 s_label_0、s_label_1、s_label_2、s_label_3

方案 2：使用 STM32Cube.AI 自定义多输出模型
输出为 s[4]
```

NanoEdge AI Studio 做单输出回归更直接，四轮缩放系数建议先用 4 个独立回归库验证。

## 6. Studio 操作流程

项目创建：

```text
New Project
    -> Extrapolation
    -> Generic
    -> Time Series
    -> Number of axes / signals = 24
    -> Target = NUCLEO-H723ZG
```

导入数据：

```text
Import Signals
    -> 选择训练 CSV
    -> 映射 24 个输入列
    -> 选择目标输出列
```

模型搜索：

```text
Benchmark / Search
    -> 选择较小 RAM/Flash 限制
    -> 启动自动搜索
    -> 记录误差、RAM、Flash、推理时间
```

生成库：

```text
Generate Library
    -> 选择 STM32 / ARM Cortex-M
    -> 下载 NanoEdge AI 静态库和头文件
```

## 7. 集成到当前工程

把 NanoEdge 生成文件加入工程：

```text
nanoedgeai.h
libneai.a 或对应静态库
knowledge.c / knowledge.h
```

初始化位置：

```c
void chassis_ai_init(void)
{
    enum neai_state error_code;

    error_code = neai_extrapolation_init(knowledge);
    (void)error_code;
}
```

推理接口示例：

```c
#include "NanoEdgeAI.h"

static float neai_input[24];
static float neai_output;

float chassis_ai_predict_power(const chassis_move_t *chassis_move)
{
    fill_neai_input(neai_input, chassis_move);
    neai_extrapolation(neai_input, &neai_output);
    return neai_output;
}
```

`fill_neai_input()` 必须严格按训练时的输入顺序填充。

## 8. 接入功率控制

预测 `P_predict`：

```c
PowerLimit.P_origin = chassis_ai_predict_power(chassis_move);
```

预测 `K_Reduction`：

```c
PowerLimit.K_Reduction = chassis_ai_predict_k(chassis_move);
PowerLimit.K_Reduction = LIMIT_MAX_MIN(PowerLimit.K_Reduction, 1.0f, 0.01f);
```

最终缩放：

```c
for (int i = 0; i < CHASSIS_MODULE_NUM; i++)
{
    chassis_move->chassis_3508[i].give_current =
        (int16_t)(chassis_move->model_3508_out[i] * PowerLimit.K_Reduction);
}
```

四轮缩放系数：

```c
float s[4];

s[0] = chassis_ai_predict_s0(chassis_move);
s[1] = chassis_ai_predict_s1(chassis_move);
s[2] = chassis_ai_predict_s2(chassis_move);
s[3] = chassis_ai_predict_s3(chassis_move);

for (int i = 0; i < CHASSIS_MODULE_NUM; i++)
{
    s[i] = LIMIT_MAX_MIN(s[i], 1.0f, 0.01f);
    chassis_move->chassis_3508[i].give_current =
        (int16_t)(chassis_move->model_3508_out[i] * s[i]);
}
```

四轮缩放后增加总功率约束：

```text
预测 s[4]
    -> 计算缩放后的预测功率
    -> 超过 set_power 时整体再乘 K_Reduction
    -> 输出最终 give_current[]
```

## 9. 验证标准

离线指标：

```text
P_predict MAE <= 3W 到 5W
K_Reduction MAE <= 0.03
s[i] MAE <= 0.05
```

上板指标：

```text
推理时间 <= 200us
RAM/Flash 小于工程余量
实际功率贴近 set_power
功率超限次数下降
give_current[] 无明显突变
vx/vy/wz 运动响应不恶化
```

对比方案：

```text
纯传统数学模型
NanoEdge 预测 K_Reduction
NanoEdge 四轮缩放 s[4]
```

## 10. 注意事项

输入数值尺度要一致。

```text
归一化 = 训练输入和上板输入使用同一套数值尺度
    -> CSV 里每一路固定单位
    -> 上板填 neai_input[] 时保持同样单位
```

第一版可以先不手动归一化：

```text
速度统一 m/s
电流统一 CAN 电流值
功率统一 W
温度统一 摄氏度
buffer_energy 统一裁判系统原始单位
```

如果手动归一化，每一路都要保存缩放参数：

```text
x_norm = (x - mean) / scale
    -> mean 和 scale 来自训练集
    -> 上板也使用同一组 mean 和 scale
```

样本要覆盖接近功率上限的区域。

```text
普通低功率巡航数据
    -> 对 K_Reduction 学习帮助有限
接近 set_power 或超过 set_power 的数据
    -> 决定限幅模型效果
```

功率上限后期可以修改。

```text
set_power 必须作为输入特征参与训练
    -> 训练数据覆盖多个功率上限
    -> 上板后修改 set_power 时模型仍能输出对应 K_Reduction
```

推荐覆盖的功率上限：

```text
40W
60W
80W
100W
比赛实际可能出现的裁判功率上限
```

NanoEdge 适合快速自动生成轻量回归库。

```text
单输出 P_predict / K_Reduction
    -> NanoEdge AI Studio 很适合
多输出 MLP + 1D-CNN
    -> STM32Cube.AI 更适合
```

NanoEdge 单输出训练顺序：

```text
先训练 P_predict
    -> 验证功率预测误差
    -> 生成 P_predict 库

再训练 K_Reduction
    -> 输入中加入 P_predict 或 P_math
    -> 输出 k_label
    -> 生成 K_Reduction 库
```

K_Reduction 运行时使用 P_predict：

```text
采集 24 路输入
    -> P_predict 库推理
    -> 拼接 P_predict 和 set_power
    -> K_Reduction 库推理
    -> K 限幅和平滑
    -> give_current[] 下发
```
