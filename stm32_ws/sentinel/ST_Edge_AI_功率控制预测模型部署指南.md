# ST Edge AI 部署底盘功率预测模型教学指南

本文面向当前 Hero 底盘功率控制工程，目标是在 STM32H723 上验证三种功率控制实现：纯传统数学模型、AI 直接替代模型、传统数学模型 + AI 残差修正模型，并把三种结果接入同一个 `chassis_power_control()` 链路做实车评估。

## 1. 先理解当前功率控制链路

当前底盘控制链路是：

```text
遥控/行为状态
    -> vx / vy / wz 车体速度指令
    -> chas_inv_cal() 轮速逆解
    -> Model_Based_Control() 生成 4 个 3508 目标电流 model_3508_out[]
    -> chassis_power_control() 生成 give_current[]
    -> CAN_cmd_CHASSIS_ALL() 下发电流
```

当前已有的功率预测函数在 `User/APP_Support/chassis_power_control.c`：

```c
P_origin = a
         + sum_omega_I
         + k_1 * sum_omega2
         + k_2 * sum_I2;
```

其中：

```text
omega        = 3508 电机当前转速
I_cmd        = Model_Based_Control() 输出的目标电流
P_origin     = 预测功率
set_power    = 裁判系统给出的功率上限再乘安全系数
K_Reduction  = 电流缩放系数
```

AI 模型替代点放在这里：

```text
model_3508_out[] + 电机转速 + PM01/裁判数据
    -> AI 模型
    -> P_predict + K_Reduction
    -> 缩放 give_current[]
```

## 2. 模型目标选择

最终上车模型优先输出 `K_Reduction`。

`K_Reduction` 模型的含义是：

```text
输入当前底盘状态、目标电流、功率上限、缓冲能量
    -> 直接输出 0.01 到 1.00 的电流缩放系数
```

运行时控制链路：

```text
Model_Based_Control() 输出 model_3508_out[]
    -> AI 计算 K_Reduction
    -> give_current[i] = model_3508_out[i] * K_Reduction
```

`P_predict` 作为辅助输出同步训练。

`P_predict` 模型的含义是：

```text
输入当前底盘状态和目标电流
    -> 输出下一小段时间内的实际功率
```

`P_predict` 的作用：

```text
训练时提供功率监督
    -> 上板时用于调试曲线和保护判断
```

推荐模型输出：

```text
输出 0：P_predict
输出 1：K_Reduction
```

这个模型需要高质量标签，标签来源可以是 PM01 实测功率、台架标定得到的稳定缩放系数、实车功率超限后的修正系数；离线公式可以生成初始 K 标签，最终 K 标签以实车稳定结果为准。

三种方案定义：

```text
方案 A：纯传统数学模型
输入 omega 和 I_cmd
    -> 公式计算 P_math
    -> 二次方程计算 K_math
    -> 缩放 give_current[]

方案 B：AI 直接替代模型
输入底盘状态、目标电流、功率上限、PM01/裁判数据
    -> AI 输出 P_predict 和 K_ai
    -> 缩放 give_current[]

方案 C：传统数学模型 + AI 残差修正模型
输入底盘状态、目标电流、功率上限、PM01/裁判数据、P_math、K_math
    -> AI 输出 delta_P 和 delta_K
    -> P_final = P_math + delta_P
    -> K_final = K_math + delta_K
    -> 缩放 give_current[]
```

方案 C 的工程含义：

```text
传统公式提供基础功率趋势
    -> AI 学习公式误差、温度影响、电压影响、地面摩擦影响、急加速瞬态误差
    -> 输出修正后的功率和缩放系数
```

在当前代码基础上，方案 C 的数据效率和上板稳定性通常更容易做出来；方案 B 的上限更高，数据覆盖和标签质量要求更高。

后续优化方向：AI 输出四轮缩放系数。

四轮缩放系数模型的含义：

```text
输入底盘目标运动、四轮目标电流、四轮实际轮速、总功率上限、单电机功率
    -> AI 输出 s0、s1、s2、s3
    -> give_current[i] = model_3508_out[i] * s[i]
```

这个模型解决的问题：

```text
总功率被限制
    -> 四个轮子获得不同电流缩放比例
    -> 关键运动方向的轮速跟踪误差下降
    -> 车体预期运动效果保持得更好
```

推荐放在第二阶段实现：

```text
第一阶段：AI 输出总缩放系数 K_Reduction
第二阶段：AI 输出四轮缩放系数 s[4]
```

## 3. 数据采集字段

每个控制周期记录一行 CSV。

推荐采样周期使用当前底盘控制周期，数据行包含：

```csv
t_ms,
vx_set,vy_set,wz_set,
wheel_speed_set_0,wheel_speed_set_1,wheel_speed_set_2,wheel_speed_set_3,
motor_speed_0,motor_speed_1,motor_speed_2,motor_speed_3,
motor_accel_0,motor_accel_1,motor_accel_2,motor_accel_3,
model_current_0,model_current_1,model_current_2,model_current_3,
give_current_0,give_current_1,give_current_2,give_current_3,
motor_power_0,motor_power_1,motor_power_2,motor_power_3,
s_label_0,s_label_1,s_label_2,s_label_3,
p_math,k_math,
set_power,buffer_energy,
pm01_v_in,pm01_i_in,pm01_p_in,
pm01_v_out,pm01_i_out,pm01_p_out,
pm01_temp,
chassis_mode
```

字段含义：

```text
model_current_i = Model_Based_Control() 原始输出
give_current_i  = 实际下发到 3508 的电流
motor_power_i   = 单个 3508 电机实际功率
s_label_i       = 单轮电流缩放系数标签
pm01_p_out      = PM01 输出功率，可作为 P_predict 标签
p_math          = 传统公式预测功率
k_math          = 传统公式求出的电流缩放系数
set_power       = 当前允许底盘功率
buffer_energy   = 裁判系统缓冲能量
```

## 4. 数据集标定

训练 `P_predict` 的标签：

```text
y = pm01_p_out
```

PM01 功率存在通信延迟和测量噪声，标签处理流程：

```text
原始 pm01_p_out
    -> 单位换算
    -> 时间对齐
    -> 低通滤波或滑动平均
    -> 得到 P_label
```

训练 `K_Reduction` 的标签：

```text
y = K_label
```

K 标签生成流程：

```text
P_label 和 set_power 得到功率误差
    -> 台架或实车得到稳定缩放系数
    -> 经过限幅和平滑得到 K_label
```

K 标签范围：

```text
0.01 <= K_label <= 1.00
```

训练方案 C 时增加残差标签：

```text
delta_P_label = P_label - P_math
delta_K_label = K_label - K_math
```

残差标签含义：

```text
传统公式已经解释的部分由 P_math 和 K_math 承担
    -> AI 学习剩余误差
    -> 最终输出 P_final 和 K_final
```

训练四轮缩放系数模型时增加单轮标签：

```text
s_label_i = give_current_i / model_current_i
```

单轮标签处理：

```text
原始 s_label_i
    -> 限幅到 0.01 到 1.00
    -> 平滑相邻周期突变
    -> 结合轮速跟踪误差筛选有效样本
```

四轮缩放系数模型的训练目标：

```text
输出 s0、s1、s2、s3
    -> 让总功率满足 P_limit
    -> 让四轮实际轮速接近四轮目标轮速
    -> 让 give_current[] 相邻周期变化平滑
```

有效数据覆盖以下工况：

```text
低速大电流
高速小电流
急加速
急停
小陀螺
满功率平移
低 buffer
不同电容电压
不同地面摩擦
```

推荐数据量：

```text
P_predict 模型：3 万到 10 万条有效样本可开始训练，20 万到 50 万条更稳
K_Reduction 模型：至少 5 万条接近功率上限或超过功率上限的样本
```

500Hz 记录 10 分钟约等于 30 万条原始样本。

## 5. 训练模型

推荐先用 MLP。

MLP 含义：

```text
MLP = 多层全连接网络
输入一组实时特征
    -> 经过若干层乘加计算
    -> 输出一个连续数值
```

推荐结构：

```text
输入维度：24 到 32
隐藏层 1：32
隐藏层 2：16
输出维度：2
```

用于双输出模型：

```text
输入 x = 当前底盘状态和电流指令
输出 y0 = 预测功率 P_predict
输出 y1 = 电流缩放系数 K_Reduction
损失函数 = P_predict 的 MSE + K_Reduction 的 Huber
```

用于方案 C 的残差模型：

```text
输入 x = 当前底盘状态、电流指令、功率上限、P_math、K_math
输出 y0 = delta_P
输出 y1 = delta_K
最终 P_final = P_math + delta_P
最终 K_final = K_math + delta_K
损失函数 = delta_P 的 MSE + delta_K 的 Huber
```

用于四轮缩放系数模型：

```text
输入 x = 当前底盘状态、四轮目标电流、四轮实际轮速、四轮单电机功率、总功率上限
输出 y0 到 y3 = s0、s1、s2、s3
最终 give_current[i] = model_3508_out[i] * s[i]
损失函数 = s[i] 的 Huber + 轮速跟踪误差惩罚 + 总功率超限惩罚
```

MSE 含义：

```text
MSE = 预测值和真实值差值的平方均值
```

Huber 含义：

```text
Huber = 小误差使用平方惩罚，大误差使用线性惩罚
```

训练集划分按完整场次划分：

```text
训练集：多数组场次
验证集：独立场次
测试集：另一块场地或另一组电池/电容状态
```

评价指标：

```text
P_predict 平均绝对误差 <= 3W 到 5W
功率接近上限区域误差 <= 5W
K_Reduction 平均绝对误差 <= 0.03
模型单次推理时间 <= 控制周期的 10% 到 20%
```

## 6. Python 训练示例

训练前准备：

```powershell
pip install pandas numpy scikit-learn tensorflow
```

示例代码：

```python
import pandas as pd
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

csv_path = "chassis_power_log.csv"
df = pd.read_csv(csv_path)

feature_cols = [
    "vx_set", "vy_set", "wz_set",
    "motor_speed_0", "motor_speed_1", "motor_speed_2", "motor_speed_3",
    "motor_accel_0", "motor_accel_1", "motor_accel_2", "motor_accel_3",
    "model_current_0", "model_current_1", "model_current_2", "model_current_3",
    "give_current_0", "give_current_1", "give_current_2", "give_current_3",
    "motor_power_0", "motor_power_1", "motor_power_2", "motor_power_3",
    "p_math", "k_math",
    "set_power", "buffer_energy",
    "pm01_v_out", "pm01_i_out", "pm01_temp",
]

# 方案 B：AI 直接替代
target_cols = ["pm01_p_out", "k_label"]

# 方案 C：传统数学模型 + AI 残差修正
# target_cols = ["delta_p_label", "delta_k_label"]

# 后续优化：AI 输出四轮缩放系数
# target_cols = ["s_label_0", "s_label_1", "s_label_2", "s_label_3"]

df = df.dropna(subset=feature_cols + target_cols)

X = df[feature_cols].to_numpy(np.float32)
y = df[target_cols].to_numpy(np.float32)

x_scaler = StandardScaler()
y_scaler = StandardScaler()
X = x_scaler.fit_transform(X).astype(np.float32)
y = y_scaler.fit_transform(y).astype(np.float32)

X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)

model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(X.shape[1],)),
    tf.keras.layers.Dense(32, activation="relu"),
    tf.keras.layers.Dense(16, activation="relu"),
    tf.keras.layers.Dense(2),
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3),
    loss=tf.keras.losses.Huber(delta=1.0),
    metrics=["mae"],
)

model.fit(
    X_train, y_train,
    validation_split=0.2,
    epochs=80,
    batch_size=256,
    callbacks=[
        tf.keras.callbacks.EarlyStopping(
            monitor="val_mae",
            patience=8,
            restore_best_weights=True,
        )
    ],
)

loss, mae = model.evaluate(X_test, y_test)
print("test_mae_normalized:", mae)

model.save("chassis_power_predict.keras")
np.save("input_mean.npy", x_scaler.mean_)
np.save("input_scale.npy", x_scaler.scale_)
np.save("target_mean.npy", y_scaler.mean_)
np.save("target_scale.npy", y_scaler.scale_)
```

导出 TFLite：

```python
import tensorflow as tf

model = tf.keras.models.load_model("chassis_power_predict.keras")

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()

with open("chassis_power_predict.tflite", "wb") as f:
    f.write(tflite_model)
```

## 7. 使用 ST Edge AI Developer Cloud

ST 官方流程是：

```text
上传模型
    -> 量化
    -> 优化
    -> Benchmark
    -> 生成 C 代码或 STM32CubeIDE 工程
```

操作流程：

```text
打开 ST Edge AI Developer Cloud
    -> 登录 MyST
    -> 选择 STM32 MCU
    -> 上传 chassis_power_predict.tflite
    -> 选择 Quantize
    -> 选择 Optimize
    -> 选择 Benchmark
    -> 选择接近 STM32H723 的目标板
    -> 查看 Flash、RAM、推理时间
    -> Generate 下载 C Code
```

ST Edge AI Developer Cloud 官方说明支持模型优化、量化、Benchmark、部署到 STM32 目标。

## 8. 使用 STM32Cube.AI / STM32Cube AI Studio

本地部署流程：

```text
安装 STM32CubeMX
    -> 安装 X-CUBE-AI / STM32Cube.AI
    -> 打开当前 .ioc 或新建测试工程
    -> 加入 .tflite 模型
    -> Analyse 查看 RAM/Flash/MACC
    -> Validate 验证桌面端和生成 C 代码输出一致性
    -> Generate Code
```

STM32Cube.AI 支持 `.tflite`、`.h5`、`.onnx`，支持 FLOAT32 和 INT8 权重格式，并生成 STM32 推理 C 代码。

## 9. 接入当前工程

生成代码后通常会得到：

```text
network.c
network.h
network_data.c
network_data.h
app_x-cube-ai.c
app_x-cube-ai.h
```

建议封装一个 AI 功率控制接口：

```c
typedef struct
{
    float p_predict;
    float k_reduction;
} chassis_ai_power_out_t;

void chassis_ai_power_init(void);
chassis_ai_power_out_t chassis_ai_power_predict(const chassis_move_t *chassis_move);
```

接入位置：

```c
void chassis_power_control(chassis_move_t *chassis_move)
{
    pm01_access_poll();

    for (int i = 0; i < CHASSIS_MODULE_NUM; i++)
    {
        PowerLimit.now_motorspeed[i] = chassis_move->chassis_3508[i].chassis_motor_measure->speed_rpm;
        PowerLimit.set_motorcurrent[i] = chassis_move->model_3508_out[i];
    }

    chassis_ai_power_out_t ai = chassis_ai_power_predict(chassis_move);

    PowerLimit.P_origin = ai.p_predict;
    PowerLimit.K_Reduction = LIMIT_MAX_MIN(ai.k_reduction, 1.0f, 0.01f);

    for (int i = 0; i < CHASSIS_MODULE_NUM; i++)
    {
        chassis_move->chassis_3508[i].give_current =
            (int16_t)(chassis_move->model_3508_out[i] * PowerLimit.K_Reduction);
    }
}
```

AI 输出有效性检查：

```c
if (!isfinite(ai.k_reduction) || !isfinite(ai.p_predict))
{
    PowerLimit.K_Reduction = 0.5f;
}

PowerLimit.K_Reduction = LIMIT_MAX_MIN(PowerLimit.K_Reduction, 1.0f, 0.01f);
```

该接法的运行机制：

```text
AI 直接输出 K_Reduction
    -> 限幅和平滑
    -> 缩放 4 个 3508 电流
```

方案 C 接入示例：

```c
typedef struct
{
    float p_math;
    float k_math;
} chassis_math_power_out_t;

typedef struct
{
    float delta_p;
    float delta_k;
} chassis_ai_residual_out_t;

void chassis_power_control(chassis_move_t *chassis_move)
{
    pm01_access_poll();

    chassis_math_power_out_t math = chassis_math_power_calc(chassis_move);
    chassis_ai_residual_out_t ai = chassis_ai_residual_predict(chassis_move,
                                                               math.p_math,
                                                               math.k_math);

    PowerLimit.P_origin = math.p_math + ai.delta_p;
    PowerLimit.K_Reduction = math.k_math + ai.delta_k;

    if (!isfinite(PowerLimit.P_origin) || !isfinite(PowerLimit.K_Reduction))
    {
        PowerLimit.P_origin = math.p_math;
        PowerLimit.K_Reduction = math.k_math;
    }

    PowerLimit.K_Reduction = LIMIT_MAX_MIN(PowerLimit.K_Reduction, 1.0f, 0.01f);

    for (int i = 0; i < CHASSIS_MODULE_NUM; i++)
    {
        chassis_move->chassis_3508[i].give_current =
            (int16_t)(chassis_move->model_3508_out[i] * PowerLimit.K_Reduction);
    }
}
```

三种方案共用同一组日志字段：

```text
mode = math / ai / hybrid
P_final
K_final
pm01_p_out
set_power
buffer_energy
give_current[]
底盘速度响应
```

四轮缩放系数接入示例：

```c
typedef struct
{
    float s[CHASSIS_MODULE_NUM];
} chassis_ai_wheel_scale_out_t;

void chassis_power_control(chassis_move_t *chassis_move)
{
    pm01_access_poll();

    chassis_ai_wheel_scale_out_t ai = chassis_ai_wheel_scale_predict(chassis_move);

    for (int i = 0; i < CHASSIS_MODULE_NUM; i++)
    {
        float s_i = LIMIT_MAX_MIN(ai.s[i], 1.0f, 0.01f);
        chassis_move->chassis_3508[i].give_current =
            (int16_t)(chassis_move->model_3508_out[i] * s_i);
    }
}
```

四轮缩放系数输出后增加总功率约束层：

```text
AI 输出 s[4]
    -> 计算预测总功率 P_after
    -> P_after 超过 P_limit 时整体乘二次缩放系数
    -> 输出最终 give_current[]
```

## 10. 上板验证

验证顺序：

```text
离线验证
    -> Cube.AI Validate
    -> 空载上板
    -> 架空轮测试
    -> 低功率地面测试
    -> 满功率测试
```

每一步记录：

```text
P_predict
pm01_p_out
set_power
K_Reduction
model_3508_out[]
give_current[]
vx_set / vy_set / wz_set
buffer_energy
```

横向评估表：

```text
纯传统数学模型：
    -> 记录 P_math、K_math、pm01_p_out、超功率次数、底盘响应

AI 直接替代模型：
    -> 记录 P_predict、K_ai、pm01_p_out、超功率次数、底盘响应

传统数学模型 + AI 残差修正模型：
    -> 记录 P_final、K_final、delta_P、delta_K、pm01_p_out、超功率次数、底盘响应

AI 四轮缩放系数模型：
    -> 记录 s0、s1、s2、s3、四轮轮速误差、四轮单电机功率、总功率、底盘响应
```

核心指标：

```text
功率误差 = P_final - pm01_p_out
超功率次数 = P_final 或 pm01_p_out 超过 set_power 的次数
功率贴边能力 = pm01_p_out 接近 set_power 的程度
底盘响应 = vx/vy/wz 指令到实际轮速变化的时间
电流平滑度 = K 和 give_current[] 的相邻周期变化量
四轮分配效果 = 四个轮速误差和四个单电机功率分布
```

通过标准：

```text
P_predict 跟随 pm01_p_out
K_Reduction 平滑
give_current 无突变
实际功率贴近 set_power
底盘响应无明显迟滞
控制周期余量充足
```

## 11. 工程参数建议

模型规模：

```text
MLP 24-32-16-2
计算量约 1.4k MAC
权重约 1KB 到 3KB
激活 RAM 小于 1KB
```

四轮缩放系数模型：

```text
MLP 32-48-24-4
计算量约 2.8k MAC
输出 s0、s1、s2、s3
适合 1ms 到 5ms 周期更新
```

更复杂模型：

```text
1D-CNN 输入 8 路信号 x 16 个采样点
计算量约 10k 到 30k MAC
适合预测功率突增趋势
```

推理周期：

```text
P_predict：1ms 到 5ms 推理一次
K_Reduction：1ms 到 5ms 更新一次
最终 K_Reduction 加一阶滤波
```

K 滤波示例：

```c
K_filtered = 0.8f * K_filtered + 0.2f * K_ai;
```

## 12. 官方资料

ST Edge AI Developer Cloud：

```text
https://stm32ai-cs.st.com/assets/docs/STM32CubeAIDeveloperCloud.html
```

STM32Cube.AI：

```text
https://stm32ai.st.com/stm32-cube-ai/
```

ST Edge AI Suite：

```text
https://www.st.com/content/st_com/en/st-edge-ai-suite.html
```

X-CUBE-AI 入门手册：

```text
https://www.st.com/resource/en/user_manual/dm00570145.pdf
```
