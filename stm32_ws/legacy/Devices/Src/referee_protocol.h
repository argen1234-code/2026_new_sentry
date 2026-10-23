/**
 ******************************************************************************
 * @file    referee_protocol.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RoboMaster 裁判系统协议定义和基础打包解析接口。
 *
 * @note    本文件只描述裁判系统串口协议、CRC 和帧状态机，不绑定具体 UART。
 * @note    UI 图形交互属于 0x0301 机器人交互帧的一种载荷，本层只负责字节级打包。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_REFEREE_PROTOCOL_H
#define PRINTK_DEVICE_REFEREE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */
/** @brief 裁判系统协议帧头起始字节。 */
#define REFEREE_SOF ((uint8_t)0xA5U)

/** @brief 裁判系统协议帧头长度，单位 byte。 */
#define REFEREE_HEADER_SIZE ((uint16_t)5U)

/** @brief 裁判系统协议命令 ID 长度，单位 byte。 */
#define REFEREE_CMD_SIZE ((uint16_t)2U)

/** @brief 裁判系统协议 CRC16 长度，单位 byte。 */
#define REFEREE_CRC16_SIZE ((uint16_t)2U)

/** @brief 裁判系统协议最大帧长，单位 byte。 */
#define REFEREE_FRAME_MAX_SIZE ((uint16_t)192U)

/** @brief 裁判系统协议最大数据段长度，单位 byte。 */
#define REFEREE_DATA_MAX_SIZE \
    ((uint16_t)(REFEREE_FRAME_MAX_SIZE - REFEREE_HEADER_SIZE - REFEREE_CMD_SIZE - REFEREE_CRC16_SIZE))

/** @brief 裁判系统协议帧头、命令 ID 和 CRC16 总长度，单位 byte。 */
#define REFEREE_FIXED_FRAME_SIZE \
    ((uint16_t)(REFEREE_HEADER_SIZE + REFEREE_CMD_SIZE + REFEREE_CRC16_SIZE))

/** @brief 机器人交互数据段中自定义载荷最大长度，单位 byte。 */
#define REFEREE_INTERACT_DATA_MAX_SIZE ((uint16_t)112U)

/** @brief 机器人交互数据段固定头长度，单位 byte。 */
#define REFEREE_INTERACT_HEAD_SIZE ((uint16_t)6U)

/** @brief CRC8 初始值。 */
#define REFEREE_CRC8_INIT ((uint8_t)0xFFU)

/** @brief CRC16 初始值。 */
#define REFEREE_CRC16_INIT ((uint16_t)0xFFFFU)

/** @brief 裁判系统执行状态。 */
typedef enum
{
    RefereeOk = 0,       /**< 执行成功。 */
    RefereeNull = 1,     /**< 输入指针为空。 */
    RefereeArg = 2,      /**< 输入参数非法。 */
    RefereeLen = 3,      /**< 输入长度非法。 */
    RefereeCrc = 4,      /**< CRC 校验失败。 */
    RefereeBadState = 5, /**< 当前状态不允许执行。 */
    RefereeIo = 6,       /**< 底层 IO 失败。 */
    RefereeOverflow = 7, /**< 输出缓冲区不足。 */
    RefereeNoFrame = 8   /**< 本次输入尚未产生完整帧。 */
} RefereeStatus;

/** @brief 裁判系统命令 ID。 */
typedef enum
{
    RefereeCmdGameStatus = 0x0001U,          /**< 比赛状态。 */
    RefereeCmdGameResult = 0x0002U,          /**< 比赛结果。 */
    RefereeCmdGameRobotHp = 0x0003U,         /**< 机器人血量。 */
    RefereeCmdEventData = 0x0101U,           /**< 场地事件。 */
    RefereeCmdWarning = 0x0104U,             /**< 裁判警告。 */
    RefereeCmdDartInfo = 0x0105U,            /**< 飞镖信息。 */
    RefereeCmdRobotStatus = 0x0201U,         /**< 本机器人状态。 */
    RefereeCmdPowerHeat = 0x0202U,           /**< 功率热量。 */
    RefereeCmdRobotPos = 0x0203U,            /**< 机器人位置。 */
    RefereeCmdBuff = 0x0204U,                /**< 增益状态。 */
    RefereeCmdRobotHurt = 0x0206U,           /**< 受击数据。 */
    RefereeCmdShootData = 0x0207U,           /**< 发射数据。 */
    RefereeCmdProjectile = 0x0208U,          /**< 弹丸允许量。 */
    RefereeCmdRfid = 0x0209U,                /**< RFID 状态。 */
    RefereeCmdDartClient = 0x020AU,          /**< 飞镖客户端命令。 */
    RefereeCmdGroundPos = 0x020BU,           /**< 地面机器人位置。 */
    RefereeCmdRadarMark = 0x020CU,           /**< 雷达标记进度。 */
    RefereeCmdSentryInfo = 0x020DU,          /**< 哨兵信息。 */
    RefereeCmdRadarInfo = 0x020EU,           /**< 雷达信息。 */
    RefereeCmdRobotInteract = 0x0301U,       /**< 机器人交互数据。 */
    RefereeCmdCustomRobot = 0x0302U,         /**< 自定义机器人数据。 */
    RefereeCmdMapCommand = 0x0303U,          /**< 小地图命令。 */
    RefereeCmdRcData = 0x0304U,              /**< 自定义控制器数据。 */
    RefereeCmdMapRobot = 0x0305U,            /**< 小地图机器人数据。 */
    RefereeCmdCustomClient = 0x0306U,        /**< 自定义客户端数据。 */
    RefereeCmdMapData = 0x0307U,             /**< 小地图路径数据。 */
    RefereeCmdCustomInfo = 0x0308U,          /**< 自定义字符信息。 */
    RefereeCmdRobotCustom = 0x0309U          /**< 机器人自定义数据。 */
} RefereeCmdId;

/** @brief 裁判系统机器人 ID。 */
typedef enum
{
    RefereeRedHero = 1,       /**< 红方英雄。 */
    RefereeRedEngineer = 2,   /**< 红方工程。 */
    RefereeRedStandard1 = 3,  /**< 红方步兵 1。 */
    RefereeRedStandard2 = 4,  /**< 红方步兵 2。 */
    RefereeRedStandard3 = 5,  /**< 红方步兵 3。 */
    RefereeRedAerial = 6,     /**< 红方空中。 */
    RefereeRedSentry = 7,     /**< 红方哨兵。 */
    RefereeBlueHero = 101,    /**< 蓝方英雄。 */
    RefereeBlueEngineer = 102,/**< 蓝方工程。 */
    RefereeBlueStandard1 = 103,/**< 蓝方步兵 1。 */
    RefereeBlueStandard2 = 104,/**< 蓝方步兵 2。 */
    RefereeBlueStandard3 = 105,/**< 蓝方步兵 3。 */
    RefereeBlueAerial = 106,  /**< 蓝方空中。 */
    RefereeBlueSentry = 107   /**< 蓝方哨兵。 */
} RefereeRobotId;

/** @brief 裁判系统解包状态。 */
typedef enum
{
    RefereeStepSof = 0,     /**< 等待帧头 SOF。 */
    RefereeStepLenLow = 1,  /**< 等待 data_len 低字节。 */
    RefereeStepLenHigh = 2, /**< 等待 data_len 高字节。 */
    RefereeStepSeq = 3,     /**< 等待序号。 */
    RefereeStepCrc8 = 4,    /**< 等待并校验帧头 CRC8。 */
    RefereeStepData = 5     /**< 接收数据段、cmd_id 和 CRC16。 */
} RefereeUnpackStep;

#if defined(__GNUC__) || defined(__CC_ARM)
#define REFEREE_PACKED __attribute__((packed))
#else
#error "REFEREE_PACKED not defined for this compiler"
#endif

/** @brief 裁判系统帧头。 */
typedef struct REFEREE_PACKED
{
    uint8_t sof;       /**< 固定帧头起始字节 0xA5。 */
    uint16_t data_len; /**< 数据段长度，单位 byte。 */
    uint8_t seq;       /**< 帧序号。 */
    uint8_t crc8;      /**< 帧头 CRC8。 */
} RefereeHeader;

/** @brief 比赛状态数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t game_type : 4;      /**< 比赛类型。 */
    uint8_t game_progress : 4;  /**< 当前比赛阶段。 */
    uint16_t stage_remain_time; /**< 当前阶段剩余时间，单位 s。 */
    uint64_t sync_time_stamp;   /**< UNIX 时间戳。 */
} RefereeGameStatus;

/** @brief 比赛结果数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t winner; /**< 胜利方，0 平局，1 红方，2 蓝方。 */
} RefereeGameResult;

/** @brief 机器人血量数据。 */
typedef struct REFEREE_PACKED
{
    uint16_t red_1_robot_hp;   /**< 红 1 英雄机器人血量。 */
    uint16_t red_2_robot_hp;   /**< 红 2 工程机器人血量。 */
    uint16_t red_3_robot_hp;   /**< 红 3 步兵机器人血量。 */
    uint16_t red_4_robot_hp;   /**< 红 4 步兵机器人血量。 */
    uint16_t reserved_1;       /**< 保留字段。 */
    uint16_t red_7_robot_hp;   /**< 红 7 哨兵机器人血量。 */
    uint16_t red_outpost_hp;   /**< 红方前哨站血量。 */
    uint16_t red_base_hp;      /**< 红方基地血量。 */
    uint16_t blue_1_robot_hp;  /**< 蓝 1 英雄机器人血量。 */
    uint16_t blue_2_robot_hp;  /**< 蓝 2 工程机器人血量。 */
    uint16_t blue_3_robot_hp;  /**< 蓝 3 步兵机器人血量。 */
    uint16_t blue_4_robot_hp;  /**< 蓝 4 步兵机器人血量。 */
    uint16_t reserved_2;       /**< 保留字段。 */
    uint16_t blue_7_robot_hp;  /**< 蓝 7 哨兵机器人血量。 */
    uint16_t blue_outpost_hp;  /**< 蓝方前哨站血量。 */
    uint16_t blue_base_hp;     /**< 蓝方基地血量。 */
} RefereeGameRobotHp;

/** @brief 场地事件数据。 */
typedef struct REFEREE_PACKED
{
    uint32_t event_data; /**< 比赛事件位图。 */
} RefereeEventData;

/** @brief 裁判警告数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t level;              /**< 警告等级。 */
    uint8_t offending_robot_id; /**< 违规机器人 ID。 */
    uint8_t count;              /**< 违规次数。 */
} RefereeWarning;

/** @brief 飞镖信息数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t dart_remaining_time; /**< 飞镖剩余时间。 */
    uint16_t dart_info;          /**< 飞镖信息位域。 */
} RefereeDartInfo;

/** @brief 本机器人状态数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t robot_id;                         /**< 本机器人 ID。 */
    uint8_t robot_level;                      /**< 机器人等级。 */
    uint16_t current_hp;                      /**< 当前血量。 */
    uint16_t maximum_hp;                      /**< 血量上限。 */
    uint16_t shooter_barrel_cooling_value;    /**< 枪口热量每秒冷却值。 */
    uint16_t shooter_barrel_heat_limit;       /**< 枪口热量上限。 */
    uint16_t chassis_power_limit;             /**< 底盘功率上限。 */
    uint8_t power_gimbal_output : 1;          /**< gimbal 口 24V 输出状态。 */
    uint8_t power_chassis_output : 1;         /**< chassis 口 24V 输出状态。 */
    uint8_t power_shooter_output : 1;         /**< shooter 口 24V 输出状态。 */
} RefereeRobotStatus;

/** @brief 功率热量数据。 */
typedef struct REFEREE_PACKED
{
    uint16_t reserved_1;                 /**< 保留字段。 */
    uint16_t reserved_2;                 /**< 保留字段。 */
    float reserved_3;                    /**< 保留字段。 */
    uint16_t buffer_energy;              /**< 缓冲能量，单位 J。 */
    uint16_t shooter_17mm_1_barrel_heat; /**< 第 1 个 17mm 发射机构热量。 */
    uint16_t shooter_17mm_2_barrel_heat; /**< 第 2 个 17mm 发射机构热量。 */
    uint16_t shooter_42mm_barrel_heat;   /**< 42mm 发射机构热量。 */
} RefereePowerHeat;

/** @brief 机器人位置数据。 */
typedef struct REFEREE_PACKED
{
    float x;     /**< 机器人 x 坐标，单位 m。 */
    float y;     /**< 机器人 y 坐标，单位 m。 */
    float angle; /**< 机器人朝向，单位 deg。 */
} RefereeRobotPos;

/** @brief 增益状态数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t recovery_buff;      /**< 回血增益。 */
    uint8_t cooling_buff;       /**< 射击热量冷却倍率。 */
    uint8_t defence_buff;       /**< 防御增益。 */
    uint8_t vulnerability_buff; /**< 负防御增益。 */
    uint16_t attack_buff;       /**< 攻击增益。 */
    uint8_t remaining_energy;   /**< 剩余能量。 */
} RefereeBuff;

/** @brief 受击数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t armor_id : 4;     /**< 装甲模块或测速模块 ID。 */
    uint8_t hurt_reason : 4;  /**< 血量变化类型。 */
} RefereeHurt;

/** @brief 发射数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t bullet_type;         /**< 弹丸类型。 */
    uint8_t shooter_number;      /**< 发射机构 ID。 */
    uint8_t launching_frequency; /**< 弹丸射频，单位 Hz。 */
    float initial_speed;         /**< 弹丸初速度，单位 m/s。 */
} RefereeShootData;

/** @brief 弹丸允许量数据。 */
typedef struct REFEREE_PACKED
{
    uint16_t projectile_allowance_17mm;      /**< 17mm 弹丸允许发弹量。 */
    uint16_t projectile_allowance_42mm;      /**< 42mm 弹丸允许发弹量。 */
    uint16_t remaining_gold_coin;            /**< 剩余金币数量。 */
    uint16_t projectile_allowance_fortress;  /**< 堡垒储备 17mm 弹丸允许量。 */
} RefereeProjectile;

/** @brief RFID 状态数据。 */
typedef struct REFEREE_PACKED
{
    uint32_t rfid_status; /**< RFID 状态位图。 */
} RefereeRfid;

/** @brief 飞镖客户端命令数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t dart_launch_opening_status; /**< 飞镖发射站状态。 */
    uint8_t reserved;                   /**< 保留字段。 */
    uint16_t target_change_time;        /**< 切换目标时的比赛剩余时间。 */
    uint16_t latest_launch_cmd_time;    /**< 最近发射指令确认时间。 */
} RefereeDartClient;

/** @brief 地面机器人位置数据。 */
typedef struct REFEREE_PACKED
{
    float hero_x;       /**< 英雄机器人 x 坐标，单位 m。 */
    float hero_y;       /**< 英雄机器人 y 坐标，单位 m。 */
    float engineer_x;   /**< 工程机器人 x 坐标，单位 m。 */
    float engineer_y;   /**< 工程机器人 y 坐标，单位 m。 */
    float standard_3_x; /**< 3 号步兵 x 坐标，单位 m。 */
    float standard_3_y; /**< 3 号步兵 y 坐标，单位 m。 */
    float standard_4_x; /**< 4 号步兵 x 坐标，单位 m。 */
    float standard_4_y; /**< 4 号步兵 y 坐标，单位 m。 */
    float reserved_1;   /**< 保留字段。 */
    float reserved_2;   /**< 保留字段。 */
} RefereeGroundPos;

/** @brief 雷达标记进度数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t mark_progress; /**< 雷达标记进度。 */
} RefereeRadarMark;

/** @brief 哨兵信息数据。 */
typedef struct REFEREE_PACKED
{
    uint32_t sentry_info;   /**< 哨兵自主决策信息。 */
    uint16_t sentry_info_2; /**< 哨兵自主决策信息 2。 */
} RefereeSentryInfo;

/** @brief 雷达信息数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t radar_info; /**< 雷达自主决策信息。 */
} RefereeRadarInfo;

/** @brief 机器人交互数据。 */
typedef struct REFEREE_PACKED
{
    uint16_t data_cmd_id;   /**< 子内容 ID。 */
    uint16_t sender_id;     /**< 发送者 ID。 */
    uint16_t receiver_id;   /**< 接收者 ID。 */
    uint8_t user_data[REFEREE_INTERACT_DATA_MAX_SIZE]; /**< 用户载荷。 */
} RefereeRobotInteract;

/** @brief 自定义机器人数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t data[30]; /**< 自定义载荷。 */
} RefereeCustomRobot;

/** @brief 小地图命令数据。 */
typedef struct REFEREE_PACKED
{
    float target_position_x; /**< 目标位置 x 坐标，单位 m。 */
    float target_position_y; /**< 目标位置 y 坐标，单位 m。 */
    uint8_t cmd_keyboard;    /**< 通用键盘键值。 */
    uint8_t target_robot_id; /**< 对方机器人 ID。 */
    uint16_t cmd_source;     /**< 信息来源 ID。 */
} RefereeMapCommand;

/** @brief 遥控控制数据。 */
typedef struct REFEREE_PACKED
{
    int16_t mouse_x;          /**< 鼠标 x 轴移动速度。 */
    int16_t mouse_y;          /**< 鼠标 y 轴移动速度。 */
    int16_t mouse_z;          /**< 鼠标滚轮移动速度。 */
    uint8_t left_button_down; /**< 鼠标左键状态。 */
    uint8_t right_button_down;/**< 鼠标右键状态。 */
    uint16_t keyboard_value;  /**< 键盘按键信息。 */
    uint16_t reserved;        /**< 保留字段。 */
} RefereeRcData;

/** @brief 小地图机器人数据。 */
typedef struct REFEREE_PACKED
{
    uint16_t hero_position_x;       /**< 英雄 x 位置，单位 cm。 */
    uint16_t hero_position_y;       /**< 英雄 y 位置，单位 cm。 */
    uint16_t engineer_position_x;   /**< 工程 x 位置，单位 cm。 */
    uint16_t engineer_position_y;   /**< 工程 y 位置，单位 cm。 */
    uint16_t infantry_3_position_x; /**< 3 号步兵 x 位置，单位 cm。 */
    uint16_t infantry_3_position_y; /**< 3 号步兵 y 位置，单位 cm。 */
    uint16_t infantry_4_position_x; /**< 4 号步兵 x 位置，单位 cm。 */
    uint16_t infantry_4_position_y; /**< 4 号步兵 y 位置，单位 cm。 */
    uint16_t infantry_5_position_x; /**< 5 号步兵 x 位置，单位 cm。 */
    uint16_t infantry_5_position_y; /**< 5 号步兵 y 位置，单位 cm。 */
    uint16_t sentry_position_x;     /**< 哨兵 x 位置，单位 cm。 */
    uint16_t sentry_position_y;     /**< 哨兵 y 位置，单位 cm。 */
} RefereeMapRobot;

/** @brief 自定义客户端数据。 */
typedef struct REFEREE_PACKED
{
    uint16_t key_value;      /**< 键盘键值。 */
    uint16_t x_position : 12;/**< 鼠标 X 轴像素位置。 */
    uint16_t mouse_left : 4; /**< 鼠标左键状态。 */
    uint16_t y_position : 12;/**< 鼠标 Y 轴像素位置。 */
    uint16_t mouse_right : 4;/**< 鼠标右键状态。 */
    uint16_t reserved;       /**< 保留字段。 */
} RefereeCustomClient;

/** @brief 小地图路径数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t intention;         /**< 意图。 */
    uint16_t start_position_x; /**< 路径起点 x，单位 dm。 */
    uint16_t start_position_y; /**< 路径起点 y，单位 dm。 */
    int8_t delta_x[49];        /**< 路径点 x 增量数组。 */
    int8_t delta_y[49];        /**< 路径点 y 增量数组。 */
    uint16_t sender_id;        /**< 发送者 ID。 */
} RefereeMapData;

/** @brief 自定义字符信息。 */
typedef struct REFEREE_PACKED
{
    uint16_t sender_id;       /**< 发送者 ID。 */
    uint16_t receiver_id;     /**< 接收者 ID。 */
    uint8_t user_data[30];    /**< UTF-16 字符数据。 */
} RefereeCustomInfo;

/** @brief 机器人自定义数据。 */
typedef struct REFEREE_PACKED
{
    uint8_t data[30]; /**< 自定义载荷。 */
} RefereeRobotCustom;

/** @brief 裁判系统完整原始帧。 */
typedef struct
{
    uint8_t data[REFEREE_FRAME_MAX_SIZE]; /**< 原始帧缓存。 */
    uint16_t len;                         /**< 原始帧长度，单位 byte。 */
    uint16_t cmd_id;                      /**< 命令 ID。 */
    uint16_t data_len;                    /**< 数据段长度，单位 byte。 */
    RefereeHeader header;                 /**< 解析后的帧头。 */
} RefereeFrame;

/** @brief 裁判系统解包上下文。 */
typedef struct
{
    uint8_t packet[REFEREE_FRAME_MAX_SIZE]; /**< 正在接收的帧缓存。 */
    uint16_t data_len;                      /**< 当前帧数据段长度。 */
    uint16_t index;                         /**< 当前写入位置。 */
    RefereeUnpackStep step;                 /**< 当前状态机步骤。 */
} RefereeUnpack;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口只表达设备能做什么，不把遥控器、裁判系统或电机解释成具体机器人控制动作。
 * ========================================================================== */

/**
 * @brief 复位裁判系统解包上下文。
 *
 * @param unpack 解包上下文。
 */
void referee_unpack_reset(RefereeUnpack *unpack);

/**
 * @brief 计算 CRC8。
 *
 * @param data 输入数据。
 * @param len 输入数据长度。
 * @param init CRC8 初始值。
 * @return CRC8 结果。
 */
uint8_t referee_crc8(const uint8_t *data, uint16_t len, uint8_t init);

/**
 * @brief 校验数据末尾 CRC8。
 *
 * @param data 输入数据，最后 1 byte 为 CRC8。
 * @param len 输入数据总长度。
 * @return 校验通过返回 1，否则返回 0。
 */
uint8_t referee_verify_crc8(const uint8_t *data, uint16_t len);

/**
 * @brief 向数据末尾写入 CRC8。
 *
 * @param data 输入输出数据。
 * @param len 输入数据总长度，最后 1 byte 用于保存 CRC8。
 */
void referee_append_crc8(uint8_t *data, uint16_t len);

/**
 * @brief 计算 CRC16。
 *
 * @param data 输入数据。
 * @param len 输入数据长度。
 * @param init CRC16 初始值。
 * @return CRC16 结果。
 */
uint16_t referee_crc16(const uint8_t *data, uint16_t len, uint16_t init);

/**
 * @brief 校验数据末尾 CRC16。
 *
 * @param data 输入数据，最后 2 byte 为 CRC16。
 * @param len 输入数据总长度。
 * @return 校验通过返回 1，否则返回 0。
 */
uint8_t referee_verify_crc16(const uint8_t *data, uint16_t len);

/**
 * @brief 向数据末尾写入 CRC16。
 *
 * @param data 输入输出数据。
 * @param len 输入数据总长度，最后 2 byte 用于保存 CRC16。
 */
void referee_append_crc16(uint8_t *data, uint16_t len);

/**
 * @brief 输入一个字节并尝试解析完整帧。
 *
 * @param unpack 解包上下文。
 * @param byte 当前输入字节。
 * @param frame 完整帧输出，可为空。
 * @return 产生完整帧返回 RefereeOk，未完成返回 RefereeNoFrame，错误返回对应状态。
 */
RefereeStatus referee_unpack_byte(RefereeUnpack *unpack, uint8_t byte, RefereeFrame *frame);

/**
 * @brief 输入一段字节流并解析完整帧。
 *
 * @param unpack 解包上下文。
 * @param data 输入字节流。
 * @param len 输入字节流长度。
 * @param frames 完整帧输出数组，可为空。
 * @param max_frames 输出数组容量。
 * @param frame_count 实际输出帧数。
 * @return 执行状态。
 */
RefereeStatus referee_unpack_buffer(RefereeUnpack *unpack,
                                    const uint8_t *data,
                                    uint16_t len,
                                    RefereeFrame *frames,
                                    uint16_t max_frames,
                                    uint16_t *frame_count);

/**
 * @brief 打包普通裁判系统帧。
 *
 * @param cmd_id 命令 ID。
 * @param payload 数据段。
 * @param payload_len 数据段长度。
 * @param seq 帧序号。
 * @param out 输出缓冲区。
 * @param out_size 输出缓冲区容量。
 * @param out_len 实际输出长度。
 * @return 打包状态。
 */
RefereeStatus referee_pack_frame(uint16_t cmd_id,
                                 const uint8_t *payload,
                                 uint16_t payload_len,
                                 uint8_t seq,
                                 uint8_t *out,
                                 uint16_t out_size,
                                 uint16_t *out_len);

/**
 * @brief 打包 0x0301 机器人交互帧。
 *
 * @param sender_id 发送者机器人 ID。
 * @param receiver_id 接收者机器人 ID。
 * @param data_cmd_id 交互子内容 ID。
 * @param data 用户载荷。
 * @param data_len 用户载荷长度。
 * @param seq 帧序号。
 * @param out 输出缓冲区。
 * @param out_size 输出缓冲区容量。
 * @param out_len 实际输出长度。
 * @return 打包状态。
 */
RefereeStatus referee_pack_interact(uint16_t sender_id,
                                    uint16_t receiver_id,
                                    uint16_t data_cmd_id,
                                    const uint8_t *data,
                                    uint16_t data_len,
                                    uint8_t seq,
                                    uint8_t *out,
                                    uint16_t out_size,
                                    uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_DEVICE_REFEREE_PROTOCOL_H
