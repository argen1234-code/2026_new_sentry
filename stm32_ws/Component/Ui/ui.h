/**
 ******************************************************************************
 * @file    ui.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   裁判客户端 UI 组件公共接口。
 *
 * @note    本组件负责裁判客户端图元、字符串和删除命令的脏标记与分包发送。
 * @note    本组件只调用 Devices/Referee 发送 0x0301 交互帧，不访问 USART、RTOS 或业务任务。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_COMPONENT_UI_H
#define PRINTK_COMPONENT_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 默认图元槽位数量。 */

#ifndef UI_FIGURE_MAX_DEFAULT
#define UI_FIGURE_MAX_DEFAULT 16U
#endif

/** @brief 默认字符串槽位数量。 */
#ifndef UI_STRING_MAX_DEFAULT
#define UI_STRING_MAX_DEFAULT 16U
#endif

/** @brief 裁判客户端字符串最大长度，单位 byte。 */
#ifndef UI_TEXT_LEN
#define UI_TEXT_LEN 30U
#endif

/** @brief 裁判客户端图元名称长度，单位 byte。 */
#ifndef UI_NAME_LEN
#define UI_NAME_LEN 3U
#endif

/** @brief 单帧最多携带的图元数量。 */
#ifndef UI_PACKET_FIG_MAX
#define UI_PACKET_FIG_MAX 7U
#endif

/** @brief 机器人 ID 到客户端 ID 的默认偏移。 */
#ifndef UI_CLIENT_ID_OFFSET
#define UI_CLIENT_ID_OFFSET 0x0100U
#endif

/** @brief 每次 ui_step 默认发送包数量。 */
#ifndef UI_SEND_PER_STEP_DEFAULT
#define UI_SEND_PER_STEP_DEFAULT 1U
#endif

/** @brief 单个图元协议载荷长度，单位 byte。 */
#define UI_GRAPHIC_SIZE ((uint16_t)15U)

/** @brief 单个字符串协议载荷长度，单位 byte。 */
#define UI_STRING_SIZE ((uint16_t)45U)

#if defined(__GNUC__) || defined(__CC_ARM) || defined(__ARMCC_VERSION)
/** @brief 裁判客户端 UI 协议 packed 修饰。 */
#define UI_PACKED __attribute__((packed))
#else
#error "UI_PACKED not defined for this compiler"
#endif

/** @brief UI 组件执行状态。 */
typedef enum
{
    UiOk = 0,       /**< 执行成功。 */
    UiNull = 1,     /**< 输入指针为空。 */
    UiArg = 2,      /**< 输入参数非法。 */
    UiLen = 3,      /**< 输入长度非法。 */
    UiBadState = 4, /**< 当前状态不允许执行。 */
    UiIo = 5        /**< 裁判系统发送失败。 */
} UiStatus;

/** @brief 裁判客户端图元操作类型。 */
typedef enum
{
    UiOpNone = 0,   /**< 无操作。 */
    UiOpAdd = 1,    /**< 新增图元。 */
    UiOpModify = 2, /**< 修改图元。 */
    UiOpDelete = 3  /**< 删除图元。 */
} UiOp;

/** @brief 裁判客户端图元类型。 */
typedef enum
{
    UiFigureLine = 0,    /**< 直线。 */
    UiFigureRect = 1,    /**< 矩形。 */
    UiFigureCircle = 2,  /**< 圆形。 */
    UiFigureEllipse = 3, /**< 椭圆。 */
    UiFigureArc = 4,     /**< 圆弧。 */
    UiFigureFloat = 5,   /**< 浮点数。 */
    UiFigureInt = 6,     /**< 整数。 */
    UiFigureString = 7   /**< 字符串。 */
} UiFigureType;

/** @brief 裁判客户端图元颜色。 */
typedef enum
{
    UiColorTeam = 0,    /**< 红蓝主色。 */
    UiColorYellow = 1,  /**< 黄色。 */
    UiColorGreen = 2,   /**< 绿色。 */
    UiColorOrange = 3,  /**< 橙色。 */
    UiColorPurple = 4,  /**< 紫红色。 */
    UiColorPink = 5,    /**< 粉色。 */
    UiColorCyan = 6,    /**< 青色。 */
    UiColorBlack = 7,   /**< 黑色。 */
    UiColorWhite = 8    /**< 白色。 */
} UiColor;

/** @brief 裁判客户端通用图元载荷。 */
typedef struct UI_PACKED
{
    uint8_t name[UI_NAME_LEN]; /**< 图元名称，高字节在前。 */
    uint32_t op : 3;           /**< UiOp 操作类型。 */
    uint32_t type : 3;         /**< UiFigureType 图元类型。 */
    uint32_t layer : 4;        /**< 图层编号。 */
    uint32_t color : 4;        /**< UiColor 颜色。 */
    uint32_t a : 9;            /**< 类型相关参数 A。 */
    uint32_t b : 9;            /**< 类型相关参数 B。 */
    uint32_t width : 10;       /**< 线宽。 */
    uint32_t start_x : 11;     /**< 起点 x 坐标。 */
    uint32_t start_y : 11;     /**< 起点 y 坐标。 */
    uint32_t c : 10;           /**< 类型相关参数 C，圆形时为半径。 */
    uint32_t end_x : 11;       /**< 终点 x 坐标或类型相关参数 D。 */
    uint32_t end_y : 11;       /**< 终点 y 坐标或类型相关参数 E。 */
} UiGraphic;

/** @brief 裁判客户端字符串载荷。 */
typedef struct UI_PACKED
{
    uint8_t name[UI_NAME_LEN]; /**< 字符串图元名称，高字节在前。 */
    uint32_t op : 3;           /**< UiOp 操作类型。 */
    uint32_t type : 3;         /**< 固定为 UiFigureString。 */
    uint32_t layer : 4;        /**< 图层编号。 */
    uint32_t color : 4;        /**< UiColor 颜色。 */
    uint32_t font_size : 9;    /**< 字号。 */
    uint32_t str_len : 9;      /**< 字符串长度，最大 UI_TEXT_LEN。 */
    uint32_t width : 10;       /**< 线宽。 */
    uint32_t start_x : 11;     /**< 起点 x 坐标。 */
    uint32_t start_y : 11;     /**< 起点 y 坐标。 */
    uint32_t reserved_c : 10;  /**< 协议保留字段。 */
    uint32_t reserved_d : 11;  /**< 协议保留字段。 */
    uint32_t reserved_e : 11;  /**< 协议保留字段。 */
    char text[UI_TEXT_LEN];    /**< 字符串内容。 */
} UiString;

#if defined(__cplusplus)
static_assert(sizeof(UiGraphic) == UI_GRAPHIC_SIZE, "UiGraphic size must be 15 bytes");
static_assert(sizeof(UiString) == UI_STRING_SIZE, "UiString size must be 45 bytes");
#else
_Static_assert(sizeof(UiGraphic) == UI_GRAPHIC_SIZE, "UiGraphic size must be 15 bytes");
_Static_assert(sizeof(UiString) == UI_STRING_SIZE, "UiString size must be 45 bytes");
#endif

/** @brief UI 组件配置。 */
typedef struct
{
    uint16_t sender_id;    /**< 发送者机器人 ID。 */
    uint16_t receiver_id;  /**< 接收者客户端 ID。 */
    uint8_t send_per_step; /**< 每次 ui_step 最多发送包数，0 使用默认值。 */
} UiCfg;

/** @brief UI 组件诊断信息。 */
typedef struct
{
    uint32_t tx_count;           /**< 成功发送包数量。 */
    uint32_t tx_error_count;     /**< 发送失败包数量。 */
    uint32_t set_graphic_count;  /**< 图元设置次数。 */
    uint32_t set_string_count;   /**< 字符串设置次数。 */
    uint32_t delete_count;       /**< 删除命令设置次数。 */
    uint16_t dirty_graphic_count;/**< 当前待发送图元数量。 */
    uint16_t dirty_string_count; /**< 当前待发送字符串数量。 */
    uint16_t last_data_cmd_id;   /**< 最近一次交互子内容 ID。 */
    uint16_t last_payload_len;   /**< 最近一次 0x0301 数据段总长度。 */
    uint8_t delete_pending;      /**< 非 0 表示存在待发送删除命令。 */
    uint8_t last_status;         /**< 最近一次发送返回状态。 */
} UiDiag;

/** @brief UI 组件上下文。 */
typedef struct
{
    UiCfg cfg;                                      /**< 配置副本。 */
    UiGraphic graphics[UI_FIGURE_MAX_DEFAULT];      /**< 当前图元槽位。 */
    UiString strings[UI_STRING_MAX_DEFAULT];        /**< 当前字符串槽位。 */
    uint8_t graphic_used[UI_FIGURE_MAX_DEFAULT];    /**< 图元槽位使用标记。 */
    uint8_t string_used[UI_STRING_MAX_DEFAULT];     /**< 字符串槽位使用标记。 */
    uint8_t graphic_dirty[UI_FIGURE_MAX_DEFAULT];   /**< 图元脏标记。 */
    uint8_t string_dirty[UI_STRING_MAX_DEFAULT];    /**< 字符串脏标记。 */
    UiDiag diag;                                    /**< 诊断信息。 */
    uint8_t delete_type;                            /**< 待发送删除类型。 */
    uint8_t delete_layer;                           /**< 待发送删除图层。 */
    uint8_t delete_pending;                         /**< 非 0 表示删除命令待发送。 */
    uint8_t initialized;                            /**< 非 0 表示已初始化。 */
} Ui;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化 UI 组件。
 *
 * 初始化会加载默认 HUD 图元，并将首帧标记为新增操作。
 *
 * @param ui UI 组件上下文。
 * @param cfg UI 组件配置。
 * @return 执行状态。
 */
UiStatus ui_init(Ui *ui, const UiCfg *cfg);

/**
 * @brief 复位 UI 组件。
 *
 * 复位保留配置，重新加载默认 HUD 图元，并将首帧标记为新增操作。
 *
 * @param ui UI 组件上下文。
 * @return 执行状态。
 */
UiStatus ui_reset(Ui *ui);

/**
 * @brief 设置一个图元槽位。
 *
 * @param ui UI 组件上下文。
 * @param index 图元槽位索引。
 * @param graphic 图元配置。
 * @return 执行状态。
 */
UiStatus ui_set_graphic(Ui *ui, uint8_t index, const UiGraphic *graphic);

/**
 * @brief 设置一个字符串槽位。
 *
 * @param ui UI 组件上下文。
 * @param index 字符串槽位索引。
 * @param string 字符串配置。
 * @return 执行状态。
 */
UiStatus ui_set_string(Ui *ui, uint8_t index, const UiString *string);

/**
 * @brief 设置删除图层命令。
 *
 * @param ui UI 组件上下文。
 * @param delete_type 删除类型。
 * @param layer 删除图层。
 * @return 执行状态。
 */
UiStatus ui_delete_layer(Ui *ui, uint8_t delete_type, uint8_t layer);

/**
 * @brief 发送当前全部待发送 UI 数据。
 *
 * @param ui UI 组件上下文。
 * @return 执行状态。
 */
UiStatus ui_flush(Ui *ui);

/**
 * @brief 执行一次 UI 发送周期。
 *
 * @param ui UI 组件上下文。
 * @return 执行状态。
 */
UiStatus ui_step(Ui *ui);

/**
 * @brief 获取 UI 组件诊断快照。
 *
 * @param ui UI 组件上下文。
 * @param out 诊断信息输出。
 * @return 执行状态。
 */
UiStatus ui_diag(const Ui *ui, UiDiag *out);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_COMPONENT_UI_H
