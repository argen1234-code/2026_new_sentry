/**
  * @file       remote_control.h
  * @brief      遥控器类型桩（本工程不接 DBUS）
  * @note       底盘代码只保存指针，不读取字段；指针指向全零结构。
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

typedef struct
{
    struct
    {
        int16_t ch[8];
        char s[4];
    } rc;
    struct
    {
        uint16_t v;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;
} RC_ctrl_t;

RC_ctrl_t *get_remote_control_point(void);

#endif
