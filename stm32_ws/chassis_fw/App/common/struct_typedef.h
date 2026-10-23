/**
  * @file       struct_typedef.h
  * @brief      工程基础类型定义
  * @note       定义 fp32、bool_t 等通用类型别名。
  */
#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>

/* 上游代码使用 Keil 的 __weak / __packed 关键字，GCC 下需要显式映射。
 * __packed 的用法已改为 `struct __attribute__((packed))`，这里只补 __weak。 */
#ifndef __weak
#define __weak __attribute__((weak))
#endif

typedef uint8_t bool_t;
typedef float fp32;
typedef double fp64;

#endif
