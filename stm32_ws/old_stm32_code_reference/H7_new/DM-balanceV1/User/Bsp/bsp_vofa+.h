#ifndef _VOFA_H
#define _VOFA_H

#include "usart.h"
#include <stdio.h>
#include "stdint.h"
#include <string.h>
#include <stdarg.h>
/* 只写数组名，自动推元素个数 */
#define Vofa_JustFloat_AUTO(arr) Vofa_JustFloat((arr), (uint8_t)(sizeof(arr) / sizeof((arr)[0])))
			
void Vofa_FireWater(const char *format, ...);
void Vofa_JustFloat(float *_data, uint8_t _num);

#endif
