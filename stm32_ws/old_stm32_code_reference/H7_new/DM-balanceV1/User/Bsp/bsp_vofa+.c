#include "bsp_vofa+.h"
#include "bsp_usart.h"

// 按printf格式写，最后必须加\r\n
void Vofa_FireWater(const char *format, ...)
{
    uint8_t txBuffer[100];
    uint32_t n;
    va_list args;
    va_start(args, format);
    n = vsnprintf((char *)txBuffer, 100, format, args);
    HAL_UART_Transmit_IT(&huart10, (uint8_t *)txBuffer, n);
    va_end(args);
}

void Vofa_JustFloat(float *_data, uint8_t _num)
{
    static uint8_t buf[256];
    static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    memcpy(buf, _data, 4 * _num);              // 1. 数据
    memcpy(buf + 4 * _num, tail, 4);           // 2. 尾
    HAL_UART_Transmit_IT(&huart10, buf, 4 * _num + 4); // 3. 一帧发出
}
