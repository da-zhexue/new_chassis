/**
 * @file ulog.c
 * @brief 日志模块(Log Module)
 * 单开一个任务来将日志输出到串口，保证线程安全。
 * 使用时记得将ulogTask加入FreeRTOS线程。
 * 给ulogTask的栈空间需大于两倍LOG_BUFFER_SIZE
 * @version 1.0
 * @date 2026-01-26
 */

#include "ulog.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "stream_buffer.h"
static StreamBufferHandle_t xStreamBuffer = NULL;

void ulogTask(void const *argument)
{
    uint8_t rx_buf[LOG_BUFFER_SIZE];
    xStreamBuffer = xStreamBufferCreate(LOG_BUFFER_SIZE * 2, 1);
    while(1) {
        const size_t received = xStreamBufferReceive(xStreamBuffer, rx_buf, LOG_BUFFER_SIZE, portMAX_DELAY);
        if (received > 0) {
            HAL_UART_Transmit(&LOG_HUART, rx_buf, received, 100);
        }
        else {
            osDelay(10);
        }
    }
}

void log_write(log_level_t level, const char *file, int line, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    
    char log_buffer[LOG_BUFFER_SIZE];
    char *buffer_ptr = log_buffer;
    int remaining_size = LOG_BUFFER_SIZE;
    
    const char *level_str;
    switch (level) {
        case LOG_LEVEL_ERROR: level_str = "ERROR"; break;
        case LOG_LEVEL_WARN:  level_str = "WARN";  break;
        case LOG_LEVEL_INFO:  level_str = "INFO";  break;
        case LOG_LEVEL_DEBUG: level_str = "DEBUG"; break;
        default:              level_str = "UNKNOWN"; break;
    }

    const int header_len = snprintf(buffer_ptr, remaining_size, "[%s] %s:%d: ", level_str, file, line);
    if (header_len > 0 && header_len < remaining_size) {
        buffer_ptr += header_len;
        remaining_size -= header_len;
    }
    
    if (remaining_size > 0) {
        int content_len = vsnprintf(buffer_ptr, remaining_size, fmt, args);
        if (content_len > 0) {
            if (content_len >= remaining_size) {
                content_len = remaining_size - 1;
            }
            buffer_ptr += content_len;
            remaining_size -= content_len;
        }
    }
    
    if (remaining_size > 1) {
        *buffer_ptr++ = '\r';
        *buffer_ptr++ = '\n';
    } else if (remaining_size > 0) {
        *buffer_ptr++ = '\n';
    }

    const size_t total_len = buffer_ptr - log_buffer;
    
    if (total_len > 0 && xStreamBuffer != NULL) {
        xStreamBufferSend(xStreamBuffer, log_buffer, total_len, portMAX_DELAY);
    }
    
    va_end(args);
}
