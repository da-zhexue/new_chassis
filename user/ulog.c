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

static QueueHandle_t xLogQueue = NULL;
static SemaphoreHandle_t dmaTxSemaphore = NULL;

// 日志等级转字符串
char* level_to_string(const log_level_t level)
{
    switch (level)
    {
        case LOG_LEVEL_ERROR:
            return "ERROR";
        case LOG_LEVEL_WARN:
            return "WARN";
        case LOG_LEVEL_INFO:
            return "INFO";
        case LOG_LEVEL_DEBUG:
            return "DEBUG";
        default:
            return "UNKNOWN";
    }
}

#ifndef LOG_POOL_SIZE
#define LOG_POOL_SIZE 100
#endif

typedef struct {
    log_level_t level;
    char message[128];
} log_message_t;

typedef struct {
    log_message_t messages[LOG_POOL_SIZE];  // 静态内存池
    uint8_t in_use[LOG_POOL_SIZE];          // 使用标记
    SemaphoreHandle_t mutex;                // 内存池互斥锁
} log_memory_pool_t;

static log_memory_pool_t log_pool = {
    .mutex = NULL
};

// 内存池初始化
static void log_pool_init(void) {
    memset(log_pool.in_use, 0, sizeof(log_pool.in_use));
    log_pool.mutex = xSemaphoreCreateMutex();
}

// 分配消息
static log_message_t* log_pool_alloc(void) {
    if (log_pool.mutex == NULL) return NULL;
    if (xSemaphoreTake(log_pool.mutex, portMAX_DELAY) != pdTRUE) {
        return NULL;
    }

    log_message_t* msg = NULL;
    for (int i = 0; i < LOG_POOL_SIZE; i++) {
        if (!log_pool.in_use[i]) {
            log_pool.in_use[i] = 1;
            msg = &log_pool.messages[i];
            break;
        }
    }

    xSemaphoreGive(log_pool.mutex);
    return msg;
}

// 释放消息
static void log_pool_free(log_message_t* msg) {
    if (msg == NULL) return;
    if (log_pool.mutex == NULL) return;

    if (xSemaphoreTake(log_pool.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    ptrdiff_t index = msg - log_pool.messages;
    if (index >= 0 && index < LOG_POOL_SIZE) {
        log_pool.in_use[index] = 0;
    }

    xSemaphoreGive(log_pool.mutex);
}

// 日志写入，异步串口输出
void log_write(const log_level_t level, const char *fmt, ...) {
    if (xLogQueue == NULL) return;

    log_message_t* msg = log_pool_alloc();
    if (msg == NULL) {
        // 内存池耗尽
        return;
    }

    msg->level = level;

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(msg->message, sizeof(msg->message), fmt, args);
    va_end(args);

    if (len < 0 || len >= sizeof(msg->message)) {
        msg->message[sizeof(msg->message) - 1] = '\0';
    }

    // 用指针传输消息
    if (xQueueSendToBack(xLogQueue, &msg, 0) != pdTRUE) {
        log_pool_free(msg);
    }
}

static char dmaBuffer[256]; // DMA发送缓冲区，仅ulogTask中使用

void ulogTask(void const *argument)
{
    log_pool_init();
    xLogQueue = xQueueCreate(10, sizeof(log_message_t*)); // 队列存指针
    if (dmaTxSemaphore == NULL) {
        dmaTxSemaphore = xSemaphoreCreateBinary();
        // 启动时确保信号量可用，否则任务第一次发送会卡住
        xSemaphoreGive(dmaTxSemaphore);
    }

    log_message_t *msg = NULL;
    while(1) {
        if (xQueueReceive(xLogQueue, &msg, portMAX_DELAY) == pdTRUE) {
            // 格式化日志消息
            int len = snprintf(dmaBuffer, sizeof(dmaBuffer), "[%s]: %s\r\n",
                               level_to_string(msg->level), msg->message);
            if (len > 0) {
                // 等待前一次DMA发送完成
                if (xSemaphoreTake(dmaTxSemaphore, portMAX_DELAY) == pdTRUE) {
                    // 启动DMA发送
                    if (HAL_UART_Transmit_DMA(&LOG_HUART, (uint8_t*)dmaBuffer, len) == HAL_OK) {
                        // 等待DMA完成(由回调释放信号量)
                        //xSemaphoreTake(dmaTxSemaphore, portMAX_DELAY);
                    } else {
                        // DMA启动失败时，主动释放信号量，防死锁
                        xSemaphoreGive(dmaTxSemaphore);
                    }
                }
            }
            log_pool_free(msg);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &LOG_HUART) { // LOG_HUART为你的串口句柄
        if (dmaTxSemaphore != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(dmaTxSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}