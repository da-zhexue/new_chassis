#ifndef __ULOG_H__
#define __ULOG_H__
#include "typedef.h"

#define LOG_HUART huart1

typedef enum {
    LOG_LEVEL_ERROR = 1,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
} log_level_t;

#define LOG_BUFFER_SIZE 64

void log_write(log_level_t level, const char *fmt, ...);
void ulogTask(void const *argument);

#ifdef LOG_ENABLE
#define LOG_ERROR(fmt, ...) log_write(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  log_write(LOG_LEVEL_WARN, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  log_write(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) log_write(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#else
#define LOG_ERROR(fmt, ...) ((void)0)
#define LOG_WARN(fmt, ...)  ((void)0)
#define LOG_INFO(fmt, ...)  ((void)0)
#define LOG_DEBUG(fmt, ...) ((void)0)
#endif

#endif
