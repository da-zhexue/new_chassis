#ifndef __ULOG_H__
#define __ULOG_H__
#include "typedef.h"

#define LOG_HUART huart6

typedef enum {
    LOG_LEVEL_ERROR = 1,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
} log_level_t;

#define LOG_BUFFER_SIZE 128

void log_write(log_level_t level, const char *file, int line, const char *fmt, ...);
void ulogTask(void const *argument);

#ifdef LOG_ENABLE
#define LOG_ERROR(fmt, ...) log_write(LOG_LEVEL_ERROR, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  log_write(LOG_LEVEL_WARN, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)  log_write(LOG_LEVEL_INFO, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) log_write(LOG_LEVEL_DEBUG, __FILE__, __LINE__, fmt, ##__VA_ARGS__)
#else
#define LOG_ERROR(fmt, ...) ((void)0)
#define LOG_WARN(fmt, ...)  ((void)0)
#define LOG_INFO(fmt, ...)  ((void)0)
#define LOG_DEBUG(fmt, ...) ((void)0)
#endif

#endif
