/** @file app_log.h @brief 编译期日志分类包装。 */
#ifndef APP_LOG_H
#define APP_LOG_H

#include "app_common.h"
#include <elog.h>

#if APP_LOG_ESP01S_ENABLE
#define APP_ESP_LOG_I(...)       elog_i(__VA_ARGS__)
#define APP_ESP_LOG_W(...)       elog_w(__VA_ARGS__)
#define APP_ESP_LOG_E(...)       elog_e(__VA_ARGS__)
#else
#define APP_ESP_LOG_I(...)       ((void)0)
#define APP_ESP_LOG_W(...)       ((void)0)
#define APP_ESP_LOG_E(...)       ((void)0)
#endif

#if APP_LOG_GRAY_ADC_ENABLE
#define APP_GRAY_ADC_LOG_I(...)  elog_i(__VA_ARGS__)
#else
#define APP_GRAY_ADC_LOG_I(...)  ((void)0)
#endif

#if APP_LOG_GRAY_STATE_ENABLE
#define APP_GRAY_STATE_LOG_I(...) elog_i(__VA_ARGS__)
#define APP_GRAY_STATE_LOG_W(...) elog_w(__VA_ARGS__)
#else
#define APP_GRAY_STATE_LOG_I(...) ((void)0)
#define APP_GRAY_STATE_LOG_W(...) ((void)0)
#endif

#if APP_LOG_VEHICLE_ENABLE
#define APP_VEHICLE_LOG_I(...)   elog_i(__VA_ARGS__)
#define APP_VEHICLE_LOG_W(...)   elog_w(__VA_ARGS__)
#else
#define APP_VEHICLE_LOG_I(...)   ((void)0)
#define APP_VEHICLE_LOG_W(...)   ((void)0)
#endif

#if APP_LOG_TRACK_ENABLE
#define APP_TRACK_LOG_I(...)     elog_i(__VA_ARGS__)
#define APP_TRACK_LOG_W(...)     elog_w(__VA_ARGS__)
#else
#define APP_TRACK_LOG_I(...)     ((void)0)
#define APP_TRACK_LOG_W(...)     ((void)0)
#endif

#if APP_LOG_ROTATE_ENABLE
#define APP_ROTATE_LOG_I(...)    elog_i(__VA_ARGS__)
#define APP_ROTATE_LOG_W(...)    elog_w(__VA_ARGS__)
#else
#define APP_ROTATE_LOG_I(...)    ((void)0)
#define APP_ROTATE_LOG_W(...)    ((void)0)
#endif

#if APP_LOG_TUNING_ENABLE
#define APP_TUNING_LOG_I(...)    elog_i(__VA_ARGS__)
#define APP_TUNING_LOG_W(...)    elog_w(__VA_ARGS__)
#define APP_TUNING_LOG_D(...)    elog_d(__VA_ARGS__)
#else
#define APP_TUNING_LOG_I(...)    ((void)0)
#define APP_TUNING_LOG_W(...)    ((void)0)
#define APP_TUNING_LOG_D(...)    ((void)0)
#endif

#if APP_LOG_IMU_ENABLE
#define APP_IMU_LOG_I(...)       elog_i(__VA_ARGS__)
#define APP_IMU_LOG_W(...)       elog_w(__VA_ARGS__)
#define APP_IMU_LOG_E(...)       elog_e(__VA_ARGS__)
#else
#define APP_IMU_LOG_I(...)       ((void)0)
#define APP_IMU_LOG_W(...)       ((void)0)
#define APP_IMU_LOG_E(...)       ((void)0)
#endif

#if APP_LOG_DIAGNOSTIC_ENABLE
#define APP_DIAG_LOG_I(...)      elog_i(__VA_ARGS__)
#define APP_DIAG_LOG_W(...)      elog_w(__VA_ARGS__)
#define APP_DIAG_LOG_E(...)      elog_e(__VA_ARGS__)
#else
#define APP_DIAG_LOG_I(...)      ((void)0)
#define APP_DIAG_LOG_W(...)      ((void)0)
#define APP_DIAG_LOG_E(...)      ((void)0)
#endif

#endif /* APP_LOG_H */
