/**
 * @file    app_path_fish.h
 * @brief   Path Fish hardware binding and non-blocking I2C service.
 */
#ifndef APP_PATH_FISH_H
#define APP_PATH_FISH_H

#include <stdbool.h>
#include <stdint.h>

#include "app_common.h"
#include "bsp_path_fish.h"
#include "ti_msp_dl_config.h"

#define PATH_FISH_I2C_ADDRESS          0x48u
#define PATH_FISH_REQUEST_PERIOD_MS       5u
#define PATH_FISH_TRANSFER_TIMEOUT_MS    10u
#define PATH_FISH_FAULT_RETRY_PERIOD_MS 250u
#define PATH_FISH_BUS_RECOVERY_PERIOD_MS 1000u
#define PATH_FISH_FAULT_BACKOFF_COUNT      3u
#define PATH_FISH_FRAME_TIMEOUT_MS       50u
#define PATH_FISH_BUS_SCL_HIGH_MASK     0x01u
#define PATH_FISH_BUS_SDA_HIGH_MASK     0x02u

typedef enum {
    PATH_FISH_TRANSPORT_OK = 0,
    PATH_FISH_TRANSPORT_TIMEOUT,
    PATH_FISH_TRANSPORT_NACK,
    PATH_FISH_TRANSPORT_ARBITRATION_LOST,
    PATH_FISH_TRANSPORT_SHORT_READ,
    PATH_FISH_TRANSPORT_OVERFLOW,
    PATH_FISH_TRANSPORT_BUS_STUCK
} path_fish_transport_status_t;

typedef struct {
    uint32_t timeout_count;
    uint32_t nack_count;
    uint32_t arbitration_lost_count;
    uint32_t short_read_count;
    uint32_t overflow_count;
    uint32_t bus_stuck_count;
    uint32_t recovery_attempt_count;
    uint32_t recovery_success_count;
    uint32_t last_controller_status;
    uint32_t last_bus_monitor;
    uint32_t current_controller_status;
    uint32_t current_bus_monitor;
    uint32_t transaction_count;
    uint32_t packet_count;
    uint32_t stream_resync_count;
    uint8_t first_packet[BSP_PATH_FISH_PACKET_SIZE];
    path_fish_transport_status_t status;
} path_fish_transport_diag_t;

extern bsp_path_fish_t Path_Fish_Sensor;

void App_PathFish_Init(void);
void Path_Fish_Poll(uint32_t now);
bool Path_Fish_TakeFrame(void);
bool Path_Fish_TakeFault(void);
bool Path_Fish_IsFresh(uint32_t now);
uint32_t Path_Fish_FrameAge(uint32_t now);
void Path_Fish_GetTransportDiag(path_fish_transport_diag_t *diag);
const char *Path_Fish_TransportStatusText(void);
void Path_Fish_SetBlackPolarity(bool black_is_one);
void Path_Fish_ToggleBlackPolarity(void);

#endif /* APP_PATH_FISH_H */
