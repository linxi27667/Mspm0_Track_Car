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
#define PATH_FISH_TRANSFER_TIMEOUT_MS     2u
#define PATH_FISH_FRAME_TIMEOUT_MS       50u

extern bsp_path_fish_t Path_Fish_Sensor;

void App_PathFish_Init(void);
void Path_Fish_Poll(uint32_t now);
bool Path_Fish_TakeFrame(void);
bool Path_Fish_TakeFault(void);
bool Path_Fish_IsFresh(uint32_t now);
uint32_t Path_Fish_FrameAge(uint32_t now);
void Path_Fish_SetBlackPolarity(bool black_is_one);
void Path_Fish_ToggleBlackPolarity(void);

#endif /* APP_PATH_FISH_H */
