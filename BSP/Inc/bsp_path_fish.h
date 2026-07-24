/**
 * @file    bsp_path_fish.h
 * @brief   Path Fish 12-channel I2C frame assembly and normalization.
 */
#ifndef BSP_PATH_FISH_H
#define BSP_PATH_FISH_H

#include <stdbool.h>
#include <stdint.h>

#define BSP_PATH_FISH_CHANNEL_COUNT      12u
#define BSP_PATH_FISH_HALF_CHANNEL_COUNT  6u
#define BSP_PATH_FISH_PACKET_SIZE          7u
#define BSP_PATH_FISH_MASK_ALL        0x0FFFu
#define BSP_PATH_FISH_PAIR_TIMEOUT_MS     15u

typedef enum {
    BSP_PATH_FISH_HALF_NONE = 0,
    BSP_PATH_FISH_HALF_FIRST,
    BSP_PATH_FISH_HALF_SECOND
} bsp_path_fish_half_t;

typedef struct {
    uint8_t raw[BSP_PATH_FISH_CHANNEL_COUNT];
    uint8_t digital[BSP_PATH_FISH_CHANNEL_COUNT];
    uint16_t mask;
    uint8_t black_count;
    bool black_is_one;

    uint8_t pending_bits[BSP_PATH_FISH_HALF_CHANNEL_COUNT];
    bsp_path_fish_half_t pending_half;
    uint32_t pending_tick;

    uint32_t frame_seq;
    uint32_t last_frame_tick;
    uint32_t error_count;
    bool data_ready;
    bool frame_fault;
} bsp_path_fish_t;

void Bsp_PathFish_Init(bsp_path_fish_t *sensor, bool black_is_one);
bool Bsp_PathFish_AcceptPacket(bsp_path_fish_t *sensor,
                               const uint8_t *packet,
                               uint8_t length,
                               uint32_t now);
void Bsp_PathFish_SetBlackPolarity(bsp_path_fish_t *sensor, bool black_is_one);
void Bsp_PathFish_NotifyTransportFault(bsp_path_fish_t *sensor);
uint32_t Bsp_PathFish_FrameAge(const bsp_path_fish_t *sensor, uint32_t now);
bool Bsp_PathFish_IsFresh(const bsp_path_fish_t *sensor,
                          uint32_t now,
                          uint32_t timeout_ms);

#endif /* BSP_PATH_FISH_H */
