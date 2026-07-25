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
#define BSP_PATH_FISH_CAL_CONFIRM_FRAMES  10u
#define BSP_PATH_FISH_WHITE_MAX_BLACK      1u
#define BSP_PATH_FISH_BLACK_MIN_BLACK     11u

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

    uint8_t stream_packet[BSP_PATH_FISH_PACKET_SIZE];
    uint8_t first_packet[BSP_PATH_FISH_PACKET_SIZE];
    uint8_t stream_count;
    uint32_t valid_packet_count;
    uint32_t stream_resync_count;

    uint32_t frame_seq;
    uint32_t last_frame_tick;
    uint32_t error_count;
    uint8_t white_surface_frames;
    uint8_t black_surface_frames;
    bool data_ready;
    bool frame_fault;
} bsp_path_fish_t;

void Bsp_PathFish_Init(bsp_path_fish_t *sensor, bool black_is_one);
bool Bsp_PathFish_AcceptPacket(bsp_path_fish_t *sensor,
                               const uint8_t *packet,
                               uint8_t length,
                               uint32_t now);
uint8_t Bsp_PathFish_AcceptBytes(bsp_path_fish_t *sensor,
                                 const uint8_t *bytes,
                                 uint8_t length,
                                 uint32_t now);
void Bsp_PathFish_SetBlackPolarity(bsp_path_fish_t *sensor, bool black_is_one);
void Bsp_PathFish_NotifyTransportFault(bsp_path_fish_t *sensor);
uint32_t Bsp_PathFish_FrameAge(const bsp_path_fish_t *sensor, uint32_t now);
bool Bsp_PathFish_IsFresh(const bsp_path_fish_t *sensor,
                          uint32_t now,
                          uint32_t timeout_ms);
bool Bsp_PathFish_WhiteSurfaceStable(const bsp_path_fish_t *sensor);
bool Bsp_PathFish_BlackSurfaceStable(const bsp_path_fish_t *sensor);

#endif /* BSP_PATH_FISH_H */
