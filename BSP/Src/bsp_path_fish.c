/**
 * @file    bsp_path_fish.c
 * @brief   Path Fish protocol parser; contains no MCU-specific code.
 */
#include "bsp_path_fish.h"

#include <stddef.h>
#include <string.h>

static bool Bsp_PathFish_DecodeBit(uint8_t value, uint8_t *bit)
{
    if (bit == NULL)
    {
        return false;
    }
    if (value == 0u || value == (uint8_t)'0')
    {
        *bit = 0u;
        return true;
    }
    if (value == 1u || value == (uint8_t)'1')
    {
        *bit = 1u;
        return true;
    }
    return false;
}

static bsp_path_fish_half_t Bsp_PathFish_HeaderToHalf(uint8_t header)
{
    if (header == (uint8_t)'#')
    {
        return BSP_PATH_FISH_HALF_FIRST;
    }
    if (header == (uint8_t)'!')
    {
        return BSP_PATH_FISH_HALF_SECOND;
    }
    return BSP_PATH_FISH_HALF_NONE;
}

static void Bsp_PathFish_Publish(bsp_path_fish_t *sensor, uint32_t now)
{
    uint16_t mask = 0u;
    uint8_t black_count = 0u;
    uint8_t channel;

    for (channel = 0u; channel < BSP_PATH_FISH_CHANNEL_COUNT; channel++)
    {
        uint8_t is_black =
            sensor->raw[channel] == (sensor->black_is_one ? 1u : 0u) ? 1u : 0u;
        sensor->digital[channel] = is_black;
        if (is_black != 0u)
        {
            mask |= (uint16_t)(1u << channel);
            black_count++;
        }
    }

    sensor->mask = mask;
    sensor->black_count = black_count;
    sensor->frame_seq++;
    sensor->last_frame_tick = now;
    sensor->data_ready = true;
    sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
}

void Bsp_PathFish_Init(bsp_path_fish_t *sensor, bool black_is_one)
{
    if (sensor == NULL)
    {
        return;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->black_is_one = black_is_one;
}

bool Bsp_PathFish_AcceptPacket(bsp_path_fish_t *sensor,
                               const uint8_t *packet,
                               uint8_t length,
                               uint32_t now)
{
    bsp_path_fish_half_t half;
    uint8_t decoded[BSP_PATH_FISH_HALF_CHANNEL_COUNT];
    uint8_t index;

    if (sensor == NULL || packet == NULL ||
        length != BSP_PATH_FISH_PACKET_SIZE)
    {
        if (sensor != NULL)
        {
            sensor->error_count++;
            sensor->frame_fault = true;
            sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
        }
        return false;
    }

    half = Bsp_PathFish_HeaderToHalf(packet[0]);
    if (half == BSP_PATH_FISH_HALF_NONE)
    {
        sensor->error_count++;
        sensor->frame_fault = true;
        sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
        return false;
    }

    for (index = 0u; index < BSP_PATH_FISH_HALF_CHANNEL_COUNT; index++)
    {
        if (!Bsp_PathFish_DecodeBit(packet[index + 1u], &decoded[index]))
        {
            sensor->error_count++;
            sensor->frame_fault = true;
            sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
            return false;
        }
    }

    if (sensor->pending_half == BSP_PATH_FISH_HALF_NONE ||
        sensor->pending_half == half ||
        (uint32_t)(now - sensor->pending_tick) >
            BSP_PATH_FISH_PAIR_TIMEOUT_MS)
    {
        if (sensor->pending_half != BSP_PATH_FISH_HALF_NONE)
        {
            sensor->error_count++;
        }
        memcpy(sensor->pending_bits, decoded, sizeof(decoded));
        sensor->pending_half = half;
        sensor->pending_tick = now;
        return false;
    }

    if (sensor->pending_half == BSP_PATH_FISH_HALF_FIRST)
    {
        memcpy(&sensor->raw[0], sensor->pending_bits, sizeof(decoded));
        memcpy(&sensor->raw[BSP_PATH_FISH_HALF_CHANNEL_COUNT],
               decoded,
               sizeof(decoded));
    }
    else
    {
        memcpy(&sensor->raw[BSP_PATH_FISH_HALF_CHANNEL_COUNT],
               sensor->pending_bits,
               sizeof(decoded));
        memcpy(&sensor->raw[0], decoded, sizeof(decoded));
    }

    Bsp_PathFish_Publish(sensor, now);
    return true;
}

void Bsp_PathFish_SetBlackPolarity(bsp_path_fish_t *sensor, bool black_is_one)
{
    if (sensor == NULL || sensor->black_is_one == black_is_one)
    {
        return;
    }

    sensor->black_is_one = black_is_one;
    sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
    if (sensor->frame_seq != 0u)
    {
        Bsp_PathFish_Publish(sensor, sensor->last_frame_tick);
    }
}

void Bsp_PathFish_NotifyTransportFault(bsp_path_fish_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->error_count++;
    sensor->frame_fault = true;
    sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
}

uint32_t Bsp_PathFish_FrameAge(const bsp_path_fish_t *sensor, uint32_t now)
{
    if (sensor == NULL || sensor->frame_seq == 0u)
    {
        return UINT32_MAX;
    }
    return (uint32_t)(now - sensor->last_frame_tick);
}

bool Bsp_PathFish_IsFresh(const bsp_path_fish_t *sensor,
                          uint32_t now,
                          uint32_t timeout_ms)
{
    return sensor != NULL && sensor->frame_seq != 0u &&
           !sensor->frame_fault &&
           Bsp_PathFish_FrameAge(sensor, now) <= timeout_ms;
}
