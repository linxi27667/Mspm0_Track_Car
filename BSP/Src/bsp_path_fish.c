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
    if (black_count <= BSP_PATH_FISH_WHITE_MAX_BLACK)
    {
        if (sensor->white_surface_frames < UINT8_MAX)
        {
            sensor->white_surface_frames++;
        }
    }
    else
    {
        sensor->white_surface_frames = 0u;
    }
    if (black_count >= BSP_PATH_FISH_BLACK_MIN_BLACK)
    {
        if (sensor->black_surface_frames < UINT8_MAX)
        {
            sensor->black_surface_frames++;
        }
    }
    else
    {
        sensor->black_surface_frames = 0u;
    }
    sensor->frame_seq++;
    sensor->last_frame_tick = now;
    sensor->data_ready = true;
    sensor->frame_fault = false;
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

uint8_t Bsp_PathFish_AcceptBytes(bsp_path_fish_t *sensor,
                                 const uint8_t *bytes,
                                 uint8_t length,
                                 uint32_t now)
{
    uint8_t accepted = 0u;
    uint8_t index;

    if (sensor == NULL || bytes == NULL)
    {
        return 0u;
    }

    for (index = 0u; index < length; index++)
    {
        uint8_t value = bytes[index];
        uint8_t decoded;

        if (Bsp_PathFish_HeaderToHalf(value) != BSP_PATH_FISH_HALF_NONE)
        {
            if (sensor->stream_count != 0u)
            {
                sensor->stream_resync_count++;
                sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
            }
            sensor->stream_packet[0] = value;
            sensor->stream_count = 1u;
            continue;
        }

        if (sensor->stream_count == 0u)
        {
            continue;
        }
        if (!Bsp_PathFish_DecodeBit(value, &decoded))
        {
            sensor->stream_count = 0u;
            sensor->stream_resync_count++;
            sensor->pending_half = BSP_PATH_FISH_HALF_NONE;
            continue;
        }

        sensor->stream_packet[sensor->stream_count] = value;
        sensor->stream_count++;
        if (sensor->stream_count == BSP_PATH_FISH_PACKET_SIZE)
        {
            if (sensor->valid_packet_count == 0u)
            {
                memcpy(sensor->first_packet, sensor->stream_packet,
                       BSP_PATH_FISH_PACKET_SIZE);
            }
            sensor->valid_packet_count++;
            (void)Bsp_PathFish_AcceptPacket(
                sensor, sensor->stream_packet, BSP_PATH_FISH_PACKET_SIZE, now);
            sensor->stream_count = 0u;
            accepted++;
        }
    }

    return accepted;
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
    sensor->stream_count = 0u;
    sensor->white_surface_frames = 0u;
    sensor->black_surface_frames = 0u;
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

bool Bsp_PathFish_WhiteSurfaceStable(const bsp_path_fish_t *sensor)
{
    return sensor != NULL &&
           sensor->white_surface_frames >= BSP_PATH_FISH_CAL_CONFIRM_FRAMES;
}

bool Bsp_PathFish_BlackSurfaceStable(const bsp_path_fish_t *sensor)
{
    return sensor != NULL &&
           sensor->black_surface_frames >= BSP_PATH_FISH_CAL_CONFIRM_FRAMES;
}
