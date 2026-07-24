/**
 * @file    app_path_fish.c
 * @brief   I2C0 binding for the Path Fish 12-channel line sensor.
 */
#include "app_path_fish.h"

typedef enum {
    PATH_FISH_TRANSFER_IDLE = 0,
    PATH_FISH_TRANSFER_RX,
    PATH_FISH_TRANSFER_COMPLETE,
    PATH_FISH_TRANSFER_ERROR
} path_fish_transfer_state_t;

static volatile path_fish_transfer_state_t s_transfer_state;
static volatile uint8_t s_rx_packet[BSP_PATH_FISH_PACKET_SIZE];
static volatile uint8_t s_rx_count;
static uint32_t s_transfer_started_at;
static uint32_t s_last_request_at;

bsp_path_fish_t Path_Fish_Sensor;

static void HW_PathFish_DrainRxFifo(void)
{
    while (!DL_I2C_isControllerRXFIFOEmpty(PATH_FISH_I2C_INST))
    {
        uint8_t value = DL_I2C_receiveControllerData(PATH_FISH_I2C_INST);
        if (s_rx_count < BSP_PATH_FISH_PACKET_SIZE)
        {
            s_rx_packet[s_rx_count] = value;
            s_rx_count++;
        }
        else
        {
            s_transfer_state = PATH_FISH_TRANSFER_ERROR;
        }
    }
}

static void HW_PathFish_ResetTransfer(void)
{
    DL_I2C_disableController(PATH_FISH_I2C_INST);
    DL_I2C_resetControllerTransfer(PATH_FISH_I2C_INST);
    DL_I2C_flushControllerRXFIFO(PATH_FISH_I2C_INST);
    DL_I2C_enableController(PATH_FISH_I2C_INST);
    s_rx_count = 0u;
    s_transfer_state = PATH_FISH_TRANSFER_IDLE;
}

static void HW_PathFish_StartRead(uint32_t now)
{
    s_rx_count = 0u;
    s_transfer_started_at = now;
    s_last_request_at = now;
    s_transfer_state = PATH_FISH_TRANSFER_RX;
    DL_I2C_startControllerTransfer(
        PATH_FISH_I2C_INST,
        PATH_FISH_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        BSP_PATH_FISH_PACKET_SIZE);
}

void App_PathFish_Init(void)
{
    Bsp_PathFish_Init(&Path_Fish_Sensor, true);
    s_transfer_started_at = 0u;
    s_last_request_at = (uint32_t)(0u - PATH_FISH_REQUEST_PERIOD_MS);
    HW_PathFish_ResetTransfer();

    DL_I2C_enableInterrupt(
        PATH_FISH_I2C_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_CONTROLLER_NACK |
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    NVIC_ClearPendingIRQ(PATH_FISH_I2C_INST_INT_IRQN);
    NVIC_EnableIRQ(PATH_FISH_I2C_INST_INT_IRQN);
}

void Path_Fish_Poll(uint32_t now)
{
    if (s_transfer_state == PATH_FISH_TRANSFER_COMPLETE)
    {
        uint8_t packet[BSP_PATH_FISH_PACKET_SIZE];
        uint8_t index;

        for (index = 0u; index < BSP_PATH_FISH_PACKET_SIZE; index++)
        {
            packet[index] = s_rx_packet[index];
        }
        s_transfer_state = PATH_FISH_TRANSFER_IDLE;
        (void)Bsp_PathFish_AcceptPacket(
            &Path_Fish_Sensor, packet, BSP_PATH_FISH_PACKET_SIZE, now);
    }
    else if (s_transfer_state == PATH_FISH_TRANSFER_ERROR)
    {
        Bsp_PathFish_NotifyTransportFault(&Path_Fish_Sensor);
        HW_PathFish_ResetTransfer();
    }
    else if (s_transfer_state == PATH_FISH_TRANSFER_RX &&
             (uint32_t)(now - s_transfer_started_at) >=
                 PATH_FISH_TRANSFER_TIMEOUT_MS)
    {
        Bsp_PathFish_NotifyTransportFault(&Path_Fish_Sensor);
        HW_PathFish_ResetTransfer();
    }

    if (s_transfer_state == PATH_FISH_TRANSFER_IDLE &&
        (uint32_t)(now - s_last_request_at) >= PATH_FISH_REQUEST_PERIOD_MS &&
        (DL_I2C_getControllerStatus(PATH_FISH_I2C_INST) &
         DL_I2C_CONTROLLER_STATUS_IDLE) != 0u)
    {
        HW_PathFish_StartRead(now);
    }
}

bool Path_Fish_TakeFrame(void)
{
    bool ready = Path_Fish_Sensor.data_ready;
    Path_Fish_Sensor.data_ready = false;
    return ready;
}

bool Path_Fish_TakeFault(void)
{
    bool fault = Path_Fish_Sensor.frame_fault;
    Path_Fish_Sensor.frame_fault = false;
    return fault;
}

bool Path_Fish_IsFresh(uint32_t now)
{
    return Bsp_PathFish_IsFresh(
        &Path_Fish_Sensor, now, PATH_FISH_FRAME_TIMEOUT_MS);
}

uint32_t Path_Fish_FrameAge(uint32_t now)
{
    return Bsp_PathFish_FrameAge(&Path_Fish_Sensor, now);
}

void Path_Fish_SetBlackPolarity(bool black_is_one)
{
    Bsp_PathFish_SetBlackPolarity(&Path_Fish_Sensor, black_is_one);
}

void Path_Fish_ToggleBlackPolarity(void)
{
    Path_Fish_SetBlackPolarity(!Path_Fish_Sensor.black_is_one);
}

void PATH_FISH_I2C_INST_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(PATH_FISH_I2C_INST))
    {
    case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
        HW_PathFish_DrainRxFifo();
        break;

    case DL_I2C_IIDX_CONTROLLER_RX_DONE:
        HW_PathFish_DrainRxFifo();
        s_transfer_state =
            s_rx_count == BSP_PATH_FISH_PACKET_SIZE
                ? PATH_FISH_TRANSFER_COMPLETE
                : PATH_FISH_TRANSFER_ERROR;
        break;

    case DL_I2C_IIDX_CONTROLLER_NACK:
    case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        s_transfer_state = PATH_FISH_TRANSFER_ERROR;
        break;

    default:
        break;
    }
}
