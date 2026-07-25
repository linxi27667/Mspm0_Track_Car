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

typedef enum {
    PATH_FISH_TRANSFER_FAULT_NONE = 0,
    PATH_FISH_TRANSFER_FAULT_NACK,
    PATH_FISH_TRANSFER_FAULT_ARBITRATION_LOST,
    PATH_FISH_TRANSFER_FAULT_SHORT_READ,
    PATH_FISH_TRANSFER_FAULT_OVERFLOW
} path_fish_transfer_fault_t;

static volatile path_fish_transfer_state_t s_transfer_state;
static volatile path_fish_transfer_fault_t s_transfer_fault;
static volatile uint8_t s_rx_packet[BSP_PATH_FISH_PACKET_SIZE];
static volatile uint8_t s_rx_count;
static uint32_t s_transfer_started_at;
static uint32_t s_last_request_at;
static uint32_t s_last_bus_recovery_at;
static uint8_t s_consecutive_transport_faults;
static path_fish_transport_diag_t s_transport_diag;

bsp_path_fish_t Path_Fish_Sensor;

#define PATH_FISH_BUS_LINES (GPIO_PATH_FISH_I2C_SDA_PIN | \
                             GPIO_PATH_FISH_I2C_SCL_PIN)
#define PATH_FISH_BUS_LEVEL_MASK (PATH_FISH_BUS_SCL_HIGH_MASK | \
                                  PATH_FISH_BUS_SDA_HIGH_MASK)

static void HW_PathFish_RecoveryDelay(void)
{
    delay_cycles(CPUCLK_FREQ / 200000u);
}

static bool HW_PathFish_RecoverBus(void)
{
    uint8_t pulse;
    bool released;

    NVIC_DisableIRQ(PATH_FISH_I2C_INST_INT_IRQN);
    DL_I2C_disableController(PATH_FISH_I2C_INST);
    DL_GPIO_disableOutput(GPIO_PATH_FISH_I2C_SDA_PORT,
                          PATH_FISH_BUS_LINES);
    DL_GPIO_initDigitalInputFeatures(
        GPIO_PATH_FISH_I2C_IOMUX_SDA, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        GPIO_PATH_FISH_I2C_IOMUX_SCL, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_PATH_FISH_I2C_SCL_PORT, PATH_FISH_BUS_LINES);

    for (pulse = 0u; pulse < 9u; pulse++)
    {
        if ((DL_GPIO_readPins(GPIO_PATH_FISH_I2C_SDA_PORT,
                              GPIO_PATH_FISH_I2C_SDA_PIN) &
             GPIO_PATH_FISH_I2C_SDA_PIN) != 0u)
        {
            break;
        }
        DL_GPIO_enableOutput(GPIO_PATH_FISH_I2C_SCL_PORT,
                             GPIO_PATH_FISH_I2C_SCL_PIN);
        HW_PathFish_RecoveryDelay();
        DL_GPIO_disableOutput(GPIO_PATH_FISH_I2C_SCL_PORT,
                              GPIO_PATH_FISH_I2C_SCL_PIN);
        HW_PathFish_RecoveryDelay();
    }

    /* Generate a STOP: SDA low, release SCL, then release SDA. */
    DL_GPIO_enableOutput(GPIO_PATH_FISH_I2C_SDA_PORT,
                         GPIO_PATH_FISH_I2C_SDA_PIN);
    HW_PathFish_RecoveryDelay();
    DL_GPIO_disableOutput(GPIO_PATH_FISH_I2C_SCL_PORT,
                          GPIO_PATH_FISH_I2C_SCL_PIN);
    HW_PathFish_RecoveryDelay();
    DL_GPIO_disableOutput(GPIO_PATH_FISH_I2C_SDA_PORT,
                          GPIO_PATH_FISH_I2C_SDA_PIN);
    HW_PathFish_RecoveryDelay();
    released =
        (DL_GPIO_readPins(GPIO_PATH_FISH_I2C_SDA_PORT,
                          PATH_FISH_BUS_LINES) & PATH_FISH_BUS_LINES) ==
        PATH_FISH_BUS_LINES;

    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_PATH_FISH_I2C_IOMUX_SDA, GPIO_PATH_FISH_I2C_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_PATH_FISH_I2C_IOMUX_SCL, GPIO_PATH_FISH_I2C_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_PATH_FISH_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_PATH_FISH_I2C_IOMUX_SCL);
    DL_I2C_resetControllerTransfer(PATH_FISH_I2C_INST);
    DL_I2C_flushControllerRXFIFO(PATH_FISH_I2C_INST);
    DL_I2C_clearInterruptStatus(PATH_FISH_I2C_INST, UINT32_MAX);
    DL_I2C_enableController(PATH_FISH_I2C_INST);
    s_rx_count = 0u;
    s_transfer_fault = PATH_FISH_TRANSFER_FAULT_NONE;
    s_transfer_state = PATH_FISH_TRANSFER_IDLE;
    NVIC_ClearPendingIRQ(PATH_FISH_I2C_INST_INT_IRQN);
    NVIC_EnableIRQ(PATH_FISH_I2C_INST_INT_IRQN);
    return released;
}

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
            s_transfer_fault = PATH_FISH_TRANSFER_FAULT_OVERFLOW;
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
    s_transfer_fault = PATH_FISH_TRANSFER_FAULT_NONE;
    s_transfer_state = PATH_FISH_TRANSFER_IDLE;
}

static void HW_PathFish_StartRead(uint32_t now)
{
    s_rx_count = 0u;
    s_transfer_started_at = now;
    s_last_request_at = now;
    s_transfer_fault = PATH_FISH_TRANSFER_FAULT_NONE;
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
    s_last_bus_recovery_at =
        (uint32_t)(0u - PATH_FISH_BUS_RECOVERY_PERIOD_MS);
    s_consecutive_transport_faults = 0u;
    s_transport_diag = (path_fish_transport_diag_t){0};
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
        s_transport_diag.transaction_count++;
        (void)Bsp_PathFish_AcceptBytes(
            &Path_Fish_Sensor, packet, BSP_PATH_FISH_PACKET_SIZE, now);
        s_transport_diag.packet_count = Path_Fish_Sensor.valid_packet_count;
        s_transport_diag.stream_resync_count =
            Path_Fish_Sensor.stream_resync_count;
        for (index = 0u; index < BSP_PATH_FISH_PACKET_SIZE; index++)
        {
            s_transport_diag.first_packet[index] =
                Path_Fish_Sensor.first_packet[index];
        }
        s_consecutive_transport_faults = 0u;
        s_transport_diag.status = PATH_FISH_TRANSPORT_OK;
        s_transfer_state = PATH_FISH_TRANSFER_IDLE;
        /* Match the vendor demo: wait 5 ms after STOP, not from START. */
        s_last_request_at = now;
    }
    else if (s_transfer_state == PATH_FISH_TRANSFER_ERROR)
    {
        switch (s_transfer_fault)
        {
        case PATH_FISH_TRANSFER_FAULT_NACK:
            s_transport_diag.nack_count++;
            s_transport_diag.status = PATH_FISH_TRANSPORT_NACK;
            break;
        case PATH_FISH_TRANSFER_FAULT_ARBITRATION_LOST:
            s_transport_diag.arbitration_lost_count++;
            s_transport_diag.status = PATH_FISH_TRANSPORT_ARBITRATION_LOST;
            break;
        case PATH_FISH_TRANSFER_FAULT_SHORT_READ:
            s_transport_diag.short_read_count++;
            s_transport_diag.status = PATH_FISH_TRANSPORT_SHORT_READ;
            break;
        case PATH_FISH_TRANSFER_FAULT_OVERFLOW:
            s_transport_diag.overflow_count++;
            s_transport_diag.status = PATH_FISH_TRANSPORT_OVERFLOW;
            break;
        default:
            break;
        }
        if (s_consecutive_transport_faults < UINT8_MAX)
        {
            s_consecutive_transport_faults++;
        }
        Bsp_PathFish_NotifyTransportFault(&Path_Fish_Sensor);
        HW_PathFish_ResetTransfer();
    }
    else if (s_transfer_state == PATH_FISH_TRANSFER_RX &&
             (uint32_t)(now - s_transfer_started_at) >=
                 PATH_FISH_TRANSFER_TIMEOUT_MS)
    {
        uint32_t status = DL_I2C_getControllerStatus(PATH_FISH_I2C_INST);
        uint32_t bus_monitor = PATH_FISH_I2C_INST->MASTER.MBMON &
                               PATH_FISH_BUS_LEVEL_MASK;
        bool bus_stuck = bus_monitor != PATH_FISH_BUS_LEVEL_MASK;

        s_transport_diag.timeout_count++;
        s_transport_diag.last_controller_status = status;
        s_transport_diag.last_bus_monitor = bus_monitor;
        s_transport_diag.status = PATH_FISH_TRANSPORT_TIMEOUT;
        if (s_consecutive_transport_faults < UINT8_MAX)
        {
            s_consecutive_transport_faults++;
        }
        if (bus_stuck)
        {
            s_transport_diag.bus_stuck_count++;
            s_transport_diag.status = PATH_FISH_TRANSPORT_BUS_STUCK;
        }
        Bsp_PathFish_NotifyTransportFault(&Path_Fish_Sensor);
        if (bus_stuck &&
            (uint32_t)(now - s_last_bus_recovery_at) >=
                PATH_FISH_BUS_RECOVERY_PERIOD_MS)
        {
            s_last_bus_recovery_at = now;
            s_last_request_at = now;
            s_transport_diag.recovery_attempt_count++;
            if (HW_PathFish_RecoverBus())
            {
                s_transport_diag.recovery_success_count++;
            }
        }
        else
        {
            HW_PathFish_ResetTransfer();
        }
    }

    uint32_t request_period =
        s_consecutive_transport_faults >= PATH_FISH_FAULT_BACKOFF_COUNT
            ? PATH_FISH_FAULT_RETRY_PERIOD_MS
            : PATH_FISH_REQUEST_PERIOD_MS;
    uint32_t controller_status =
        DL_I2C_getControllerStatus(PATH_FISH_I2C_INST);
    uint32_t bus_monitor = PATH_FISH_I2C_INST->MASTER.MBMON &
                           PATH_FISH_BUS_LEVEL_MASK;
    bool bus_free =
        (controller_status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0u &&
        (controller_status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0u &&
        bus_monitor == PATH_FISH_BUS_LEVEL_MASK;

    if (s_transfer_state == PATH_FISH_TRANSFER_IDLE &&
        (controller_status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0u &&
        bus_monitor != PATH_FISH_BUS_LEVEL_MASK &&
        (uint32_t)(now - s_last_bus_recovery_at) >=
            PATH_FISH_BUS_RECOVERY_PERIOD_MS)
    {
        s_transport_diag.last_controller_status = controller_status;
        s_transport_diag.last_bus_monitor = bus_monitor;
        s_transport_diag.bus_stuck_count++;
        s_transport_diag.status = PATH_FISH_TRANSPORT_BUS_STUCK;
        s_last_bus_recovery_at = now;
        s_last_request_at = now;
        s_transport_diag.recovery_attempt_count++;
        if (HW_PathFish_RecoverBus())
        {
            s_transport_diag.recovery_success_count++;
        }
        controller_status = DL_I2C_getControllerStatus(PATH_FISH_I2C_INST);
        bus_monitor = PATH_FISH_I2C_INST->MASTER.MBMON &
                      PATH_FISH_BUS_LEVEL_MASK;
        bus_free =
            (controller_status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0u &&
            (controller_status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0u &&
            bus_monitor == PATH_FISH_BUS_LEVEL_MASK;
    }

    if (s_transfer_state == PATH_FISH_TRANSFER_IDLE &&
        (uint32_t)(now - s_last_request_at) >= request_period &&
        bus_free)
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

void Path_Fish_GetTransportDiag(path_fish_transport_diag_t *diag)
{
    if (diag != NULL)
    {
        *diag = s_transport_diag;
        diag->current_controller_status =
            DL_I2C_getControllerStatus(PATH_FISH_I2C_INST);
        diag->current_bus_monitor = PATH_FISH_I2C_INST->MASTER.MBMON &
                                    PATH_FISH_BUS_LEVEL_MASK;
    }
}

const char *Path_Fish_TransportStatusText(void)
{
    switch (s_transport_diag.status)
    {
    case PATH_FISH_TRANSPORT_OK:               return "OK";
    case PATH_FISH_TRANSPORT_TIMEOUT:          return "TMO";
    case PATH_FISH_TRANSPORT_NACK:             return "NAK";
    case PATH_FISH_TRANSPORT_ARBITRATION_LOST: return "ARB";
    case PATH_FISH_TRANSPORT_SHORT_READ:       return "SHT";
    case PATH_FISH_TRANSPORT_OVERFLOW:         return "OVF";
    case PATH_FISH_TRANSPORT_BUS_STUCK:        return "BUS";
    default:                                   return "UNK";
    }
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
        if (s_transfer_state != PATH_FISH_TRANSFER_ERROR)
        {
            if (s_rx_count == BSP_PATH_FISH_PACKET_SIZE)
            {
                s_transfer_state = PATH_FISH_TRANSFER_COMPLETE;
            }
            else
            {
                s_transfer_fault = PATH_FISH_TRANSFER_FAULT_SHORT_READ;
                s_transfer_state = PATH_FISH_TRANSFER_ERROR;
            }
        }
        break;

    case DL_I2C_IIDX_CONTROLLER_NACK:
        s_transfer_fault = PATH_FISH_TRANSFER_FAULT_NACK;
        s_transfer_state = PATH_FISH_TRANSFER_ERROR;
        break;

    case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        s_transfer_fault = PATH_FISH_TRANSFER_FAULT_ARBITRATION_LOST;
        s_transfer_state = PATH_FISH_TRANSFER_ERROR;
        break;

    default:
        break;
    }
}
