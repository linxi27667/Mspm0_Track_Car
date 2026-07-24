/**
 * @file    main.c
 * @brief   MSPM0G3507 主程序（参考 f407zet6 架构：模块化初始化 + elog + 调试开关）
 * @note    架构:
 *            System_Init()  — RTT/SysConfig/SysTick/elog/外设模块统一初始化
 *            SelfTest 阶段  — 由 SELFTEST_MODE 宏控制，上电自检各设备
 *            Track 阶段     — 循迹主循环
 *          调试:
 *            SELFTEST_MODE=1 启动后跑一遍自检并打印诊断
 *            SELFTEST_MODE=0 跳过自检直接进入循迹模式
 */

/* ==================================================================
 * ====== 用户调参区 ======
 * 所有可调参数的默认值唯一定义在 APP/Inc/user_tuning.h。
 * 本文件不再重复定义，修改参数请编辑 user_tuning.h 后重新编译。
 *
 * 远程可调参数（线环、速度与电机环）的范围
 * 同时由 host/param_schema.py 驱动，tests/test_param_schema.py 验证一致性。
 * 最终比赛档值唯一落在 user_tuning.h，RACE 构建直接编译生效。
 * ================================================================== */

#include "ti_msp_dl_config.h"
#include <elog.h>
#include "SEGGER_RTT.h"
#include "app_log.h"
#include "bsp_uart.h"
#include "app_all.h"
#include "app_esp01s_cfg.h"
#include "track.h"
#include "elog_port.h"           /* elog ESP 透传后端控制 */
#include <stdio.h>
#include <string.h>

/* ================= 调试开关 ================= */
/* SELFTEST_MODE=1: 上电后自动跑一次自检（电机/编码器/MPU/鱼眼） */
/* SELFTEST_MODE=0: 跳过自检，直接进入循迹模式 */
/* ================= 全局时标 ================= */
extern volatile uint32_t systick_ms;  /* 定义在 bsp_common.c */

/* ================= 中断处理函数（全部集中于此） ================= */

void SysTick_Handler(void)
{
    systick_ms++;
}

/* ================= HardFault 诊断 ================= */
/* Cortex-M0+ 仅有 SHCSR, 没有 CFSR/HFSR/DFSR */
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "mov r0, sp\n"
        "ldr r1, =HardFault_Diag\n"
        "bx r1\n"
    );
}

void HardFault_Diag(uint32_t *sp)
{
    uint32_t r0  = sp[0];
    uint32_t r1  = sp[1];
    uint32_t r2  = sp[2];
    uint32_t r3  = sp[3];
    uint32_t r12 = sp[4];
    uint32_t lr  = sp[5];
    uint32_t pc  = sp[6];
    uint32_t xpsr = sp[7];

    SEGGER_RTT_Write(0, "*** HARDFAULT ***\r\n", 20);
    elog_e("FAULT", "PC=0x%08lX LR=0x%08lX XPSR=0x%08lX",
           (unsigned long)pc, (unsigned long)lr, (unsigned long)xpsr);
    elog_e("FAULT", "R0=0x%08lX R1=0x%08lX R2=0x%08lX R3=0x%08lX R12=0x%08lX",
           (unsigned long)r0, (unsigned long)r1, (unsigned long)r2,
           (unsigned long)r3, (unsigned long)r12);
    elog_e("FAULT", "SHCSR=0x%04lX", (unsigned long)(SCB->SHCSR & 0xFFFF));

    while (1) { /* 死循环等待调试器 */ }
}

/* ================= GPIO 输入回读使能 ================= */
/* SysConfig 的 DL_GPIO_initDigitalOutput 不启用 INE (bit18),
 * 导致 GPIO_ReadPin(读DIN) 始终返回0。此处对所有输出引脚 OR 上 INE。 */
static void Enable_GPIO_InputReadback(void)
{
    const uint32_t INE_BIT = IOMUX_PINCM_INENA_ENABLE;

    /* 编码器 PHA/PHB — 确保ISR中可原子读取两相信号。 */
    IOMUX->SECCFG.PINCM[Left_Encoder_LEFT_PHA_IOMUX] |= INE_BIT;
    IOMUX->SECCFG.PINCM[Left_Encoder_LEFT_PHB_IOMUX] |= INE_BIT;
    IOMUX->SECCFG.PINCM[Right_Encoder_PHA_IOMUX] |= INE_BIT;
    IOMUX->SECCFG.PINCM[Right_Encoder_PHB_IOMUX] |= INE_BIT;

    /* 板载状态LED与蜂鸣器控制引脚 */
    IOMUX->SECCFG.PINCM[Board_Output_STATUS_LED_IOMUX] |= INE_BIT;
    IOMUX->SECCFG.PINCM[Board_Output_BUZZER_IOMUX] |= INE_BIT;
}

/* ================= 外设中断 ISR ================= */

extern uint8_t s_rx_dma_buf[];
extern uint8_t g_esp_rx_dma_buf[];   /* ESP-01S UART1 DMA RX 暂存（app_esp01s.c 定义） */

void UART0_IRQHandler(void)
{
    uint32_t irq = DL_UART_getPendingInterrupt(UART0);
    switch (irq) {
        case DL_UART_IIDX_RX_TIMEOUT_ERROR: {
            DL_DMA_disableChannel(DMA, DMA_CH0_RX);
            uint16_t rx_len = BSP_UART_RX_BUF_SIZE -
                              DL_DMA_getTransferSize(DMA, DMA_CH0_RX);
            while (!DL_UART_isRXFIFOEmpty(UART0)) {
                if (rx_len < BSP_UART_RX_BUF_SIZE) {
                    s_rx_dma_buf[rx_len++] = (uint8_t)DL_UART_receiveData(UART0);
                } else {
                    (void)DL_UART_receiveData(UART0);
                }
            }
            Uart_RX_IRQHandler(rx_len);
            break;
        }
        default: break;
    }
}

/* ESP-01S UART1 RX 超时中断 — 与 UART0 同模式，仅替换外设实例与 DMA 通道 */
void UART1_IRQHandler(void)
{
    uint32_t irq = DL_UART_getPendingInterrupt(UART_ESP01S_INST);
    switch (irq) {
        case DL_UART_IIDX_RX_TIMEOUT_ERROR: {
            DL_DMA_disableChannel(DMA, ESP01S_DMA_CH_RX);
            uint16_t rx_len = ESP01S_DMA_RX_BUF_SIZE -
                              DL_DMA_getTransferSize(DMA, ESP01S_DMA_CH_RX);
            while (!DL_UART_isRXFIFOEmpty(UART_ESP01S_INST)) {
                if (rx_len < ESP01S_DMA_RX_BUF_SIZE) {
                    g_esp_rx_dma_buf[rx_len++] = (uint8_t)DL_UART_receiveData(UART_ESP01S_INST);
                } else {
                    (void)DL_UART_receiveData(UART_ESP01S_INST);
                }
            }
            App_ESP01S_RX_IRQHandler(rx_len);
            break;
        }
        default: break;
    }
}

void DMA_IRQHandler(void)
{
    switch (DL_DMA_getPendingInterrupt(DMA)) {
        case DL_DMA_EVENT_IIDX_DMACH1:
            Uart_Tx_DMA_Done_IRQHandler();
            break;
        case DL_DMA_EVENT_IIDX_DMACH2:
            App_ESP01S_RX_DMA_Done_IRQHandler();
            break;
        case DL_DMA_EVENT_IIDX_DMACH3:
            App_ESP01S_Tx_DMA_Done_IRQHandler();
            break;
        default: break;
    }
}

/* GPIO GROUP1中断 — 左右编码器A/B两相双边沿触发。 */
void GROUP1_IRQHandler(void)
{
    const uint32_t encoder_pins = LEFT_ENC_PHA_PIN | LEFT_ENC_PHB_PIN |
                                  RIGHT_ENC_PHA_PIN | RIGHT_ENC_PHB_PIN;
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOB, encoder_pins);
    if (pending != 0u) {
        DL_GPIO_clearInterruptStatus(GPIOB, pending);
        Encoder_GPIO_IRQHandler(pending);
    }
}

/* =================================================================
 * 系统初始化 — 参考 f407zet6 main.c 的初始化顺序
 * ================================================================= */

/* ESP-01S 建连进度回调（RTT 打印） */
static void on_esp_progress(uint8_t step, const char *msg)
{
    APP_ESP_LOG_I("esp", "step %d: %s", step, msg);
}

#if APP_UART1_DIAG_CONSOLE_ENABLE
static void UART1_Diag_WriteByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_ESP01S_INST)) {}
    DL_UART_Main_transmitData(UART_ESP01S_INST, data);
}

static void UART1_Diag_WriteString(const char *s)
{
    if (s == NULL) return;
    while (*s != '\0') {
        if (*s == '\n') {
            UART1_Diag_WriteByte('\r');
        }
        UART1_Diag_WriteByte((uint8_t)*s++);
    }
}

static void UART1_Diag_PollEcho(void)
{
    static char rx_line[64];
    static uint8_t rx_len = 0;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_ESP01S_INST)) {
        char c = (char)DL_UART_Main_receiveData(UART_ESP01S_INST);
        if (c == '\r' || c == '\n' || rx_len >= (sizeof(rx_line) - 1u)) {
            if (rx_len > 0u) {
                rx_line[rx_len] = '\0';
                UART1_Diag_WriteString("\n[PA9 RX] ");
                UART1_Diag_WriteString(rx_line);
                UART1_Diag_WriteString("\n");
                rx_len = 0;
            }
        } else {
            rx_line[rx_len++] = c;
        }
    }
}
#endif

static void Log_Build_Config(void)
{
    elog_i("CONFIG", "ESP01S=%s", APP_ESP01S_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "TUNING=%s", APP_TUNING_MODE ? "ON" : "OFF");
    elog_i("CONFIG", "ROTATE=%s", APP_TUNING_ROTATE_LINK_TEST_MODE ? "LINK_TEST" : "REAL");
    elog_i("CONFIG", "SELFTEST=%s", SELFTEST_MODE ? "ON_MOTOR_TEST" : "OFF");
    elog_i("CONFIG", "LOG_ESP=%s", APP_LOG_ESP01S_ENABLE ? "ON" : "OFF_LINK_ACTIVE");
    elog_i("CONFIG", "LOG_GRAY_ADC=%s", APP_LOG_GRAY_ADC_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "LOG_GRAY_STATE=%s", APP_LOG_GRAY_STATE_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "LOG_ROTATE=%s", APP_LOG_ROTATE_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "LOG_VEHICLE=%s", APP_LOG_VEHICLE_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "LOG_TRACK=%s", APP_LOG_TRACK_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "LOG_TUNING=%s", APP_LOG_TUNING_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "LOG_IMU=%s", APP_LOG_IMU_ENABLE ? "ON" : "OFF");
    elog_i("CONFIG", "LOG_DIAG=%s", APP_LOG_DIAGNOSTIC_ENABLE ? "ON" : "OFF");
}

static void System_Init(void)
{
    /* 1. RTT 优先（最早的日志通道） */
    SEGGER_RTT_Init();

    /* 2. SysConfig 生成的外设初始化 */
    SYSCFG_DL_init();

    /* 3. SysTick 1ms */
    SysTick_Config(CPUCLK_FREQ / 1000);

    /* 4. elog 日志框架 */
    elog_init();
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_ERROR,  ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN,   ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO,   ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG,  ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME | ELOG_FMT_LINE);
    elog_start();
    Log_Build_Config();

    /* 5. 启用 GPIO 输入回读（诊断用） */
    Enable_GPIO_InputReadback();

    /* 6. 应用层模块初始化（顺序参考 f407zet6: Motor/Safety/Key/Encoder） */
    App_Led_Init();
    elog_i("LED", "GPIO configured: channels=%u", (unsigned)BSP_LED_NUM);
    App_Buzzer_Init();
    elog_i("BUZZER", "GPIO configured: PB5 active-low, non-blocking pattern");
    App_Key_Init();
    elog_i("KEY", "GPIO configured: channels=%u", (unsigned)BSP_KEY_NUM);
    App_Uart_Init();
    elog_i("UART", "Debug UART configured: 115200 8N1");

#if APP_ESP01S_ENABLE
    /* 7. ESP-01S 非阻塞初始化（仅初始化 HW + 启动 BOOT 状态机）
     *    WiFi/TCP 建连由主循环 App_ESP01S_Poll() 逐步推进，不阻塞主流程
     *    现场需先改 app_esp01s_cfg.h 的 WiFi SSID/PASS 和服务器 IP/PORT */
    (void)App_ESP01S_Init(on_esp_progress);
    elog_port_esp_enable(true);   /* 启用 elog → ESP 透传（未连线时日志自动丢弃） */
    App_Tuning_Init();   /* 注册 JSON 接收回调 */
#else
    DL_UART_disableInterrupt(UART_ESP01S_INST,
                             DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_UART_disableDMAReceiveEvent(UART_ESP01S_INST, DL_UART_DMA_INTERRUPT_RX);
    DL_UART_disableDMATransmitEvent(UART_ESP01S_INST);
    elog_port_esp_enable(false);
    elog_i("SYS", "ESP-01S disabled; UART1 PA8/PA9 diagnostic console at 115200 8N1");
#if APP_UART1_DIAG_CONSOLE_ENABLE
    UART1_Diag_WriteString("\n[BOOT] UART1 PA8=TX PA9=RX diagnostic console 115200 8N1\n");
#endif
#endif

    elog_i("SYS", "System initialized - CPU=%luHz", (unsigned long)CPUCLK_FREQ);
}

/* =================================================================
 * 自检模块（SELFTEST_MODE=1 时执行）
 * ================================================================= */
#if SELFTEST_MODE && APP_LOG_DIAGNOSTIC_ENABLE

static void Diag_PINCM(void)
{
    elog_i("IOMUX", "=== PINCM Dump (expect 0x00040081 with INE) ===");
    elog_i("IOMUX", "PA0(PATH_SDA)=0x%08lX PA1(PATH_SCL)=0x%08lX",
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_PATH_FISH_I2C_IOMUX_SDA],
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_PATH_FISH_I2C_IOMUX_SCL]);
    elog_i("IOMUX", "PA18(OLED_SDA)=0x%08lX PA17(OLED_SCL)=0x%08lX",
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_OLED_I2C_IOMUX_SDA],
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_OLED_I2C_IOMUX_SCL]);
    elog_i("IOMUX", "PA22(L_AIN1_PWM, TIMA0_C1)=0x%08lX",
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_Motor_Left_Timer_C1_IOMUX]);
    elog_i("IOMUX", "PA15(L_AIN2_PWM, TIMA0_C2)=0x%08lX",
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_Motor_Left_Timer_C2_IOMUX]);
    elog_i("IOMUX", "PB0(R_BIN1_PWM, TIMA1_C0)=0x%08lX",
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_Motor_Right_BIN1_Timer_C0_IOMUX]);
    elog_i("IOMUX", "PB11(R_BIN2_PWM, TIMG6_C1)=0x%08lX",
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_Motor_Right_BIN2_Timer_C1_IOMUX]);
    elog_i("IOMUX", "PB4(STATUS_LED, PINCM17)=0x%08lX [ref]",
           (unsigned long)IOMUX->SECCFG.PINCM[Board_Output_STATUS_LED_IOMUX]);
    elog_i("IOMUX", "PB5(BUZZER, PINCM18)=0x%08lX [ref]",
           (unsigned long)IOMUX->SECCFG.PINCM[Board_Output_BUZZER_IOMUX]);

    /* GPIO 寄存器 */
    elog_i("GPIO", "A: DOE=0x%08lX DOUT=0x%08lX DIN=0x%08lX",
           (unsigned long)GPIOA->DOE31_0, (unsigned long)GPIOA->DOUT31_0,
           (unsigned long)GPIOA->DIN31_0);
    elog_i("GPIO", "B: DOE=0x%08lX DOUT=0x%08lX DIN=0x%08lX",
           (unsigned long)GPIOB->DOE31_0, (unsigned long)GPIOB->DOUT31_0,
           (unsigned long)GPIOB->DIN31_0);

}

static void SelfTest_ICM42688(void)
{
    elog_i("TEST", "==== ICM42688-P Test ====");
    (void)App_Imu_Init();
    Bsp_Delay_ms(100);

    if (Imu_Sensor.initialized && Imu_Read_Data()) {
        elog_i("TEST", "ICM OK: ax=%.2f ay=%.2f az=%.2f",
               Imu_Sensor.accel_x, Imu_Sensor.accel_y, Imu_Sensor.accel_z);
        elog_i("TEST", "ICM OK: gx=%.2f gy=%.2f gz=%.2f",
               Imu_Sensor.gyro_x, Imu_Sensor.gyro_y, Imu_Sensor.gyro_z);
        elog_i("TEST", "ICM OK: temp=%.1fC", Imu_Sensor.temp);
    } else {
        elog_e("TEST", "ICM FAIL: init/read error");
    }
    elog_i("TEST", "==== ICM42688-P Done ====");
}

static void SelfTest_Motor_Encoder(void)
{
    elog_i("TEST", "==== Motor+Encoder Test ====");
    App_Motor_Init();
    Bsp_Delay_ms(50);

    /* 引脚说明 */
    elog_i("MOT", "Left AT8236: AIN1=TIMA0_C1(PA22) AIN2=TIMA0_C2(PA15) Enc=GPIO4x(PB21/PB22)");
    elog_i("MOT", "Right AT8236: BIN1=TIMA1_C0(PB0) BIN2=TIMG6_C1(PB11) Enc=GPIO4x(PB12/PB13)");

    /* PWM 定时器运行状态 + 寄存器诊断 */
    uint32_t left_1 = DL_Timer_getTimerCount(MOTOR_LEFT_AIN1_PWM_TIMER);
    uint32_t right_in1_1 = DL_Timer_getTimerCount(MOTOR_RIGHT_BIN1_PWM_TIMER);
    uint32_t right_in2_1 = DL_Timer_getTimerCount(MOTOR_RIGHT_BIN2_PWM_TIMER);
    Bsp_Delay_ms(10);
    uint32_t left_2 = DL_Timer_getTimerCount(MOTOR_LEFT_AIN1_PWM_TIMER);
    uint32_t right_in1_2 = DL_Timer_getTimerCount(MOTOR_RIGHT_BIN1_PWM_TIMER);
    uint32_t right_in2_2 = DL_Timer_getTimerCount(MOTOR_RIGHT_BIN2_PWM_TIMER);
    elog_i("MOT", "TIMA0 cnt: %lu->%lu (%s) LOAD=%lu",
           (unsigned long)left_1, (unsigned long)left_2,
           (left_2 != left_1) ? "RUNNING" : "STOPPED",
           (unsigned long)DL_Timer_getLoadValue(MOTOR_LEFT_AIN1_PWM_TIMER));
    elog_i("MOT", "TIMA1 cnt: %lu->%lu (%s) LOAD=%lu",
           (unsigned long)right_in1_1, (unsigned long)right_in1_2,
           (right_in1_2 != right_in1_1) ? "RUNNING" : "STOPPED",
           (unsigned long)DL_Timer_getLoadValue(MOTOR_RIGHT_BIN1_PWM_TIMER));
    elog_i("MOT", "TIMG6 cnt: %lu->%lu (%s) LOAD=%lu",
           (unsigned long)right_in2_1, (unsigned long)right_in2_2,
           (right_in2_2 != right_in2_1) ? "RUNNING" : "STOPPED",
           (unsigned long)DL_Timer_getLoadValue(MOTOR_RIGHT_BIN2_PWM_TIMER));

    /* 同步编码器基准 (调用 Update_Status 让 last_count = 当前值, 首次返回0) */
    Bsp_Motor_Update_Status(&Motor_Left);
    Bsp_Motor_Update_Status(&Motor_Right);
    Motor_Left.encoder_speed  = 0;
    Motor_Right.encoder_speed = 0;

    /* ---- 正转测试: PWM=50, 每200ms采样一次, 共2秒 ----
     * 左右编码器均读取GPIO 4x累计计数的差值。 */
    elog_i("TEST", "Forward PWM=50 for 2s (sampled every 200ms)...");
    Bsp_Motor_Set_Speed(&Motor_Left, 50);
    Bsp_Motor_Set_Speed(&Motor_Right, 50);

    uint32_t left_in1_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_LEFT_AIN1_PWM_TIMER, MOTOR_LEFT_AIN1_PWM_CHANNEL);
    uint32_t left_in2_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_LEFT_AIN2_PWM_TIMER, MOTOR_LEFT_AIN2_PWM_CHANNEL);
    uint32_t right_in1_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_RIGHT_BIN1_PWM_TIMER, MOTOR_RIGHT_BIN1_PWM_CHANNEL);
    uint32_t right_in2_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_RIGHT_BIN2_PWM_TIMER, MOTOR_RIGHT_BIN2_PWM_CHANNEL);
    elog_i("MOT", "Fwd CC: L=[%lu,%lu] R=[%lu,%lu] (expect [50,100])",
           (unsigned long)left_in1_cc, (unsigned long)left_in2_cc,
           (unsigned long)right_in1_cc, (unsigned long)right_in2_cc);

    int32_t left_fwd = 0, right_fwd = 0;
    for (int i = 0; i < 10; i++) {
        Bsp_Delay_ms(200);
        Bsp_Motor_Update_Status(&Motor_Left);
        Bsp_Motor_Update_Status(&Motor_Right);
        left_fwd  += Motor_Left.encoder_speed;
        right_fwd += Motor_Right.encoder_speed;
    }
    Bsp_Motor_Set_Speed(&Motor_Left, 0);
    Bsp_Motor_Set_Speed(&Motor_Right, 0);
    Bsp_Delay_ms(300);

    elog_i("TEST", "Forward: LeftEnc=%ld RightEnc=%ld", (long)left_fwd, (long)right_fwd);

    if (left_fwd > 10 && right_fwd > 10) {
        elog_i("TEST", "Motor+Enc OK: both wheels rotated forward");
    } else if (left_fwd > 10 || right_fwd > 10) {
        elog_w("TEST", "Partial: L=%ld R=%ld (check wiring)", (long)left_fwd, (long)right_fwd);
    } else {
        elog_e("TEST", "Both FAIL - check: 1)AT8236 VM/GND 2)Motor power supply 3)Wiring");
    }

    /* ---- 反转测试: PWM=-50, 每200ms采样一次, 共2秒 ---- */
    elog_i("TEST", "Backward PWM=-50 for 2s (sampled every 200ms)...");
    Bsp_Motor_Set_Speed(&Motor_Left, -50);
    Bsp_Motor_Set_Speed(&Motor_Right, -50);

    left_in1_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_LEFT_AIN1_PWM_TIMER, MOTOR_LEFT_AIN1_PWM_CHANNEL);
    left_in2_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_LEFT_AIN2_PWM_TIMER, MOTOR_LEFT_AIN2_PWM_CHANNEL);
    right_in1_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_RIGHT_BIN1_PWM_TIMER, MOTOR_RIGHT_BIN1_PWM_CHANNEL);
    right_in2_cc = DL_Timer_getCaptureCompareValue(
        MOTOR_RIGHT_BIN2_PWM_TIMER, MOTOR_RIGHT_BIN2_PWM_CHANNEL);
    elog_i("MOT", "Bwd CC: L=[%lu,%lu] R=[%lu,%lu] (expect [100,50])",
           (unsigned long)left_in1_cc, (unsigned long)left_in2_cc,
           (unsigned long)right_in1_cc, (unsigned long)right_in2_cc);

    int32_t left_bwd = 0, right_bwd = 0;
    for (int i = 0; i < 10; i++) {
        Bsp_Delay_ms(200);
        Bsp_Motor_Update_Status(&Motor_Left);
        Bsp_Motor_Update_Status(&Motor_Right);
        left_bwd  += Motor_Left.encoder_speed;
        right_bwd += Motor_Right.encoder_speed;
    }
    Bsp_Motor_Set_Speed(&Motor_Left, 0);
    Bsp_Motor_Set_Speed(&Motor_Right, 0);

    elog_i("TEST", "Backward: LeftEnc=%ld RightEnc=%ld", (long)left_bwd, (long)right_bwd);

    /* 方向验证: 正转应>0, 反转应<0 */
    if (left_fwd > 10 && left_bwd < -10) {
        elog_i("TEST", "Left enc direction OK (fwd=%ld bwd=%ld)", (long)left_fwd, (long)left_bwd);
    } else {
        elog_w("TEST", "Left enc direction ISSUE (fwd=%ld bwd=%ld)", (long)left_fwd, (long)left_bwd);
    }
    if (right_fwd > 10 && right_bwd < -10) {
        elog_i("TEST", "Right enc direction OK (fwd=%ld bwd=%ld)", (long)right_fwd, (long)right_bwd);
    } else {
        elog_w("TEST", "Right enc direction ISSUE (fwd=%ld bwd=%ld)", (long)right_fwd, (long)right_bwd);
    }
    elog_i("TEST", "==== Motor+Encoder Done ====");
}

static void SelfTest_PathFish(void)
{
    uint32_t started_at;

    elog_i("TEST", "==== Path Fish I2C0 Test ====");
    App_PathFish_Init();
    started_at = Bsp_GetTick();
    while ((uint32_t)(Bsp_GetTick() - started_at) < 100u) {
        Path_Fish_Poll(Bsp_GetTick());
    }
    elog_i("TEST", "Fish seq=%lu mask=0x%03X BC=%u age=%lu err=%lu P=%u",
           (unsigned long)Path_Fish_Sensor.frame_seq,
           Path_Fish_Sensor.mask, Path_Fish_Sensor.black_count,
           (unsigned long)Path_Fish_FrameAge(Bsp_GetTick()),
           (unsigned long)Path_Fish_Sensor.error_count,
           Path_Fish_Sensor.black_is_one ? 1u : 0u);
    elog_i("TEST", "==== Path Fish Done ====");
}

static void SelfTest_KeyLed(void)
{
    elog_i("TEST", "==== Key+LED Test ====");
    for (uint8_t i = 0; i < BSP_LED_NUM; i++) {
        Led_On(i);
        Bsp_Delay_ms(200);
        Led_Off(i);
    }
    elog_i("TEST", "==== Key+LED Done ====");
}

/* 自检主入口 */
static void Run_SelfTest(void)
{
    elog_i("SYS", "########## SelfTest Start ##########");

    Diag_PINCM();
    Bsp_Delay_ms(100);

    SelfTest_ICM42688();
    Bsp_Delay_ms(200);

    SelfTest_PathFish();
    Bsp_Delay_ms(200);

    SelfTest_KeyLed();
    Bsp_Delay_ms(200);

    SelfTest_Motor_Encoder();
    Bsp_Delay_ms(200);

    elog_i("SYS", "########## SelfTest Complete ##########");
}

#endif /* SELFTEST_MODE && APP_LOG_DIAGNOSTIC_ENABLE */

/* =================================================================
 * 主函数
 * ================================================================= */
typedef struct {
    uint32_t last_5ms;
    uint32_t last_10ms;
    uint32_t last_100ms;
    uint32_t last_500ms;
    uint32_t last_1000ms;
    uint8_t last_state;
    uint8_t last_circle;
    uint8_t last_black_count;
} main_runtime_t;

static uint8_t s_oled_page;
static bool s_oled_failure_logged;

static bool Main_PeriodElapsed(uint32_t now, uint32_t *last, uint32_t period_ms)
{
    if ((uint32_t)(now - *last) < period_ms) return false;
    *last = now;
    return true;
}

static bool Main_StationaryReady(void)
{
    safety_state_t state = Safety_GetState();
    return state == SAFETY_STATE_READY &&
           (Track.state == TRACK_STATE_IDLE ||
            Track.state == TRACK_STATE_FINISHED);
}

static void Main_InitializeTracking(void)
{
    Safety_Init();
    Track_Init();
    elog_i("MOTOR", "PWM and encoders configured: dual outputs disabled");
    Track_Set_Circles(USER_TRACK_CIRCLES);
    Track_Set_Speed(USER_TRACK_BASE_SPEED);

    elog_i("SYS", "Waiting for 2 centered Path Fish frames before READY");
    elog_i("SYS", "K1=Start K2=Stop K3=OLED(K3Long=ScaleCal) K4=Polarity");
}

static void Main_UpdatePathFish(uint32_t now)
{
    static uint32_t last_notified_seq;

    Path_Fish_Poll(now);
    if (Path_Fish_Sensor.frame_fault && !Safety_AllowsMotorOutput()) {
        (void)Path_Fish_TakeFault();
        Safety_ResetPathFishReadiness();
    }

    if (Path_Fish_Sensor.frame_seq != last_notified_seq) {
        track_line_observation_t line = Track_ClassifyLine12(
            Path_Fish_Sensor.mask, 0.0f,
            (uint8_t)(Track.right_angle_threshold + 0.5f));
        bool centered_normal =
            line.type == TRACK_LINE_NORMAL &&
            Track_Abs_Float(line.error) <= 0.75f;

        last_notified_seq = Path_Fish_Sensor.frame_seq;
        Safety_NotifyPathFishFrame(Path_Fish_Sensor.last_frame_tick,
                                   centered_normal);
    }
}

static void Main_RunControl(uint32_t now, main_runtime_t *runtime)
{
    if (!Main_PeriodElapsed(now, &runtime->last_5ms, 5u)) return;

    /* IMU恢复仅在Safety已禁止运动时执行；成功后自动重新静止校准。 */
    App_Imu_Recovery_Update(now, Safety_AllowsMotorOutput());

    /* 静止校准独立于电机授权，每个 5 ms 节拍只采一个样本。 */
    if (Imu_Sensor.calibration_state == IMU_CAL_RUNNING) {
        (void)Imu_Calibration_Update();
    }

    if (Safety_ControlTick()) {
        Track_Update();
        App_Motor_Speed_PID_Update();
    } else if (Safety_GetState() == SAFETY_STATE_FAULT) {
        const char *reason = Safety_FaultName(Safety_GetFault());
        if (Track_Is_Motor_Test()) {
            Track_Abort_Motor_Test(reason);
        } else if (Track_Is_Rotating()) {
            Track_Abort_Rotation(reason);
        } else if (Track_Is_Heading_Hold()) {
            (void)Track_Stop_Heading_Hold();
        } else {
            Safety_ForceMotorStop();
            Track.state = TRACK_STATE_FINISHED;
            Track.left_speed = 0;
            Track.right_speed = 0;
            Track.target_speed = 0;
            Track.turn_output = 0.0f;
        }
    }
}

static void Main_LogVehicleStatus(main_runtime_t *runtime)
{
    uint8_t state = (uint8_t)Track.state;
    uint8_t circle = (uint8_t)Track.current_circle;
    uint8_t black_count = Path_Fish_Sensor.black_count;
    if (!Track_State_Allows_Motor(Track.state)) return;

    APP_VEHICLE_LOG_I("CAR", "S=%d C=%d E=%ld G=%ld LT=%d RT=%d LP=%ld RP=%ld LE=%ld RE=%ld LX=%ld RX=%ld BC=%d",
                      Track.state, Track.current_circle,
                      (long)(Track.error * 1000), (long)Track.gyro_rate,
                      Track.left_speed, Track.right_speed,
                      (long)Motor_Left.current_pwm, (long)Motor_Right.current_pwm,
                      (long)Motor_Left.encoder_speed, (long)Motor_Right.encoder_speed,
                      (long)Motor_Left.total_position, (long)Motor_Right.total_position,
                      black_count);
    if (Track.state == TRACK_STATE_ROTATE) {
        APP_VEHICLE_LOG_I("ROTCTL", "actual=%ld error=%ld rate=%ld output=%ld",
                          (long)(Track.rotate_actual_deg * 10.0f),
                          (long)(Track.rotate_error_deg * 10.0f),
                          (long)(Imu_Sensor.gyro_z * 10.0f),
                          (long)(Track.turn_output * 10.0f));
    } else if (Track.state == TRACK_STATE_HEADING_HOLD) {
        APP_VEHICLE_LOG_I("HOLDCTL", "target=%ld actual=%ld error=%ld rate=%ld output=%ld",
                          (long)(Track.heading_hold_target * 10.0f),
                          (long)(Track.heading_hold_actual * 10.0f),
                          (long)(Track.heading_hold_error * 10.0f),
                          (long)(Imu_Sensor.gyro_z * 10.0f),
                          (long)(Track.turn_output * 10.0f));
    }

    runtime->last_state = state;
    runtime->last_circle = circle;
    runtime->last_black_count = black_count;
}

static void Main_LogPathFish(void)
{
    if (!Track_State_Is_LineTracking(Track.state)) return;

    APP_GRAY_ADC_LOG_I("FISH",
        "A=%u%u%u%u%u%u%u%u%u%u%u%u mask=0x%03X BC=%u",
        Path_Fish_Sensor.digital[0], Path_Fish_Sensor.digital[1],
        Path_Fish_Sensor.digital[2], Path_Fish_Sensor.digital[3],
        Path_Fish_Sensor.digital[4], Path_Fish_Sensor.digital[5],
        Path_Fish_Sensor.digital[6], Path_Fish_Sensor.digital[7],
        Path_Fish_Sensor.digital[8], Path_Fish_Sensor.digital[9],
        Path_Fish_Sensor.digital[10], Path_Fish_Sensor.digital[11],
        Path_Fish_Sensor.mask, Path_Fish_Sensor.black_count);
    APP_GRAY_STATE_LOG_I("FISH", "seq=%lu age=%lu err=%lu polarity=%u",
        (unsigned long)Path_Fish_Sensor.frame_seq,
        (unsigned long)Path_Fish_FrameAge(Bsp_GetTick()),
        (unsigned long)Path_Fish_Sensor.error_count,
        Path_Fish_Sensor.black_is_one ? 1u : 0u);
}

static void Main_UpdateOled(uint32_t now)
{
    char line[17];

    if (s_oled_page == 0u) {
        (void)snprintf(line, sizeof(line), "M:%03X BC:%02u P:%u",
                       Path_Fish_Sensor.mask,
                       Path_Fish_Sensor.black_count,
                       Path_Fish_Sensor.black_is_one ? 1u : 0u);
        App_Oled_SetLine(1u, line);

        line[0] = 'A';
        line[1] = ':';
        for (uint8_t i = 0u; i < BSP_PATH_FISH_CHANNEL_COUNT; i++) {
            line[i + 2u] = Path_Fish_Sensor.digital[i] ? '1' : '0';
        }
        line[14] = '\0';
        App_Oled_SetLine(2u, line);

        (void)snprintf(line, sizeof(line), "AGE:%03lu ERR:%04lu",
                       (unsigned long)(
                           Path_Fish_FrameAge(now) > 999u
                               ? 999u : Path_Fish_FrameAge(now)),
                       (unsigned long)(Path_Fish_Sensor.error_count % 10000u));
        App_Oled_SetLine(3u, line);

        (void)snprintf(line, sizeof(line), "E:%+05d READY:%u",
                       (int)(Track.error * 100.0f),
                       Safety_PathFishReady() ? 1u : 0u);
        App_Oled_SetLine(4u, line);
    } else {
        (void)snprintf(line, sizeof(line), "S:%u E:%+05d",
                       (unsigned)Track.state,
                       (int)(Track.error * 100.0f));
        App_Oled_SetLine(1u, line);
        (void)snprintf(line, sizeof(line), "L:%+04ld R:%+04ld",
                       (long)Track.left_speed, (long)Track.right_speed);
        App_Oled_SetLine(2u, line);
        (void)snprintf(line, sizeof(line), "C:%u RA:%u BC:%u",
                       Track.current_circle, Track.right_angle_count,
                       Path_Fish_Sensor.black_count);
        App_Oled_SetLine(3u, line);
        (void)snprintf(line, sizeof(line), "KP:%d KD:%d",
                       (int)(Track.line_pid.kp * 10.0f),
                       (int)(Track.line_pid.kd * 10.0f));
        App_Oled_SetLine(4u, line);
    }
}

static bool Main_PolarityAllowed(void)
{
    safety_state_t state = Safety_GetState();
    return (state == SAFETY_STATE_BOOT || state == SAFETY_STATE_READY) &&
           (Track.state == TRACK_STATE_IDLE ||
            Track.state == TRACK_STATE_FINISHED);
}

static void Main_HandleKeyPush(uint8_t key)
{
    if (key == 0u) {
        if (Safety_GetState() == SAFETY_STATE_READY) {
            if (Track_Start()) {
                elog_i("CTRL", "Tracking started");
            } else {
                elog_w("CTRL", "Start blocked: %s", Track_Get_Start_Result());
            }
        } else {
            elog_w("CTRL", "Start blocked (safety=%s)", Safety_StateName(Safety_GetState()));
        }
    } else if (key == 1u) {
        Track_Stop();
        elog_i("CTRL", "Tracking stopped");
    } else if (key == 2u) {
        s_oled_page = (uint8_t)((s_oled_page + 1u) % 2u);
        Main_UpdateOled(Bsp_GetTick());
        elog_i("OLED", "Page=%u", s_oled_page);
    } else if (key == 3u && Main_PolarityAllowed()) {
        Path_Fish_ToggleBlackPolarity();
        Safety_ResetPathFishReadiness();
        Main_UpdateOled(Bsp_GetTick());
        elog_i("SENSOR", "Path Fish black polarity=%u",
               Path_Fish_Sensor.black_is_one ? 1u : 0u);
    } else if (key == 3u) {
        elog_w("SENSOR", "Polarity change blocked while vehicle active");
    }
}

static bool Main_KeyActionAllowed(uint8_t key, bsp_key_event_t event)
{
    if (event == BSP_KEY_EVENT_PUSH && key == 3u) return Main_PolarityAllowed();

    return event == BSP_KEY_EVENT_LONGPUSH && key == 2u &&
           Main_StationaryReady();
}

static void Main_ProcessKeys(void)
{
    for (uint8_t key = 0; key < BSP_KEY_NUM; key++) {
        bsp_key_event_t event = Key_GetEvent(key);
        bool action_feedback;

        if (event == BSP_KEY_EVENT_IDLE) continue;

        action_feedback = Main_KeyActionAllowed(key, event);
        Led_Toggle(0u);

        if (event == BSP_KEY_EVENT_PUSH) {
            elog_i("KEY", "K%d pressed", (int)key);
            Main_HandleKeyPush(key);
        } else if (event == BSP_KEY_EVENT_DOUBLECLICK) {
            elog_i("KEY", "K%d double-click", (int)key);
            if (key == 0u && Safety_GetState() == SAFETY_STATE_READY) {
                uint8_t circles = (uint8_t)(Track.target_circles + 1u);
                if (circles > 5u) circles = 1u;
                Track_Set_Circles(circles);
                elog_i("CTRL", "Circles set to %d", circles);
            }
        } else if (event == BSP_KEY_EVENT_LONGPUSH) {
            elog_i("KEY", "K%d long-pressed", (int)key);
            if (key == 2u) {
                if (action_feedback && APP_TUNING_MODE) {
                    Buzzer_Stop();
                    App_Motor_Calibrate_Scale();
                } else {
                    elog_w("CALIB", "Scale calibration blocked: requires TUNING, READY and idle wheels-up bench");
                }
            }
        }

        Buzzer_Request(action_feedback ? 2u : 1u, systick_ms);
    }
}

static void Main_ProcessDebugUart(void)
{
    uint8_t frame[BSP_UART_RX_BUF_SIZE + 1];
    uint16_t len = 0;
    while (Uart_Get_Frame(frame, &len)) {
        if (len >= BSP_UART_RX_BUF_SIZE) len = BSP_UART_RX_BUF_SIZE - 1u;
        frame[len] = '\0';
        elog_i("COMM", "RX(%u): %s", (unsigned)len, (char *)frame);
    }
}

static void Main_PollCommunication(void)
{
#if APP_ESP01S_ENABLE
    App_ESP01S_Poll();
    App_Tuning_Poll();
    elog_port_esp_flush();
#elif APP_UART1_DIAG_CONSOLE_ENABLE
    UART1_Diag_PollEcho();
#endif
}

static void Main_RunLoop(void)
{
    main_runtime_t runtime = {
        .last_state = 0xFFu,
        .last_circle = 0xFFu,
        .last_black_count = 0xFFu,
    };

    while (1) {
        uint32_t now = systick_ms;

        /* 高频连续任务：只推进蜂鸣器节奏状态，不阻塞循迹控制。 */
        Buzzer_Update(now);

        /* 高频连续任务：推进鱼眼I2C0非阻塞读取，完整帧才通知Safety。 */
        Main_UpdatePathFish(now);

        /* 高频连续任务：每圈进入主控制；内部按 5 ms 非阻塞更新循迹与速度 PID。 */
        Main_RunControl(now, &runtime);

        /* OLED使用独立I2C1，每5ms最多刷新一个脏字符。 */
        App_Oled_Poll(now);
        if (!s_oled_failure_logged && App_Oled_ErrorCount() != 0u) {
            s_oled_failure_logged = true;
            elog_w("OLED", "I2C1 display disabled after error=%lu; motion unaffected",
                   (unsigned long)App_Oled_ErrorCount());
        }

        /* 定时任务：按键消抖、显示内容、车辆状态和鱼眼诊断。 */
        if (Main_PeriodElapsed(now, &runtime.last_10ms, 10u)) Key_Scan_Task();
        if (Main_PeriodElapsed(now, &runtime.last_100ms, 100u)) {
            Main_ProcessKeys();
            Main_UpdateOled(now);
        }
        if (Main_PeriodElapsed(now, &runtime.last_500ms, 500u)) Main_LogVehicleStatus(&runtime);
        if (Main_PeriodElapsed(now, &runtime.last_1000ms, 1000u)) Main_LogPathFish();

        /* 事件任务：排空调试串口帧，并推进 ESP/TCP 状态机与遥测队列。 */
        Main_ProcessDebugUart();
        Main_PollCommunication();
    }
}

int main(void)
{
    System_Init();
#if SELFTEST_MODE && APP_LOG_DIAGNOSTIC_ENABLE
    Run_SelfTest();
    elog_i("SYS", "Switching to tracking mode...");
    Bsp_Delay_ms(500);
#endif
    Main_InitializeTracking();
    Main_RunLoop();
}
