/**
 * @file    app_grayscale.c
 * @brief   八路灰度传感器硬件绑定实现（MSPM0G3507）
 * @note    ADC 读取改进:
 *          - HW_Gray_Init 中确保 PA27 为模拟输入（PINCM=0）
 *          - 补充 DL_ADC12_initSingleSample 配置 CTL1/CTL2 (SysConfig 生成代码缺失)
 *          - MANUAL 电源模式 (SDK 示例: 保持 ADC 上电, 非自动掉电)
 *          - 每次读取后重新 enableConversions (ENC 位可能被硬件清除)
 */
#include "app_grayscale.h"
#include "app_common.h"
#include "app_log.h"
#include "ti_msp_dl_config.h"
#include <elog.h>
#include <stdbool.h>

/* ================= 1. 硬件底层函数 (HW_ 前缀) ================= */

static void HW_Gray_Init(void)
{
    /* 1. PA27 = IOMUX_PINCM60, 清除数字功能让 ADC 模拟输入生效 */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM60] = 0;

    /* 2. 关键修复: SysConfig 生成代码缺少 DL_ADC12_initSingleSample 调用,
     *    导致 CTL1.SAMPMODE=0(手动采样), SC 位无法触发自动采样.
     *    参考SDK adc12_single_conversion 示例配置. */
    DL_ADC12_initSingleSample(GRAYSCALE_ADC_INST,
        DL_ADC12_REPEAT_MODE_DISABLED,          /* 单次转换, 不重复 */
        DL_ADC12_SAMPLING_SOURCE_AUTO,          /* 自动采样 (SC位触发) */
        DL_ADC12_TRIG_SRC_SOFTWARE,             /* 软件触发 */
        DL_ADC12_SAMP_CONV_RES_12_BIT,          /* 12位分辨率 */
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED /* 无符号格式 */
    );

    /* 3. MANUAL 电源模式 — SDK示例注释: "Disable automatic power down mode
     *    to get non-zero ADC result". MANUAL=保持上电, AUTO=转换间掉电 */
    DL_ADC12_setPowerDownMode(GRAYSCALE_ADC_INST, DL_ADC12_POWER_DOWN_MODE_MANUAL);

    /* 4. 使能转换 (ENC=1) */
    DL_ADC12_enableConversions(GRAYSCALE_ADC_INST);

    /* 5. 等待 ADC 稳定 */
    Bsp_Delay_ms(10);

    APP_GRAY_STATE_LOG_I("GRAY", "Init: PA27 PINCM60=0x%08lX PWRDN=MANUAL ENC=%d",
           (unsigned long)IOMUX->SECCFG.PINCM[IOMUX_PINCM60],
           DL_ADC12_isConversionsEnabled(GRAYSCALE_ADC_INST) ? 1 : 0);
}

static void HW_Gray_Gpio_Write(void *port, uint32_t pin, uint8_t level)
{
    if (port == NULL) return;
    GPIO_WritePin((GPIO_Regs *)port, pin, level);
}

/* ADC 读取 — 参考 SDK ADCMSPM0.c 轮询方式 + adc12_single_conversion 示例
 * 关键: 每次读取后必须重新 enableConversions, 因为 ENC 位可能被硬件清除 */
static uint16_t HW_Gray_ADC_Read(void)
{
    static uint32_t s_last_timeout_log = 0;

    /* 清除之前可能残留的中断标志 */
    DL_ADC12_clearInterruptStatus(GRAYSCALE_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    /* 启动软件转换 (设置 SC 位, 触发自动采样+转换) */
    DL_ADC12_startConversion(GRAYSCALE_ADC_INST);

    /* 等待 MEM0 结果就绪 (raw 标志, 不依赖中断使能), 加超时保护 */
    uint32_t start = Bsp_GetTick();
    bool timeout = false;
    while ((DL_ADC12_getRawInterruptStatus(GRAYSCALE_ADC_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) &
            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0) {
        if ((Bsp_GetTick() - start) >= 2u) {
            timeout = true;
            break;
        }
    }

    if (timeout) {
        DL_ADC12_stopConversion(GRAYSCALE_ADC_INST);
        DL_ADC12_clearInterruptStatus(GRAYSCALE_ADC_INST,
            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        DL_ADC12_disableConversions(GRAYSCALE_ADC_INST);
        DL_ADC12_enableConversions(GRAYSCALE_ADC_INST);
        if ((Bsp_GetTick() - s_last_timeout_log) >= 500u) {
            s_last_timeout_log = Bsp_GetTick();
            APP_GRAY_STATE_LOG_W("GRAY", "ADC timeout ch=%u, frame invalid",
                                 (unsigned)Grayscale_Sensor.current_ch);
        }
        return BSP_GRAY_ADC_INVALID;
    }

    /* 读取结果（12位，0~4095） */
    uint16_t result = DL_ADC12_getMemResult(GRAYSCALE_ADC_INST, DL_ADC12_MEM_IDX_0);

    /* 清除中断标志, 为下次读取做准备 */
    DL_ADC12_clearInterruptStatus(GRAYSCALE_ADC_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);

    /* 重新使能转换 (SDK示例: ENC位可能被硬件清除) */
    DL_ADC12_enableConversions(GRAYSCALE_ADC_INST);

    return result;
}

static uint32_t HW_Gray_GetTick(void)
{
    return Bsp_GetTick();
}

/* ================= 2. 对象实例化与引脚拼装 ================= */

bsp_grayscale_t Grayscale_Sensor = {
    .ad0        = {GRAY_AD0_PORT, GRAY_AD0_PIN},
    .ad1        = {GRAY_AD1_PORT, GRAY_AD1_PIN},
    .ad2        = {GRAY_AD2_PORT, GRAY_AD2_PIN},
    .settle_ms  = BSP_GRAY_SETTLE_MS,

    .Init       = HW_Gray_Init,
    .Gpio_Write = HW_Gray_Gpio_Write,
    .ADC_Read   = HW_Gray_ADC_Read,
    .GetTick    = HW_Gray_GetTick
};

/* ================= 3. 对外业务切入点 ================= */

void App_Grayscale_Init(void)
{
    Bsp_Grayscale_Init_Device(&Grayscale_Sensor);
}

void Grayscale_Update(void)
{
    Bsp_Grayscale_Update(&Grayscale_Sensor);
}

void Grayscale_Calibrate_White(void)
{
    Bsp_Grayscale_Calibrate_White(&Grayscale_Sensor);
}

void Grayscale_Calibrate_Black(void)
{
    Bsp_Grayscale_Calibrate_Black(&Grayscale_Sensor);
}

bool Grayscale_Calibration_Valid(uint16_t minimum_contrast)
{
    return Bsp_Grayscale_Calibration_Valid(&Grayscale_Sensor,
                                           minimum_contrast);
}
