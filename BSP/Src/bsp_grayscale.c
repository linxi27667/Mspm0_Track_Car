/**
 * @file    bsp_grayscale.c
 * @brief   八路灰度传感器核心逻辑实现（BSP — 纯逻辑，跨平台）
 * @note    非阻塞状态机：每次调用只读1路，8次完成一轮扫描
 *          含归一化(0=白,1=黑) + 滞回比较 + black_count统计
 */
#include "bsp_grayscale.h"

/* ================= 内部函数 ================= */

/* 设置通道选择地址线（AD0/AD1/AD2 = ch的二进制） */
static void Gray_Select_Channel(bsp_grayscale_t *gs, uint8_t ch)
{
    if (gs->Gpio_Write) {
        gs->Gpio_Write(gs->ad0.port, gs->ad0.pin, (ch >> 0) & 1);
        gs->Gpio_Write(gs->ad1.port, gs->ad1.pin, (ch >> 1) & 1);
        gs->Gpio_Write(gs->ad2.port, gs->ad2.pin, (ch >> 2) & 1);
    }
}

/* 滞回比较器 — 抗抖动判断单通道黑白（自适应极性）
 * 支持两种传感器极性:
 *   正常极性: 黑线ADC低, 白地ADC高 (大多数传感器)
 *   反向极性: 黑线ADC高, 白地ADC低 (部分传感器LED朝向不同)
 * 通过比较 black/white 大小自动判断极性。
 * mid = (black + white) / 2
 * hyst = |white - black| / 4   (25%滞回)
 * 正常极性: ADC > mid+hyst → 白;  ADC < mid-hyst → 黑
 * 反向极性: ADC > mid+hyst → 黑;  ADC < mid-hyst → 白
 *
 * 0~4095内的低ADC可能是真实深黑，不能用幅值猜测硬件故障；ADC读取
 * 超时由BSP_GRAY_ADC_INVALID/frame_fault单独上报。 */
static void Gray_Hysteresis(bsp_grayscale_t *gs, uint8_t ch)
{
    uint16_t sample = gs->scan_analog[ch];
    uint16_t b = gs->black[ch];
    uint16_t w = gs->white[ch];
    uint16_t mid  = (b + w) / 2;
    uint16_t hyst = (w > b) ? ((w - b) / 4) : ((b - w) / 4);
    uint16_t high = mid + hyst;
    uint16_t low  = mid - hyst;

    if (w >= b) {
        /* 正常极性: 黑线ADC低, 白地ADC高 */
        if (sample > high) {
            gs->scan_digital[ch] = 0;    /* 白色 */
        } else if (sample < low) {
            gs->scan_digital[ch] = 1;    /* 黑色 */
        }
    } else {
        /* 反向极性: 黑线ADC高, 白地ADC低 */
        if (sample > high) {
            gs->scan_digital[ch] = 1;    /* 黑色 */
        } else if (sample < low) {
            gs->scan_digital[ch] = 0;    /* 白色 */
        }
    }
    /* low ≤ analog ≤ high → 保持上一状态（惯性抗抖动） */
}

/* 归一化：将 ADC 值映射到 0(白)~1(黑)（自适应极性）
 * 未校准时用简单线性映射; 校准后用白/黑基准值映射
 * 正常极性: black<white, ADC越高越白
 * 反向极性: black>white, ADC越低越白 */
static void Gray_Normalize(bsp_grayscale_t *gs, uint8_t ch)
{
    uint16_t sample = gs->scan_analog[ch];

    if (!gs->calibrated) {
        /* 未校准：简单线性映射 (ADC越大→越白→norm越小) */
        gs->scan_norm[ch] = 1.0f - (float)sample / (float)BSP_GRAY_ADC_MAX;
        return;
    }

    uint16_t w = gs->white[ch];
    uint16_t b = gs->black[ch];
    int32_t range = (int32_t)w - (int32_t)b;
    if (range < 0) range = -range;  /* 反向极性取绝对值 */
    if (range < 1) range = 1;       /* 防止除零 */

    int32_t val;
    if (w >= b) {
        /* 正常极性: norm=0 → 白(高ADC), norm=1 → 黑(低ADC) */
        val = (int32_t)sample - b;
        if (val < 0) val = 0;
        if (val > range) val = range;
        gs->scan_norm[ch] = 1.0f - (float)val / (float)range;
    } else {
        /* 反向极性: norm=0 → 白(低ADC), norm=1 → 黑(高ADC) */
        val = (int32_t)b - (int32_t)sample;
        if (val < 0) val = 0;
        if (val > range) val = range;
        gs->scan_norm[ch] = 1.0f - (float)val / (float)range;
    }
}

/* 原子发布一轮完整扫描；主循环只读取发布快照。 */
static void Gray_Publish_Frame(bsp_grayscale_t *gs)
{
    gs->black_count = 0;
    for (uint8_t i = 0; i < BSP_GRAY_CHANNEL_NUM; i++) {
        gs->analog[i] = gs->scan_analog[i];
        gs->norm[i] = gs->scan_norm[i];
        gs->digital[i] = gs->scan_digital[i];
        if (gs->digital[i]) gs->black_count++;
    }
    gs->frame_fault = gs->scan_fault;
    gs->scan_fault = 0u;
    gs->data_ready = 1;
}

/* ================= 1. 设备初始化 ================= */
void Bsp_Grayscale_Init_Device(bsp_grayscale_t *gs)
{
    if (gs == NULL) return;

    /* 稳定时间缺省值 */
    if (gs->settle_ms == 0) gs->settle_ms = BSP_GRAY_SETTLE_MS;

    /* 清零所有数据 */
    for (uint8_t i = 0; i < BSP_GRAY_CHANNEL_NUM; i++) {
        gs->analog[i]  = 0;
        gs->norm[i]    = 0;
        gs->white[i]   = 0;
        gs->black[i]   = 0;
        gs->digital[i] = 0;
        gs->scan_analog[i] = 0;
        gs->scan_norm[i] = 0;
        gs->scan_digital[i] = 0;
    }

    gs->current_ch       = 0;
    gs->data_ready       = 0;
    gs->frame_fault      = 0;
    gs->scan_fault       = 0;
    gs->calibrated       = 0;
    gs->black_calibrated = 0;
    gs->black_count      = 0;
    gs->last_switch_tick = 0;

    /* 调用硬件初始化（GPIO/ADC由APP层SysConfig配置） */
    if (gs->Init) gs->Init();

    /* 初始选通通道0，开始第一轮扫描 */
    Gray_Select_Channel(gs, 0);
    if (gs->GetTick) gs->last_switch_tick = gs->GetTick();
}

/* ================= 2. 非阻塞更新（状态机） ================= */
void Bsp_Grayscale_Update(bsp_grayscale_t *gs)
{
    if (gs == NULL || gs->ADC_Read == NULL || gs->GetTick == NULL) return;

    /* 未到稳定时间，跳过本次调用 */
    uint32_t now = gs->GetTick();
    if ((now - gs->last_switch_tick) < gs->settle_ms) return;

    gs->last_switch_tick = now;

    /* ① 读取当前通道ADC值（上一次选通的通道，现已稳定） */
    uint16_t sample = gs->ADC_Read();
    bool sample_valid = sample != BSP_GRAY_ADC_INVALID &&
                        sample <= BSP_GRAY_ADC_MAX;
    if (sample_valid) {
        gs->scan_analog[gs->current_ch] = sample;
    } else {
        /* 不把上一通道的值灌入当前通道。保留该通道上一轮工作值，
         * 并锁存整帧故障；控制层看到frame_fault后必须停止运动。 */
        gs->scan_fault = 1u;
    }

    /* ①.5 校准策略: 开机自动校准white + 手动K4校准black
     * 去除动态跟踪: 实测发现动态跟踪会把black越拉越低(白地ADC噪声波动),
     *   最终 black=0 导致滞回区间失效, Digital乱跳。
     * 现在black只由手动K4校准确定, 保证基准稳定。
     * white由开机自动校准确定(车放白地), 后续不更新。 */

    /* ② 归一化（0=白, 1=黑） */
    if (sample_valid) {
        Gray_Normalize(gs, gs->current_ch);
    }

    /* ③ 黑白判断: 滞回比较器(自适应极性)
     * 需要白色校准(calibrated) + 黑色校准(black_calibrated)都完成才判断,
     * 否则强制白色(防止black=white/2猜测值导致反向极性传感器误判) */
    if (sample_valid && gs->calibrated && gs->black_calibrated) {
        Gray_Hysteresis(gs, gs->current_ch);
    } else if (sample_valid) {
        gs->scan_digital[gs->current_ch] = 0;  /* 未校准黑线: 强制白 */
    }

    /* ④ 切换到下一个通道 */
    gs->current_ch = (gs->current_ch + 1) & 0x07;
    Gray_Select_Channel(gs, gs->current_ch);

    /* ⑤ 一轮8路扫描完成 */
    if (gs->current_ch == 0) {
        Gray_Publish_Frame(gs);
    }
}

/* ================= 3. 校准接口 ================= */
/* 使用方法：先让传感器扫描至少一轮（data_ready=1），再调用校准 */
void Bsp_Grayscale_Calibrate_White(bsp_grayscale_t *gs)
{
    if (gs == NULL || gs->frame_fault) return;
    for (uint8_t i = 0; i < BSP_GRAY_CHANNEL_NUM; i++) {
        gs->white[i] = gs->analog[i];
    }
    gs->calibrated = 1;
}

void Bsp_Grayscale_Calibrate_Black(bsp_grayscale_t *gs)
{
    if (gs == NULL || gs->frame_fault) return;
    for (uint8_t i = 0; i < BSP_GRAY_CHANNEL_NUM; i++) {
        gs->black[i] = gs->analog[i];
    }
    gs->calibrated = 1;
    gs->black_calibrated = 1;  /* 标记黑色已手动校准 */
}

bool Bsp_Grayscale_Calibration_Valid(const bsp_grayscale_t *gs,
                                     uint16_t minimum_contrast)
{
    if (gs == NULL || !gs->calibrated || !gs->black_calibrated) return false;
    return Bsp_Grayscale_ContrastValid8(gs->white, gs->black,
                                        minimum_contrast);
}
