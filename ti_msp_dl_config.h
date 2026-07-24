/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for Motor_Left_Timer */
#define Motor_Left_Timer_INST                                              TIMA0
#define Motor_Left_Timer_INST_IRQHandler                        TIMA0_IRQHandler
#define Motor_Left_Timer_INST_INT_IRQN                          (TIMA0_INT_IRQn)
#define Motor_Left_Timer_INST_CLK_FREQ                                   4000000
/* GPIO defines for channel 1 */
#define GPIO_Motor_Left_Timer_C1_PORT                                      GPIOA
#define GPIO_Motor_Left_Timer_C1_PIN                              DL_GPIO_PIN_22
#define GPIO_Motor_Left_Timer_C1_IOMUX                           (IOMUX_PINCM47)
#define GPIO_Motor_Left_Timer_C1_IOMUX_FUNC             IOMUX_PINCM47_PF_TIMA0_CCP1
#define GPIO_Motor_Left_Timer_C1_IDX                         DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_Motor_Left_Timer_C2_PORT                                      GPIOA
#define GPIO_Motor_Left_Timer_C2_PIN                              DL_GPIO_PIN_15
#define GPIO_Motor_Left_Timer_C2_IOMUX                           (IOMUX_PINCM37)
#define GPIO_Motor_Left_Timer_C2_IOMUX_FUNC             IOMUX_PINCM37_PF_TIMA0_CCP2
#define GPIO_Motor_Left_Timer_C2_IDX                         DL_TIMER_CC_2_INDEX

/* Defines for Motor_Right_BIN1_Timer */
#define Motor_Right_BIN1_Timer_INST                                        TIMA1
#define Motor_Right_BIN1_Timer_INST_IRQHandler                        TIMA1_IRQHandler
#define Motor_Right_BIN1_Timer_INST_INT_IRQN                        (TIMA1_INT_IRQn)
#define Motor_Right_BIN1_Timer_INST_CLK_FREQ                                 4000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_Right_BIN1_Timer_C0_PORT                                   GPIOB
#define GPIO_Motor_Right_BIN1_Timer_C0_PIN                           DL_GPIO_PIN_0
#define GPIO_Motor_Right_BIN1_Timer_C0_IOMUX                         (IOMUX_PINCM12)
#define GPIO_Motor_Right_BIN1_Timer_C0_IOMUX_FUNC             IOMUX_PINCM12_PF_TIMA1_CCP0
#define GPIO_Motor_Right_BIN1_Timer_C0_IDX                     DL_TIMER_CC_0_INDEX

/* Defines for Motor_Right_BIN2_Timer */
#define Motor_Right_BIN2_Timer_INST                                        TIMG6
#define Motor_Right_BIN2_Timer_INST_IRQHandler                        TIMG6_IRQHandler
#define Motor_Right_BIN2_Timer_INST_INT_IRQN                        (TIMG6_INT_IRQn)
#define Motor_Right_BIN2_Timer_INST_CLK_FREQ                                 4000000
/* GPIO defines for channel 1 */
#define GPIO_Motor_Right_BIN2_Timer_C1_PORT                                   GPIOB
#define GPIO_Motor_Right_BIN2_Timer_C1_PIN                          DL_GPIO_PIN_11
#define GPIO_Motor_Right_BIN2_Timer_C1_IOMUX                         (IOMUX_PINCM28)
#define GPIO_Motor_Right_BIN2_Timer_C1_IOMUX_FUNC             IOMUX_PINCM28_PF_TIMG6_CCP1
#define GPIO_Motor_Right_BIN2_Timer_C1_IDX                     DL_TIMER_CC_1_INDEX




/* Defines for PATH_FISH_I2C */
#define PATH_FISH_I2C_INST                                                  I2C0
#define PATH_FISH_I2C_INST_IRQHandler                            I2C0_IRQHandler
#define PATH_FISH_I2C_INST_INT_IRQN                                I2C0_INT_IRQn
#define PATH_FISH_I2C_BUS_SPEED_HZ                                        400000
#define GPIO_PATH_FISH_I2C_SDA_PORT                                        GPIOA
#define GPIO_PATH_FISH_I2C_SDA_PIN                                 DL_GPIO_PIN_0
#define GPIO_PATH_FISH_I2C_IOMUX_SDA                              (IOMUX_PINCM1)
#define GPIO_PATH_FISH_I2C_IOMUX_SDA_FUNC                IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_PATH_FISH_I2C_SCL_PORT                                        GPIOA
#define GPIO_PATH_FISH_I2C_SCL_PIN                                 DL_GPIO_PIN_1
#define GPIO_PATH_FISH_I2C_IOMUX_SCL                              (IOMUX_PINCM2)
#define GPIO_PATH_FISH_I2C_IOMUX_SCL_FUNC                IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for OLED_I2C */
#define OLED_I2C_INST                                                       I2C1
#define OLED_I2C_INST_IRQHandler                                 I2C1_IRQHandler
#define OLED_I2C_INST_INT_IRQN                                     I2C1_INT_IRQn
#define OLED_I2C_BUS_SPEED_HZ                                             400000
#define GPIO_OLED_I2C_SDA_PORT                                             GPIOA
#define GPIO_OLED_I2C_SDA_PIN                                     DL_GPIO_PIN_18
#define GPIO_OLED_I2C_IOMUX_SDA                                  (IOMUX_PINCM40)
#define GPIO_OLED_I2C_IOMUX_SDA_FUNC                   IOMUX_PINCM40_PF_I2C1_SDA
#define GPIO_OLED_I2C_SCL_PORT                                             GPIOA
#define GPIO_OLED_I2C_SCL_PIN                                     DL_GPIO_PIN_17
#define GPIO_OLED_I2C_IOMUX_SCL                                  (IOMUX_PINCM39)
#define GPIO_OLED_I2C_IOMUX_SCL_FUNC                   IOMUX_PINCM39_PF_I2C1_SCL


/* Defines for UART_Debug */
#define UART_Debug_INST                                                    UART0
#define UART_Debug_INST_IRQHandler                              UART0_IRQHandler
#define UART_Debug_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_UART_Debug_RX_PORT                                            GPIOA
#define GPIO_UART_Debug_TX_PORT                                            GPIOA
#define GPIO_UART_Debug_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_UART_Debug_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_UART_Debug_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_UART_Debug_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_UART_Debug_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_Debug_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define UART_Debug_BAUD_RATE                                            (115200)
#define UART_Debug_IBRD_32_MHZ_115200_BAUD                                  (17)
#define UART_Debug_FBRD_32_MHZ_115200_BAUD                                  (23)
/* Defines for UART_ESP01S */
#define UART_ESP01S_INST                                                   UART1
#define UART_ESP01S_INST_IRQHandler                             UART1_IRQHandler
#define UART_ESP01S_INST_INT_IRQN                                 UART1_INT_IRQn
#define GPIO_UART_ESP01S_RX_PORT                                           GPIOA
#define GPIO_UART_ESP01S_TX_PORT                                           GPIOA
#define GPIO_UART_ESP01S_RX_PIN                                    DL_GPIO_PIN_9
#define GPIO_UART_ESP01S_TX_PIN                                    DL_GPIO_PIN_8
#define GPIO_UART_ESP01S_IOMUX_RX                                (IOMUX_PINCM20)
#define GPIO_UART_ESP01S_IOMUX_TX                                (IOMUX_PINCM19)
#define GPIO_UART_ESP01S_IOMUX_RX_FUNC                 IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_ESP01S_IOMUX_TX_FUNC                 IOMUX_PINCM19_PF_UART1_TX
#define UART_ESP01S_BAUD_RATE                                           (115200)
#define UART_ESP01S_IBRD_32_MHZ_115200_BAUD                                 (17)
#define UART_ESP01S_FBRD_32_MHZ_115200_BAUD                                 (23)




/* Defines for IMU_SPI */
#define IMU_SPI_INST                                                       SPI1
#define IMU_SPI_INST_IRQHandler                                 SPI1_IRQHandler
#define IMU_SPI_INST_INT_IRQN                                     SPI1_INT_IRQn
#define GPIO_IMU_SPI_PICO_PORT                                            GPIOB
#define GPIO_IMU_SPI_PICO_PIN                                     DL_GPIO_PIN_8
#define GPIO_IMU_SPI_IOMUX_PICO                                 (IOMUX_PINCM25)
#define GPIO_IMU_SPI_IOMUX_PICO_FUNC                 IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_IMU_SPI_POCI_PORT                                            GPIOB
#define GPIO_IMU_SPI_POCI_PIN                                     DL_GPIO_PIN_7
#define GPIO_IMU_SPI_IOMUX_POCI                                 (IOMUX_PINCM24)
#define GPIO_IMU_SPI_IOMUX_POCI_FUNC                 IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for IMU_SPI */
#define GPIO_IMU_SPI_SCLK_PORT                                            GPIOB
#define GPIO_IMU_SPI_SCLK_PIN                                     DL_GPIO_PIN_9
#define GPIO_IMU_SPI_IOMUX_SCLK                                 (IOMUX_PINCM26)
#define GPIO_IMU_SPI_IOMUX_SCLK_FUNC                 IOMUX_PINCM26_PF_SPI1_SCLK



/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define UART_Debug_INST_DMA_TRIGGER_0                        (DMA_UART0_RX_TRIG)

/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (1)
#define UART_Debug_INST_DMA_TRIGGER_1                        (DMA_UART0_TX_TRIG)

/* Defines for DMA_CH2 */
#define DMA_CH2_CHAN_ID                                                      (2)
#define UART_ESP01S_INST_DMA_TRIGGER_0                       (DMA_UART1_RX_TRIG)

/* Defines for DMA_CH3 */
#define DMA_CH3_CHAN_ID                                                      (3)
#define UART_ESP01S_INST_DMA_TRIGGER_1                       (DMA_UART1_TX_TRIG)



/* Port definition for Pin Group GPIO_GRP_1 */
#define GPIO_GRP_1_PORT                                                  (GPIOB)

/* Defines for PIN_1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_GRP_1_PIN_1_PIN                                    (DL_GPIO_PIN_14)
#define GPIO_GRP_1_PIN_1_IOMUX                                   (IOMUX_PINCM31)
/* Port definition for Pin Group Key_PA30 */
#define Key_PA30_PORT                                                    (GPIOA)

/* Defines for KEY4: GPIOA.30 with pinCMx 5 on package pin 37 */
#define Key_PA30_KEY4_PIN                                       (DL_GPIO_PIN_30)
#define Key_PA30_KEY4_IOMUX                                       (IOMUX_PINCM5)
/* Port definition for Pin Group IMU_CS */
#define IMU_CS_PORT                                                      (GPIOB)

/* Defines for CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define IMU_CS_CS_PIN                                            (DL_GPIO_PIN_6)
#define IMU_CS_CS_IOMUX                                          (IOMUX_PINCM23)
/* Port definition for Pin Group Key_Group */
#define Key_Group_PORT                                                   (GPIOB)

/* Defines for KEY1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define Key_Group_KEY1_PIN                                      (DL_GPIO_PIN_15)
#define Key_Group_KEY1_IOMUX                                     (IOMUX_PINCM32)
/* Defines for KEY2: GPIOB.16 with pinCMx 33 on package pin 4 */
#define Key_Group_KEY2_PIN                                      (DL_GPIO_PIN_16)
#define Key_Group_KEY2_IOMUX                                     (IOMUX_PINCM33)
/* Defines for KEY3: GPIOB.2 with pinCMx 15 on package pin 50 */
#define Key_Group_KEY3_PIN                                       (DL_GPIO_PIN_2)
#define Key_Group_KEY3_IOMUX                                     (IOMUX_PINCM15)
/* Port definition for Pin Group Board_Output */
#define Board_Output_PORT                                                (GPIOB)

/* Defines for STATUS_LED: GPIOB.4 with pinCMx 17 on package pin 52 */
#define Board_Output_STATUS_LED_PIN                              (DL_GPIO_PIN_4)
#define Board_Output_STATUS_LED_IOMUX                            (IOMUX_PINCM17)
/* Defines for BUZZER: GPIOB.5 with pinCMx 18 on package pin 53 */
#define Board_Output_BUZZER_PIN                                  (DL_GPIO_PIN_5)
#define Board_Output_BUZZER_IOMUX                                (IOMUX_PINCM18)
/* Port definition for Pin Group Right_Encoder */
#define Right_Encoder_PORT                                               (GPIOB)

/* Defines for PHA: GPIOB.12 with pinCMx 29 on package pin 64 */
// groups represented: ["Left_Encoder","Right_Encoder"]
// pins affected: ["LEFT_PHA","LEFT_PHB","PHA","PHB"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define Right_Encoder_PHA_IIDX                              (DL_GPIO_IIDX_DIO12)
#define Right_Encoder_PHA_PIN                                   (DL_GPIO_PIN_12)
#define Right_Encoder_PHA_IOMUX                                  (IOMUX_PINCM29)
/* Defines for PHB: GPIOB.13 with pinCMx 30 on package pin 1 */
#define Right_Encoder_PHB_IIDX                              (DL_GPIO_IIDX_DIO13)
#define Right_Encoder_PHB_PIN                                   (DL_GPIO_PIN_13)
#define Right_Encoder_PHB_IOMUX                                  (IOMUX_PINCM30)
/* Port definition for Pin Group Left_Encoder */
#define Left_Encoder_PORT                                                (GPIOB)

/* Defines for LEFT_PHA: GPIOB.21 with pinCMx 49 on package pin 20 */
#define Left_Encoder_LEFT_PHA_IIDX                          (DL_GPIO_IIDX_DIO21)
#define Left_Encoder_LEFT_PHA_PIN                               (DL_GPIO_PIN_21)
#define Left_Encoder_LEFT_PHA_IOMUX                              (IOMUX_PINCM49)
/* Defines for LEFT_PHB: GPIOB.22 with pinCMx 50 on package pin 21 */
#define Left_Encoder_LEFT_PHB_IIDX                          (DL_GPIO_IIDX_DIO22)
#define Left_Encoder_LEFT_PHB_PIN                               (DL_GPIO_PIN_22)
#define Left_Encoder_LEFT_PHB_IOMUX                              (IOMUX_PINCM50)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_Motor_Left_Timer_init(void);
void SYSCFG_DL_Motor_Right_BIN1_Timer_init(void);
void SYSCFG_DL_Motor_Right_BIN2_Timer_init(void);
void SYSCFG_DL_PATH_FISH_I2C_init(void);
void SYSCFG_DL_OLED_I2C_init(void);
void SYSCFG_DL_UART_Debug_init(void);
void SYSCFG_DL_UART_ESP01S_init(void);
void SYSCFG_DL_IMU_SPI_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
