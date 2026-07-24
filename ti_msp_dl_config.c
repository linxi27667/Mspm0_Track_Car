/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gMotor_Left_TimerBackup;
DL_TimerA_backupConfig gMotor_Right_BIN1_TimerBackup;
DL_TimerG_backupConfig gMotor_Right_BIN2_TimerBackup;
DL_SPI_backupConfig gICM_SPIBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_Motor_Left_Timer_init();
    SYSCFG_DL_Motor_Right_BIN1_Timer_init();
    SYSCFG_DL_Motor_Right_BIN2_Timer_init();
    SYSCFG_DL_PATH_FISH_I2C_init();
    SYSCFG_DL_OLED_I2C_init();
    SYSCFG_DL_UART_Debug_init();
    SYSCFG_DL_UART_ESP01S_init();
    SYSCFG_DL_ICM_SPI_init();
    SYSCFG_DL_DMA_init();
    /* Ensure backup structures have no valid state */
	gMotor_Left_TimerBackup.backupRdy 	= false;
	gMotor_Right_BIN1_TimerBackup.backupRdy 	= false;
	gMotor_Right_BIN2_TimerBackup.backupRdy 	= false;

	gICM_SPIBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(Motor_Left_Timer_INST, &gMotor_Left_TimerBackup);
	retStatus &= DL_TimerA_saveConfiguration(Motor_Right_BIN1_Timer_INST, &gMotor_Right_BIN1_TimerBackup);
	retStatus &= DL_TimerG_saveConfiguration(Motor_Right_BIN2_Timer_INST, &gMotor_Right_BIN2_TimerBackup);
	retStatus &= DL_SPI_saveConfiguration(ICM_SPI_INST, &gICM_SPIBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(Motor_Left_Timer_INST, &gMotor_Left_TimerBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(Motor_Right_BIN1_Timer_INST, &gMotor_Right_BIN1_TimerBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(Motor_Right_BIN2_Timer_INST, &gMotor_Right_BIN2_TimerBackup, false);
	retStatus &= DL_SPI_restoreConfiguration(ICM_SPI_INST, &gICM_SPIBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(Motor_Left_Timer_INST);
    DL_TimerA_reset(Motor_Right_BIN1_Timer_INST);
    DL_TimerG_reset(Motor_Right_BIN2_Timer_INST);
    DL_I2C_reset(PATH_FISH_I2C_INST);
    DL_I2C_reset(OLED_I2C_INST);
    DL_UART_Main_reset(UART_Debug_INST);
    DL_UART_Main_reset(UART_ESP01S_INST);
    DL_SPI_reset(ICM_SPI_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(Motor_Left_Timer_INST);
    DL_TimerA_enablePower(Motor_Right_BIN1_Timer_INST);
    DL_TimerG_enablePower(Motor_Right_BIN2_Timer_INST);
    DL_I2C_enablePower(PATH_FISH_I2C_INST);
    DL_I2C_enablePower(OLED_I2C_INST);
    DL_UART_Main_enablePower(UART_Debug_INST);
    DL_UART_Main_enablePower(UART_ESP01S_INST);
    DL_SPI_enablePower(ICM_SPI_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_Left_Timer_C1_IOMUX,GPIO_Motor_Left_Timer_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_Left_Timer_C1_PORT, GPIO_Motor_Left_Timer_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_Left_Timer_C2_IOMUX,GPIO_Motor_Left_Timer_C2_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_Left_Timer_C2_PORT, GPIO_Motor_Left_Timer_C2_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_Right_BIN1_Timer_C0_IOMUX,GPIO_Motor_Right_BIN1_Timer_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_Right_BIN1_Timer_C0_PORT, GPIO_Motor_Right_BIN1_Timer_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_Right_BIN2_Timer_C1_IOMUX,GPIO_Motor_Right_BIN2_Timer_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_Right_BIN2_Timer_C1_PORT, GPIO_Motor_Right_BIN2_Timer_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_PATH_FISH_I2C_IOMUX_SDA,
        GPIO_PATH_FISH_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_PATH_FISH_I2C_IOMUX_SCL,
        GPIO_PATH_FISH_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_PATH_FISH_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_PATH_FISH_I2C_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_I2C_IOMUX_SDA,
        GPIO_OLED_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_OLED_I2C_IOMUX_SCL,
        GPIO_OLED_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_I2C_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_Debug_IOMUX_TX, GPIO_UART_Debug_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_Debug_IOMUX_RX, GPIO_UART_Debug_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_ESP01S_IOMUX_TX, GPIO_UART_ESP01S_IOMUX_TX_FUNC);
    
	DL_GPIO_initPeripheralInputFunctionFeatures(
		 GPIO_UART_ESP01S_IOMUX_RX, GPIO_UART_ESP01S_IOMUX_RX_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_ICM_SPI_IOMUX_SCLK, GPIO_ICM_SPI_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_ICM_SPI_IOMUX_PICO, GPIO_ICM_SPI_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_ICM_SPI_IOMUX_POCI, GPIO_ICM_SPI_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalOutput(GPIO_GRP_1_PIN_1_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Key_PA30_KEY4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(ICM_CS_CS_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Key_Group_KEY1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_Group_KEY2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Key_Group_KEY3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(Board_Output_STATUS_LED_IOMUX);

    DL_GPIO_initDigitalOutput(Board_Output_BUZZER_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Right_Encoder_PHA_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Right_Encoder_PHB_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Left_Encoder_LEFT_PHA_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Left_Encoder_LEFT_PHB_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOB, GPIO_GRP_1_PIN_1_PIN |
		Board_Output_STATUS_LED_PIN);
    DL_GPIO_setPins(GPIOB, ICM_CS_CS_PIN |
		Board_Output_BUZZER_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_GRP_1_PIN_1_PIN |
		ICM_CS_CS_PIN |
		Board_Output_STATUS_LED_PIN |
		Board_Output_BUZZER_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_12_EDGE_RISE_FALL |
		DL_GPIO_PIN_13_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_21_EDGE_RISE_FALL |
		DL_GPIO_PIN_22_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, Right_Encoder_PHA_PIN |
		Right_Encoder_PHB_PIN |
		Left_Encoder_LEFT_PHA_PIN |
		Left_Encoder_LEFT_PHB_PIN);
    DL_GPIO_enableInterrupt(GPIOB, Right_Encoder_PHA_PIN |
		Right_Encoder_PHB_PIN |
		Left_Encoder_LEFT_PHA_PIN |
		Left_Encoder_LEFT_PHB_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();

}


/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gMotor_Left_TimerClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gMotor_Left_TimerConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 100,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Motor_Left_Timer_init(void) {

    DL_TimerA_setClockConfig(
        Motor_Left_Timer_INST, (DL_TimerA_ClockConfig *) &gMotor_Left_TimerClockConfig);

    DL_TimerA_initPWMMode(
        Motor_Left_Timer_INST, (DL_TimerA_PWMConfig *) &gMotor_Left_TimerConfig);

    DL_TimerA_setCaptureCompareValue(Motor_Left_Timer_INST, 100, DL_TIMER_CC_1_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(Motor_Left_Timer_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_Left_Timer_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptureCompareValue(Motor_Left_Timer_INST, 100, DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(Motor_Left_Timer_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_Left_Timer_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_enableClock(Motor_Left_Timer_INST);



    DL_TimerA_setCCPDirection(Motor_Left_Timer_INST , DL_TIMER_CC1_OUTPUT | DL_TIMER_CC2_OUTPUT );

}
/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gMotor_Right_BIN1_TimerClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gMotor_Right_BIN1_TimerConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 100,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Motor_Right_BIN1_Timer_init(void) {

    DL_TimerA_setClockConfig(
        Motor_Right_BIN1_Timer_INST, (DL_TimerA_ClockConfig *) &gMotor_Right_BIN1_TimerClockConfig);

    DL_TimerA_initPWMMode(
        Motor_Right_BIN1_Timer_INST, (DL_TimerA_PWMConfig *) &gMotor_Right_BIN1_TimerConfig);

    DL_TimerA_setCaptureCompareValue(Motor_Right_BIN1_Timer_INST, 100, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareOutCtl(Motor_Right_BIN1_Timer_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_Right_BIN1_Timer_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_enableClock(Motor_Right_BIN1_Timer_INST);


    
    DL_TimerA_setCCPDirection(Motor_Right_BIN1_Timer_INST , DL_TIMER_CC0_OUTPUT );

}
/*
 * Timer clock configuration to be sourced by  / 8 (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gMotor_Right_BIN2_TimerClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gMotor_Right_BIN2_TimerConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 100,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Motor_Right_BIN2_Timer_init(void) {

    DL_TimerG_setClockConfig(
        Motor_Right_BIN2_Timer_INST, (DL_TimerG_ClockConfig *) &gMotor_Right_BIN2_TimerClockConfig);

    DL_TimerG_initPWMMode(
        Motor_Right_BIN2_Timer_INST, (DL_TimerG_PWMConfig *) &gMotor_Right_BIN2_TimerConfig);

    DL_TimerG_setCaptureCompareValue(Motor_Right_BIN2_Timer_INST, 100, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(Motor_Right_BIN2_Timer_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(Motor_Right_BIN2_Timer_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_enableClock(Motor_Right_BIN2_Timer_INST);


    
    DL_TimerG_setCCPDirection(Motor_Right_BIN2_Timer_INST , DL_TIMER_CC1_OUTPUT );

}


static const DL_I2C_ClockConfig gPATH_FISH_I2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_PATH_FISH_I2C_init(void) {

    DL_I2C_setClockConfig(PATH_FISH_I2C_INST,
        (DL_I2C_ClockConfig *) &gPATH_FISH_I2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(PATH_FISH_I2C_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(PATH_FISH_I2C_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(PATH_FISH_I2C_INST, 7);
    DL_I2C_setControllerTXFIFOThreshold(PATH_FISH_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(PATH_FISH_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(PATH_FISH_I2C_INST);


    /* Enable module */
    DL_I2C_enableController(PATH_FISH_I2C_INST);


}
static const DL_I2C_ClockConfig gOLED_I2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_OLED_I2C_init(void) {

    DL_I2C_setClockConfig(OLED_I2C_INST,
        (DL_I2C_ClockConfig *) &gOLED_I2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(OLED_I2C_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(OLED_I2C_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(OLED_I2C_INST, 7);
    DL_I2C_setControllerTXFIFOThreshold(OLED_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(OLED_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(OLED_I2C_INST);


    /* Enable module */
    DL_I2C_enableController(OLED_I2C_INST);


}


static const DL_UART_Main_ClockConfig gUART_DebugClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_DebugConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_Debug_init(void)
{
    DL_UART_Main_setClockConfig(UART_Debug_INST, (DL_UART_Main_ClockConfig *) &gUART_DebugClockConfig);

    DL_UART_Main_init(UART_Debug_INST, (DL_UART_Main_Config *) &gUART_DebugConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_Debug_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_Debug_INST, UART_Debug_IBRD_32_MHZ_115200_BAUD, UART_Debug_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_Debug_INST,
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);

    /* Configure DMA Receive Event */
    DL_UART_Main_enableDMAReceiveEvent(UART_Debug_INST, DL_UART_DMA_INTERRUPT_RX);
    /* Configure DMA Transmit Event */
    DL_UART_Main_enableDMATransmitEvent(UART_Debug_INST);
    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_Debug_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_Debug_INST, DL_UART_RX_FIFO_LEVEL_1_4_FULL);
    DL_UART_Main_setTXFIFOThreshold(UART_Debug_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    DL_UART_Main_setRXInterruptTimeout(UART_Debug_INST, 15);

    DL_UART_Main_enable(UART_Debug_INST);
}

static const DL_UART_Main_ClockConfig gUART_ESP01SClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_ESP01SConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_ESP01S_init(void)
{
    DL_UART_Main_setClockConfig(UART_ESP01S_INST, (DL_UART_Main_ClockConfig *) &gUART_ESP01SClockConfig);

    DL_UART_Main_init(UART_ESP01S_INST, (DL_UART_Main_Config *) &gUART_ESP01SConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_ESP01S_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_ESP01S_INST, UART_ESP01S_IBRD_32_MHZ_115200_BAUD, UART_ESP01S_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_ESP01S_INST,
                                 DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);

    /* Configure DMA Receive Event */
    DL_UART_Main_enableDMAReceiveEvent(UART_ESP01S_INST, DL_UART_DMA_INTERRUPT_RX);
    /* Configure DMA Transmit Event */
    DL_UART_Main_enableDMATransmitEvent(UART_ESP01S_INST);
    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_ESP01S_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_ESP01S_INST, DL_UART_RX_FIFO_LEVEL_1_4_FULL);
    DL_UART_Main_setTXFIFOThreshold(UART_ESP01S_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    DL_UART_Main_setRXInterruptTimeout(UART_ESP01S_INST, 15);

    DL_UART_Main_enable(UART_ESP01S_INST);
}

static const DL_SPI_Config gICM_SPI_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gICM_SPI_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_ICM_SPI_init(void) {
    DL_SPI_setClockConfig(ICM_SPI_INST, (DL_SPI_ClockConfig *) &gICM_SPI_clockConfig);

    DL_SPI_init(ICM_SPI_INST, (DL_SPI_Config *) &gICM_SPI_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     1000000 = (32000000)/((1 + 15) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(ICM_SPI_INST, 15);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(ICM_SPI_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(ICM_SPI_INST);
}

static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_Debug_INST_DMA_TRIGGER_0,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void)
{
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID , (DL_DMA_Config *) &gDMA_CH0Config);
}
static const DL_DMA_Config gDMA_CH1Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_UNCHANGED,
    .srcIncrement   = DL_DMA_ADDR_INCREMENT,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_Debug_INST_DMA_TRIGGER_1,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH1_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL1);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL1);
    DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID , (DL_DMA_Config *) &gDMA_CH1Config);
}
static const DL_DMA_Config gDMA_CH2Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_ESP01S_INST_DMA_TRIGGER_0,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH2_init(void)
{
    DL_DMA_initChannel(DMA, DMA_CH2_CHAN_ID , (DL_DMA_Config *) &gDMA_CH2Config);
}
static const DL_DMA_Config gDMA_CH3Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_UNCHANGED,
    .srcIncrement   = DL_DMA_ADDR_INCREMENT,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_ESP01S_INST_DMA_TRIGGER_1,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH3_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL3);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL3);
    DL_DMA_initChannel(DMA, DMA_CH3_CHAN_ID , (DL_DMA_Config *) &gDMA_CH3Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_CH0_init();
    SYSCFG_DL_DMA_CH1_init();
    SYSCFG_DL_DMA_CH2_init();
    SYSCFG_DL_DMA_CH3_init();
}


