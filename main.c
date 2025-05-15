/***************************************************************************//**
* \file main.c
* \version 1.0
*
* Main source file of the USB echo device application.
*
*******************************************************************************
* \copyright
* (c) (2021-2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include <string.h>
#include "cy_usb_common.h"
#include "cy_usbss_cal_drv.h"
#include "cy_usb_echo_device.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"
#include "cy_debug.h"
#include "cybsp.h"
#include "cy_usbd_version.h"
#include "cy_hbdma_version.h"
#include "app_version.h"
#include "cy_usb_app_common.h"
#include "cy_fx_common.h"

/* Select SCB interface used for UART based logging. */
#define LOGGING_SCB             (SCB1)
#define LOGGING_SCB_IDX         (1)
#define DEBUG_LEVEL             (3u)

#if DEBUG_INFRA_EN
/* Debug log related initilization */
#define LOGBUF_SIZE (1024u)
uint8_t logBuff[LOGBUF_SIZE];
#if USBFS_LOGS_ENABLE
    cy_stc_debug_config_t dbgCfg = {logBuff, DEBUG_LEVEL, LOGBUF_SIZE, CY_DEBUG_INTFCE_USBFS_CDC, true};
#else
    cy_stc_debug_config_t dbgCfg = {logBuff, DEBUG_LEVEL, LOGBUF_SIZE, CY_DEBUG_INTFCE_UART_SCB1, true};
#endif /* USBFS_LOGS_ENABLE */
TaskHandle_t printLogTaskHandle;
#endif /* DEBUG_INFRA_EN */


/* GPIO to be used as REMOTE-WAKE trigger: P13.0 */
#define REMOTEWAKE_RQT_PORT     (P13_0_PORT)
#define REMOTEWAKE_RQT_PIN      (P13_0_PIN)

cy_stc_usbss_cal_ctxt_t ssCalCtxt;
cy_stc_usb_cal_ctxt_t   hsCalCtxt;

/* Global variables associated with High BandWidth DMA setup. */
cy_stc_hbdma_context_t HBW_DrvCtxt;             /* High BandWidth DMA driver context. */
cy_stc_hbdma_dscr_list_t HBW_DscrList;          /* High BandWidth DMA descriptor free list. */
cy_stc_hbdma_buf_mgr_t HBW_BufMgr;              /* High BandWidth DMA buffer manager. */
cy_stc_hbdma_mgr_context_t HBW_MgrCtxt;         /* High BandWidth DMA manager context. */

/* CPU DMA register pointers. */
DMAC_Type *pCpuDmacBase;
DW_Type *pCpuDw0Base, *pCpuDw1Base;

cy_stc_usb_usbd_ctxt_t  usbdCtxt;
cy_stc_usb_app_ctxt_t   appCtxt;


/* USB HS related descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];

/* USBSS related descriptors */
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];

/* Common descriptors shared across speed. */
extern const uint8_t CyFxLangString[];
extern const uint8_t CyFxMfgString[];
extern const uint8_t CyFxProdString[];

extern const uint8_t CyFxDevQualDscr[];
extern const uint8_t CyFxBOSDscr[];

#if FREERTOS_ENABLE
extern void xPortPendSVHandler( void );
extern void xPortSysTickHandler( void );
extern void vPortSVCHandler( void );

/*******************************************************************************
* Function Name: Cy_SysTickIntrWrapper
****************************************************************************//**
*
* Wrapper function to handle the systick interrupt.
*
* \param void
*
* \return void
*
*******************************************************************************/

void Cy_SysTickIntrWrapper (void)
{
    Cy_USBD_TickIncrement(&usbdCtxt);
    xPortSysTickHandler();
}

/*******************************************************************************
* Function Name: vPortSetupTimerInterrupt
****************************************************************************//**
* Initialise the systick timer to generate tick events for the FreeRTOS scheduler
*
* \param  void
*
* \return void
*
*******************************************************************************/

void vPortSetupTimerInterrupt( void )
{
    /* Register the exception vectors. */
    Cy_SysInt_SetVector(PendSV_IRQn, xPortPendSVHandler);
    Cy_SysInt_SetVector(SVCall_IRQn, vPortSVCHandler);
    Cy_SysInt_SetVector(SysTick_IRQn, Cy_SysTickIntrWrapper);

    /* Start the SysTick timer with a period of 1 ms. */
    Cy_SysTick_SetClockSource (CY_SYSTICK_CLOCK_SOURCE_CLK_CPU);
    Cy_SysTick_SetReload(Cy_SysClk_ClkFastGetFrequency() / 1000U);
    Cy_SysTick_Clear ();
    Cy_SysTick_Enable ();
}

#if DEBUG_INFRA_EN
/*******************************************************************************
* Function Name: Cy_PrintTaskHandler
****************************************************************************//**
*
* Task to handle display of debug log messages
*
* \param pTaskParam
*  Parameter passed to the task
*
* \return void
*
*******************************************************************************/

void Cy_PrintTaskHandler(void *pTaskParam)
{
    while(1)
    {
        /* Print any pending logs to the output console. */
        Cy_Debug_PrintLog();

        /* Put the thread to sleep for 5 ms */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif /* DEBUG_INFRA_EN */
#endif /* FREERTOS_ENABLE */


/*******************************************************************************
 * Function name: Cy_USB_ConfigureIOs
 ****************************************************************************//**
 *
 * Function used to configure common GPIOs used across all FX10 applications.
 * The following pins are configured in strong drive mode:
 *  P0.0, P0.1   : Used for debug status indication during USB link bring-up
 *  P11.0, P11.1 : Chip level Digital Design For Test pins used for debugging.
 *  P9.2, P9.3   : USB Design For Test GPIOs used for debugging.
 *
 * \param dftEnable
 * Whether the Design For Test pins should be configured for debug function.
 *
 *******************************************************************************/
void Cy_USB_ConfigureIOs (
        bool dftEnable)
{
    cy_stc_gpio_pin_config_t pinCfg;

    /* Clear the structure to start with. */
    memset ((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure P0.0 and P0.1 as General Purpose Output pins
     * with strong output drivers enabled.
     */
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom     = P0_0_GPIO;
    Cy_GPIO_Pin_Init(P0_0_PORT, P0_0_PIN, &pinCfg);
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom     = P0_1_GPIO;
    Cy_GPIO_Pin_Init(P0_1_PORT, P0_1_PIN, &pinCfg);

    if (dftEnable) {
        /* Choose chip level DDFT function for P11.0 and P11.1 pins
         * and enable output drivers.
         */
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P11_0_SRSS_DDFT_PIN_IN0;
        Cy_GPIO_Pin_Init(P11_0_PORT, P11_0_PIN, &pinCfg);
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P11_1_SRSS_DDFT_PIN_IN1;
        Cy_GPIO_Pin_Init(P11_1_PORT, P11_1_PIN, &pinCfg);

        /* Choose HBWSS GPIO DFT function for P9.2 and P9.3 pins and
         * enable output drivers.
         */
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P9_2_LVDS2USB32SS_USB32_GPIO_DDFT_O0;
        Cy_GPIO_Pin_Init(P9_2_PORT, P9_2_PIN, &pinCfg);
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P9_3_LVDS2USB32SS_USB32_GPIO_DDFT_O1;
        Cy_GPIO_Pin_Init(P9_3_PORT, P9_3_PIN, &pinCfg);

        /* Route the DDFT outputs from the High BandWidth SubSystem to
         * the chip level pins.
         */
        SRSS_TST_DDFT_FAST_CTL_REG = 0x00000C0BUL;
        SRSS_TST_DDFT_SLOW_CTL_REG = 0xC0008080UL;
    }
}

/*******************************************************************************
 * Function name: Cy_Fx3G2_OnResetInit
 ****************************************************************************//**
 * TODO Ideally, this should be defined in cybsp.c
 * This function performs initialization that is required to enable scatter
 * loading of data into the High BandWidth RAM during device boot-up. The FX10/FX20
 * device comes up with the High BandWidth RAM disabled and hence any attempt
 * to read/write the RAM will cause the processor to hang. The RAM needs to
 * be enabled with default clock settings to allow scatter loading to work.
 * This function needs to be called from Cy_OnResetUser.
 *
 *******************************************************************************/
void
Cy_Fx3G2_OnResetInit (
        void)
{
    /* Enable clk_hf4 with IMO as input. */
    SRSS->CLK_ROOT_SELECT[4] = SRSS_CLK_ROOT_SELECT_ENABLE_Msk;

    /* Enable LVDS2USB32SS IP and select clk_hf[4] as clock input. */
    MAIN_REG->CTRL = (
            MAIN_REG_CTRL_IP_ENABLED_Msk |
            (1UL << MAIN_REG_CTRL_NUM_FAST_AHB_STALL_CYCLES_Pos) |
            (1UL << MAIN_REG_CTRL_NUM_SLOW_AHB_STALL_CYCLES_Pos) |
            (3UL << MAIN_REG_CTRL_DMA_SRC_SEL_Pos));
}

/*****************************************************************************
 * Function Name: Cy_PrintString
 *****************************************************************************
 * Summary
 *  Prints a string through the SCB0-UART.
 *
 * Parameters:
 *  string - String to be printed.
 *  length - Length of the string in bytes.
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_PrintString(const char *string, uint8_t length)
{
    uint8_t index;

    for (index = 0; index < length; index++)
    {
        while (Cy_SCB_UART_GetNumInTxFifo (LOGGING_SCB) >= 16)
        {
            Cy_SysLib_DelayUs(1);
        }

        Cy_SCB_WriteTxFifo(LOGGING_SCB, (uint32_t)string[index]);
    }
}

#define CHARMAP(c)      (((c) >= 10) ? ('A' + (c) - 10) : ('0' + (c)))

void Cy_PrintByte(uint8_t data)
{
    while (Cy_SCB_UART_GetNumInTxFifo (LOGGING_SCB) >= 16);
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP(data >> 4));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP(data & 0xF));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, ' ');
}

void PrintDword(uint32_t data)
{
    while (Cy_SCB_UART_GetNumInTxFifo (LOGGING_SCB) >= 16);
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0xF0000000) >> 28));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0x0F000000) >> 24));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0x00F00000) >> 20));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0x000F0000) >> 16));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0x0000F000) >> 12));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0x00000F00) >>  8));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0x000000F0) >>  4));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((data & 0x0000000F)));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, '\r');
    Cy_SCB_WriteTxFifo (LOGGING_SCB, '\n');
}

void PrintReg(const char *name, uint8_t len, uint32_t val)
{
    Cy_PrintString(name, len);
    Cy_SCB_WriteTxFifo (LOGGING_SCB, ':');
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val >> 28) & 0x0F));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val >> 24) & 0x0F));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val >> 20) & 0x0F));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val >> 16) & 0x0F));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val >> 12) & 0x0F));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val >>  8) & 0x0F));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val >>  4) & 0x0F));
    Cy_SCB_WriteTxFifo (LOGGING_SCB, CHARMAP((val      ) & 0x0F));
    Cy_PrintString("\r\n", 2);
}

void PrintBuffer(uint8_t *buf_p, uint16_t len)
{
    uint8_t i;
    Cy_PrintString("BUF:", 4);
    Cy_PrintByte(len);
    Cy_SCB_WriteTxFifo (LOGGING_SCB, ':');
    for (i = 0; i < len; i++)
        Cy_PrintByte(buf_p[i]);
    Cy_PrintString("\r\n", 2);
}
/*****************************************************************************
 * Function Name: Cy_USB_HS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-HS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_HS_ISR(void)
{
    if (Cy_USBHS_Cal_IntrHandler(&hsCalCtxt))
    {
        portYIELD_FROM_ISR(true);
    }
}

/*****************************************************************************
 * Function Name: Cy_USB_SS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-HS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_SS_ISR(void)
{
    /* Call the USB32DEV interrupt handler. */
    Cy_USBSS_Cal_IntrHandler(&ssCalCtxt);
    
#if FREERTOS_ENABLE
    portYIELD_FROM_ISR(true);
#endif /* FREERTOS_ENABLE */
}

void Cy_USB_IngressDma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_IN);
#if FREERTOS_ENABLE
    portYIELD_FROM_ISR(true);
#endif /* FREERTOS_ENABLE */
}

void Cy_USB_EgressDma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_EG);
#if FREERTOS_ENABLE
    portYIELD_FROM_ISR(true);
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
 * Function Name: PrintVersionInfo
 ******************************************************************************
 * Summary:
 *  Function to print version information to UART console.
 *
 * Parameters:
 *  type: Type of version string.
 *  version: Version number including major, minor, patch and build number.
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_PrintVersionInfo (const char *type, uint32_t version)
{
    char tString[32];
    uint16_t vBuild;
    uint8_t vMajor, vMinor, vPatch;
    uint8_t typeLen = strlen(type);

    vMajor = (version >> 28U);
    vMinor = ((version >> 24U) & 0x0FU);
    vPatch = ((version >> 16U) & 0xFFU);
    vBuild = (uint16_t)(version & 0xFFFFUL);

    memcpy(tString, type, typeLen);
    tString[typeLen++] = '0' + (vMajor / 10);
    tString[typeLen++] = '0' + (vMajor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vMinor / 10);
    tString[typeLen++] = '0' + (vMinor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vPatch / 10);
    tString[typeLen++] = '0' + (vPatch % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vBuild / 1000);
    tString[typeLen++] = '0' + ((vBuild % 1000) / 100);
    tString[typeLen++] = '0' + ((vBuild % 100) / 10);
    tString[typeLen++] = '0' + (vBuild % 10);
    tString[typeLen++] = '\r';
    tString[typeLen++] = '\n';
    tString[typeLen]   = 0;

    Cy_Debug_AddToLog(1,"%s", tString);
}

/*****************************************************************************
 * Function Name: VbusDetGpio_ISR
 *****************************************************************************
 * Summary
 *  Interrupt handler for the Vbus detect GPIO transition detection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void VbusDetGpio_ISR(void)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = &appCtxt;
    cy_stc_usbd_app_msg_t xMsg;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken;
#endif /* FREERTOS_ENABLE */
    uint32_t gpio_state = Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);

    /* Clear the GPIO interrupt. */
    Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);

    if (gpio_state != 0) {
        if (pAppCtxt->vbusPresent == false) {
            DBG_APP_INFO("VBUS presence detected\r\n");
            pAppCtxt->vbusPresent = true;

            xMsg.type = CY_USB_VBUS_DETECT_PRESENT;
#if FREERTOS_ENABLE
            xQueueSendFromISR(pAppCtxt->xQueue, &xMsg, &xHigherPriorityTaskWoken);
#else
            Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */
        } else {
            DBG_APP_INFO("Spurious GPIO INT - 1\r\n");
        }
    } else {
        if (pAppCtxt->vbusPresent != false) {
            DBG_APP_INFO("VBUS absence detected\r\n");
            pAppCtxt->vbusPresent = false;

            xMsg.type = CY_USB_VBUS_DETECT_ABSENT;
#if FREERTOS_ENABLE
            xQueueSendFromISR(pAppCtxt->xQueue, &xMsg, &xHigherPriorityTaskWoken);
#else
            Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */
        } else {
            DBG_APP_INFO("Spurious GPIO INT - 0\r\n");
        }
    }
}

/*****************************************************************************
 * Function Name: Cy_USB_RemoteWakeGpio_ISR
 *****************************************************************************
 * Summary
 *  Interrupt handler for the remote wake-up trigger GPIO.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void Cy_USB_RemoteWakeGpio_ISR(void)
{
    uint32_t gpio_state = Cy_GPIO_Read(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN);

    /* Clear the GPIO interrupt. */
    Cy_GPIO_ClearInterrupt(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN);

    if (gpio_state != 0) {
    DBG_APP_INFO("Remote wake request detected\r\n");
    /* Duration of signal will be taken care by HS_CAL */
    Cy_USBD_SignalRemoteWakeup(&usbdCtxt, true);
    }
}
/*******************************************************************************
 * Function name: Cy_Fx3g2_InitPeripheralClocks
 ****************************************************************************//**
 *
 * Function used to enable clocks to different peripherals on the FX10/FX20 device.
 *
 * \param adcClkEnable
 * Whether to enable clock to the ADC in the USBSS block.
 *
 * \param usbfsClkEnable
 * Whether to enable bus reset detect clock input to the USBFS block.
 *
 *******************************************************************************/
void Cy_Fx3g2_InitPeripheralClocks (
        bool adcClkEnable,
        bool usbfsClkEnable)
{
    if (adcClkEnable) {
        /* Divide PERI clock at 75 MHz by 75 to get 1 MHz clock using 16-bit divider #1. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1, 74);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_LVDS2USB32SS_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, 1);
    }

    if (usbfsClkEnable) {
        /* Divide PERI clock at 75 MHz by 750 to get 100 KHz clock using 16-bit divider #2. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 2, 749);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 2);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_USB_CLOCK_DEV_BRS, CY_SYSCLK_DIV_16_BIT, 2);
    }
}
/*****************************************************************************
 * Function Name: Cy_USB_USBSSInit
 *****************************************************************************
 * Summary
 *  Initialize USBSS and USBHS block and attempt device enumeration.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_USB_USBSSInit (void)
{
    cy_stc_gpio_pin_config_t pinCfg;
    cy_stc_sysint_t intrCfg;

    /* Configure pins used for debugging. */
    Cy_USB_ConfigureIOs(true);

    memset ((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure VBus detect GPIO. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    pinCfg.intEdge   = CY_GPIO_INTR_BOTH;
    pinCfg.intMask   = 0x01UL;
    Cy_GPIO_Pin_Init(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, &pinCfg);

    /* Register edge detect interrupt for Vbus detect GPIO. */
    intrCfg.intrSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrPriority = 7;
    Cy_SysInt_Init(&intrCfg, VbusDetGpio_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Configure GPIO pin as interrupt source to trigger remote wakeup. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    pinCfg.intEdge   = CY_GPIO_INTR_RISING;
    pinCfg.intMask   = 0x01UL;
    Cy_GPIO_Pin_Init(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN, &pinCfg);

    /* Register edge detect interrupt for Vbus detect GPIO. */
    intrCfg.intrSrc = ioss_interrupts_gpio_dpslp_13_IRQn;
    intrCfg.intrPriority = 7;
    Cy_SysInt_Init(&intrCfg, Cy_USB_RemoteWakeGpio_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* For Bringup  this is required */
    MAIN_REG->CTRL = 0x81100003;

    /* Register the ISR and enable the interrupt. */
    intrCfg.intrSrc      = lvds2usb32ss_usb32_int_o_IRQn;
    intrCfg.intrPriority = 4;
    Cy_SysInt_Init(&intrCfg, &Cy_USB_SS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    intrCfg.intrSrc      = lvds2usb32ss_usb32_wakeup_int_o_IRQn;
    intrCfg.intrPriority = 5;
    Cy_SysInt_Init(&intrCfg, &Cy_USB_SS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* ISRs for the USB Ingress and Egress DMA adapters. */
    intrCfg.intrSrc      = lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
    Cy_SysInt_Init(&intrCfg, &Cy_USB_IngressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    intrCfg.intrSrc      = lvds2usb32ss_usb32_egrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
    Cy_SysInt_Init(&intrCfg, &Cy_USB_EgressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register the ISR for USBHS and enable the interrupt. */
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrPriority = 4;
    Cy_SysInt_Init(&intrCfg, Cy_USB_HS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    intrCfg.intrSrc = usbhsdev_interrupt_u2d_dpslp_o_IRQn;
    intrCfg.intrPriority = 4;
    Cy_SysInt_Init(&intrCfg, Cy_USB_HS_ISR);
    NVIC_EnableIRQ(usbhsdev_interrupt_u2d_dpslp_o_IRQn);
}

/*****************************************************************************
 * Function Name: OutEpDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DMA transfer completion on OUT endpoint.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void
OutEpDma_ISR (uint8_t endpNum)
{
    /*
     * For this application single BULK_OUT endpoint is used so hardcoded value
     * used here.
     */
    DBG_APP_TRACE("...ISR...OutEpDma...endpNum:0x%x\r\n",endpNum);
    if (endpNum == 0x00) {
        Cy_USB_AppClearCpuDmaInterrupt(&appCtxt, endpNum, CY_USB_ENDP_DIR_OUT);
        Cy_USB_Endp0ReadComplete((void *)&appCtxt);
    } else {
        Cy_USB_AppClearCpuDmaInterrupt(&appCtxt, endpNum, CY_USB_ENDP_DIR_OUT);
        Cy_USB_EchoDeviceDmaReadCompletion(&appCtxt, endpNum);
    }
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: InEpDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DMA transfer completion on IN endpoint.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void InEpDma_ISR (uint8_t endpNum)
{
    DBG_APP_TRACE("...ISR...InEpDma...endpNum:0x%x\r\n",endpNum);
    Cy_USB_AppClearCpuDmaInterrupt(&appCtxt, endpNum, CY_USB_ENDP_DIR_IN);
    /* endpoint direction and endp number together makes address */
    Cy_USB_EchoDeviceDmaWriteCompletion(&appCtxt,0x80 | endpNum);
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: Cy_InitHbDma
 ******************************************************************************
 * Summary:
 *  Initialization of HBW DMA.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
bool Cy_InitHbDma(void)
{
    cy_en_hbdma_status_t      drvstat;
    cy_en_hbdma_mgr_status_t  mgrstat;

    /* Initialize the HBW DMA driver layer. */
    drvstat = Cy_HBDma_Init(LVDSSS_LVDS, USB32DEV, &HBW_DrvCtxt, 0, 0);
    if (drvstat != CY_HBDMA_SUCCESS) {
        return false;
    }

    /* Setup a HBW DMA descriptor list. */
    mgrstat = Cy_HBDma_DscrList_Create(&HBW_DscrList, 256U);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS) {
        return false;
    }

    /*
     * Initialize the DMA buffer manager. We will use 256 KB of space
     * from 0x1C040000UL onwards.
     */
    mgrstat =
    Cy_HBDma_BufMgr_Create(&HBW_BufMgr, (uint32_t *)0x1C040000UL, 0x40000UL);

    if (mgrstat != CY_HBDMA_MGR_SUCCESS) {
       return false;
    }

    /* Initialize the HBW DMA channel manager. */
    mgrstat = Cy_HBDma_Mgr_Init(&HBW_MgrCtxt, &HBW_DrvCtxt, &HBW_DscrList, &HBW_BufMgr);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS) {
        return false;
    }

    return true;
}

/*****************************************************************************
* Function Name: Cy_USBSS_DeInit
******************************************************************************
* Summary:
*  Temporary function to reset the USB block and GTX PHY to ensure device
*  disconnects from the host.
*
* Parameters:
*  cy_stc_usbss_cal_ctxt_t *pCalCtxt: Pointer to USB-SS CAL context.

* Return:
*  None
*****************************************************************************/
void
Cy_USBSS_DeInit (cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{

    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type  *USB32DEV_MAIN = &base->USB32DEV_MAIN;

    /* Disable the clock for USB3.2 function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;

    /* Disable PHYSS */
    base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 &=
                    ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 &=
        ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    /* Disable the SuperSpeed Device function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk;
}

/*****************************************************************************
* Function Name: Cy_USB_EnableUsbBlock
******************************************************************************
* Summary:
*  Function to enable the USB32DEV IP block before enabling a new USB
*  connection.
*
* Parameters:
*  None

* Return:
*  None
*****************************************************************************/
void Cy_USB_EnableUsbBlock (void)
{
    /* Enable the USB DMA adapters and respective interrupts. */
    Cy_HBDma_Init(NULL, USB32DEV, &HBW_DrvCtxt, 0, 0);

    NVIC_EnableIRQ(lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn);
    NVIC_EnableIRQ(lvds2usb32ss_usb32_egrs_dma_int_o_IRQn);

    /* Make sure to enable USB32DEV IP first. */
    USB32DEV->USB32DEV_MAIN.CTRL |= USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;
}

/*****************************************************************************
* Function Name: Cy_USB_DisableUsbBlock
******************************************************************************
* Summary:
*  Function to disable the USB32DEV IP block after terminating current
*  connection.
*
* Parameters:
*  None

* Return:
*  None
*****************************************************************************/
static void Cy_USB_DisableUsbBlock (void)
{
    /* Disable the USB32DEV IP. */
    USB32DEV->USB32DEV_MAIN.CTRL &= ~USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;

    /* Disable HBDMA adapter interrupts and the adapter itself. */
    NVIC_DisableIRQ(lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn);
    NVIC_DisableIRQ(lvds2usb32ss_usb32_egrs_dma_int_o_IRQn);
    Cy_HBDma_DeInit(&HBW_DrvCtxt);
    DBG_APP_INFO("ADP DISABLE\r\n");
}


/*****************************************************************************
* Function Name: UsbConnectionEnable
******************************************************************************
* Summary:
*  Function used to enable USB 3.x connection.
*
* Parameters:
*  cy_stc_usb_app_ctxt_t *pAppCtxt: Application context structure.

* Return:
*  void
*****************************************************************************/
bool UsbConnectionEnable (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    DBG_APP_INFO("UsbConnectionEnable >>\r\n");

    Cy_USBD_ConnectDevice(pAppCtxt->pUsbdCtxt, USB_CONN_TYPE);

    pAppCtxt->usbConnectDone = true;
    DBG_APP_INFO("UsbConnectionEnable <<\r\n");
    return true;
}

/*****************************************************************************
* Function Name: UsbConnectionDisable
******************************************************************************
* Summary:
*  Function which disables the USB 3.x connection.
*
* Parameters:
*  cy_stc_usb_app_ctxt_t *pAppCtxt: Application context structure pointer.

* Return:
*  void
*****************************************************************************/
void UsbConnectionDisable (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    Cy_USB_AppHbDmaDestroyEndpDmaSetAll(pAppCtxt);
    Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
    Cy_USBSS_DeInit(pAppCtxt->pUsbdCtxt->pSsCalCtxt);
    pAppCtxt->usbConnectDone = false;
    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    Cy_USB_DisableUsbBlock();
}


/*****************************************************************************
* Function Name: lightDisable
******************************************************************************
* Summary:
*  Function which disables the USB 3.x connection.
*
* Parameters:
*  cy_stc_usb_app_ctxt_t *pAppCtxt: Application context structure pointer.

* Return:
*  void
*****************************************************************************/
void lightDisable (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
    Cy_USBSS_DeInit(pAppCtxt->pUsbdCtxt->pSsCalCtxt);
    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
}

extern void Cy_App_MakeConfigDescriptor(void);

/*****************************************************************************
* Function Name: main(void)
******************************************************************************
* Summary:
*  Entry to the application.
*
* Parameters:
*  void

* Return:
*  Does not return.
*****************************************************************************/
int main (void)
{

    /* Initialize the PDL driver library and set the clock variables. */
    Cy_PDL_Init (&cy_deviceIpBlockCfgFX3G2);

    /* Do all the relevant clock configuration at start-up. */
    cybsp_init();

    Cy_Fx3g2_InitPeripheralClocks(true, true);

    /* Initialize the PDL and register ISR for USB block. */
    Cy_USB_USBSSInit();

    /* Disable the watchdog timer and reset. */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();

    /* Enable interrupts. */
    __enable_irq ();
    
#if DEBUG_INFRA_EN
#if !USBFS_LOGS_ENABLE
    /* Initialize the UART for logging. */
    InitUart(LOGGING_SCB_IDX);

#endif /* USBFS_LOGS_ENABLE */

    Cy_Debug_LogInit(&dbgCfg);
    /* Create task for printing logs and check status. */
    xTaskCreate(Cy_PrintTaskHandler, "PrintLogTask", 512, NULL, 5, &printLogTaskHandle);
    Cy_SysLib_Delay(500);
    Cy_Debug_AddToLog(1, "***** FX20: USBSS Device Application *****\r\n");
    /* Print application, USBD stack and HBDMA version information. */
    Cy_PrintVersionInfo("APP_VERSION: ", APP_VERSION_NUM);
    Cy_PrintVersionInfo("USBD_VERSION: ", USBD_VERSION_NUM);
    Cy_PrintVersionInfo("HBDMA_VERSION: ", HBDMA_VERSION_NUM);
#endif /* DEBUG_INFRA_EN */

    memset((void *)&usbdCtxt, 0, sizeof(cy_stc_usb_usbd_ctxt_t));
    memset((void *)&hsCalCtxt, 0, sizeof(cy_stc_usb_cal_ctxt_t));
    memset((void *)&ssCalCtxt, 0, sizeof(cy_stc_usbss_cal_ctxt_t));
    memset((void *)&appCtxt, 0, sizeof(cy_stc_usb_app_ctxt_t));

    pCpuDmacBase = ((DMAC_Type *)DMAC_BASE);
    pCpuDw0Base  = ((DW_Type *)DW0_BASE);
    pCpuDw1Base  = ((DW_Type *)DW1_BASE);

    /* Store IP base address in CAL context. */
    ssCalCtxt.regBase  = USB32DEV;
    hsCalCtxt.pCalBase = MXS40USBHSDEV_USBHSDEV;
    hsCalCtxt.pPhyBase = MXS40USBHSDEV_USBHSPHY;

    USB32DEV->USB32DEV_MAIN.CTRL |= USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;

    /*
     * Make sure any previous USB connection state is cleared.
     * Give some delay to allow the host to process disconnection.
     */
    Cy_USBSS_DeInit(&ssCalCtxt);
    Cy_SysLib_Delay(1000);

    /* Initialize the HbDma IP and DMA Manager */
    Cy_InitHbDma();
    DBG_APP_INFO("InitHbDma done\r\n");

    /* Initialize the USBD layer */
    Cy_USB_USBD_Init(&appCtxt, &usbdCtxt, pCpuDmacBase, &hsCalCtxt, &ssCalCtxt, &HBW_MgrCtxt);
    DBG_APP_INFO("USBD_Init done\r\n");

    /* Specify that DMA clock should be set to 240 MHz once USB 3.x connection is active. */
    Cy_USBD_SetDmaClkFreq(&usbdCtxt, CY_HBDMA_CLK_240_MHZ);

    /*
     * Routing of debug pins for monitoring:
     * DDFT0      (P11.0): U0
     * DDFT1          (P11.1): U3
     * USB.GPIO_DDFT0 (P9.2) : RX_LFPS from PHY RX
     * USB.GPIO_DDFT1 (P9.3) : TX_LFPS from PHY
     * SIP.GPIO_DDFT0 (P9.4) : Not used
     * SIP.GPIO_DDFT1 (P9.5) : Not used
     */
    Cy_UsbFx_SelectDFTFunctions(CY_FX_DBG_FUNC_USBSS_LTSSM_U0, CY_FX_DBG_FUNC_USBSS_LTSSM_U3,
            CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT, CY_FX_DBG_FUNC_USBPHY0_TX_LFPS_EN,
            CY_FX_DBG_FUNC_NONE, CY_FX_DBG_FUNC_NONE);

    DBG_APP_INFO("Calling SetDscr()  \r\n");
    /* USB 2.0 related descriptors. */
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_HS_DEVICE_DSCR,
            0, (uint8_t *)CyFxUSB20DeviceDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_FS_CONFIG_DSCR,
            0, (uint8_t *)CyFxUSBFSConfigDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_HS_CONFIG_DSCR,
            0, (uint8_t *)CyFxUSBHSConfigDscr);
     Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_DEVICE_QUAL_DSCR,
            0, (uint8_t *)CyFxDevQualDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_HS_BOS_DSCR,
            0, (uint8_t *)CyFxBOSDscr);

    /* Create the configuration descriptor with the required number of endpoints. */
    Cy_App_MakeConfigDescriptor();

    /* Register USB descriptors with the stack. */
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_SS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_SS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxLangString);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxMfgString);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxProdString);

    /* Clear VBus present status. */
    appCtxt.vbusPresent    = false;
    appCtxt.usbConnectDone = false;

    /* Initialize the application and create echo device thread. */
    Cy_USB_AppInit(&appCtxt, &usbdCtxt, pCpuDmacBase, pCpuDw0Base, pCpuDw1Base, &HBW_MgrCtxt);

#if FREERTOS_ENABLE
    /* Invokes scheduler: Not expected to return. */
    vTaskStartScheduler();
#endif /* FREERTOS_ENABLE */

    while(1)
    {
        Cy_SysLib_Delay(10000);
    }

    return 0;
}

/*****************************************************************************
 * Function Name: Cy_OnResetUser(void)
 ******************************************************************************
 * Summary:
 *  Init function which is executed before the load regions in RAM are updated.
 *  The High BandWidth subsystem needs to be enable here to allow variables
 *  placed in the High BandWidth SRAM to be updated.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_OnResetUser (void)
{
    Cy_Fx3G2_OnResetInit();
}

/* [] END OF FILE */
