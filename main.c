/***************************************************************************//**
* \file main.c
* \version 1.0
*
* Main source file of the USB echo device application.
*
*******************************************************************************
* \copyright
* (c) (2021-2024), Cypress Semiconductor Corporation (an Infineon company) or
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

#if USBX2_EN
#if USBSS_GEN2_ENABLE
#define USBSS_TGT_SPEED (CY_USBD_USB_DEV_SS_GEN2X2)
#else
#define USBSS_TGT_SPEED (CY_USBD_USB_DEV_SS_GEN1X2)
#endif /* USBSS_GEN2_ENABLE */
#else
#if USBSS_GEN2_ENABLE
#define USBSS_TGT_SPEED (CY_USBD_USB_DEV_SS_GEN2)
#else
#define USBSS_TGT_SPEED (CY_USBD_USB_DEV_SS_GEN1)
#endif /* USBSS_GEN2_ENABLE */
#endif /* USBX2_EN */

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

void Cy_SysTickIntrWrapper (void)
{
    Cy_USBD_TickIncrement(&usbdCtxt);
    xPortSysTickHandler();
}

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

/**
 * @typedef cy_en_fxusb_dbg_func_sel_t
 * @brief List of EZ-USB FX debug functions which can be routed to the Digital
 * Design for Test (DDFT) IOs of the device.
 */
typedef enum {
    CY_FX_DBG_FUNC_NONE                       = 0x0000,         /**< No debug function selected. */

    CY_FX_DBG_FUNC_USBPHY0_RXEQ_TRAINING      = 0x0002,         /**< USB PHY0 in RxEq training. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_RATE_10G      = 0x0003,         /**< USB PHY0 PIPE data rate set to 10Gbps. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_RX_TERM       = 0x0004,         /**< USB PHY0 RX termination enable. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_RX_ELEC_IDLE  = 0x0005,         /**< USB PHY0 PIPE RX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_TX_ELEC_IDLE  = 0x0006,         /**< USB PHY0 PIPE TX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY0_PIPE_CLK_EN        = 0x0007,         /**< USB PHY0 PIPE clock enable. */
    CY_FX_DBG_FUNC_USBPHY0_P3_ENTRY           = 0x0008,         /**< USB PHY0 P3 entry PHY status. */
    CY_FX_DBG_FUNC_USBPHY0_P3_EXIT            = 0x0009,         /**< USB PHY0 P3 exit PHY status. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P0           = 0x000A,         /**< USB PHY0 in P0 state. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P1           = 0x000B,         /**< USB PHY0 in P1 state. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P2           = 0x000C,         /**< USB PHY0 in P2 state. */
    CY_FX_DBG_FUNC_USBPHY0_STATE_P3           = 0x000D,         /**< USB PHY0 in P3 state. */
    CY_FX_DBG_FUNC_USBPHY0_TX_SER_EN          = 0x000E,         /**< USB PHY0 Transmit Serialiser enable. */
    CY_FX_DBG_FUNC_USBPHY0_TX_ELEC_IDLE       = 0x000F,         /**< USB PHY0 Transmit Electrical Idle. */
    CY_FX_DBG_FUNC_USBPHY0_TX_LFPS_EN         = 0x0010,         /**< USB PHY0 LFPS Transmit enable. */
    CY_FX_DBG_FUNC_USBPHY0_TX_HS_CLKDRV_EN    = 0x0011,         /**< USB PHY0 Transmit HS clock driver enable. */
    CY_FX_DBG_FUNC_USBPHY0_CDR_LOCK_TO_REF    = 0x0012,         /**< USB PHY0 CDR in Lock to Reference mode. */
    CY_FX_DBG_FUNC_USBPHY0_PROT_LPBK_EN       = 0x0013,         /**< USB PHY0 Protocol level loopback enabled. */
    CY_FX_DBG_FUNC_USBPHY0_TX_PULLUP_EN       = 0x0014,         /**< USB PHY0 TX pull-up enable for Rx Detection. */
    CY_FX_DBG_FUNC_USBPHY0_TX_RXDET_OP        = 0x0015,         /**< USB PHY0 Rx Detection Output. */
    CY_FX_DBG_FUNC_USBPHY0_RX_PRESENT         = 0x0016,         /**< USB PHY0 Receiver Present status. */
    CY_FX_DBG_FUNC_USBPHY0_AFE_ZCAL_COMP_OUT  = 0x0017,         /**< USB PHY0 TX impedance calibration output. */
    CY_FX_DBG_FUNC_USBPHY0_WARM_RST_DET       = 0x0018,         /**< USB PHY0 detecting Warm reset. */
    CY_FX_DBG_FUNC_USBPHY0_MASK_LFPS_DET      = 0x0019,         /**< USB PHY0 LFPS detect status masked. */
    CY_FX_DBG_FUNC_USBPHY0_HS_DATA_PRESENT    = 0x001A,         /**< USB PHY0 High Speed Data Present output. */
    CY_FX_DBG_FUNC_USBPHY0_HS_DATA_IN_U0      = 0x001B,         /**< USB PHY0 HS data in U0 state. */
    CY_FX_DBG_FUNC_USBPHY0_RX_HUNT_FOR_ALIGN  = 0x001C,         /**< USB PHY0 receiver hunting for alignment. */
    CY_FX_DBG_FUNC_USBPHY0_RX_ALIGN_FORCED    = 0x001D,         /**< USB PHY0 receiver data alignment forced. */
    CY_FX_DBG_FUNC_USBPHY0_G1_ELBUF_UNDERFLOW = 0x001E,         /**< USB PHY0 Gen1 Elastic Buffer Underflow. */
    CY_FX_DBG_FUNC_USBPHY0_G1_ELBUF_OVERFLOW  = 0x001F          /**< USB PHY0 Gen1 Elastic Buffer Overflow. */,
    CY_FX_DBG_FUNC_USBPHY0_RX_ALIGNED         = 0x0020,         /**< USB PHY0 Receiver Aligned status. */
    CY_FX_DBG_FUNC_USBPHY0_RX_LOCKED          = 0x0021,         /**< USB PHY0 Receiver Locked status. */
    CY_FX_DBG_FUNC_USBPHY0_G1_8B10B_DISP      = 0x0022,         /**< USB PHY0 active Gen1 8B10B Disparity. */
    CY_FX_DBG_FUNC_USBPHY0_G2_ELBUF_UNDERFLOW = 0x0023          /**< USB PHY0 Gen2 Elastic Buffer Underflow. */,
    CY_FX_DBG_FUNC_USBPHY0_G3_ELBUF_OVERFLOW  = 0x0024          /**< USB PHY0 Gen2 Elastic Buffer Overflow. */,
    CY_FX_DBG_FUNC_USBPHY0_ADC_COMP_OUT       = 0x0025,         /**< USB PHY0 ADC comparator output. */
    CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT     = 0x0081,         /**< USB PHY0 Receiver LFPS detector output. */
    CY_FX_DBG_FUNC_USBPHY0_RX_SIGNAL_LOCK     = 0x0083,         /**< USB PHY0 Receiver Signal lock status. */
    CY_FX_DBG_FUNC_USBPHY0_RX_AFE_OSA_DONE    = 0x0085,         /**< USB PHY0 AFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY0_RX_FILTERED_LOCK   = 0x008B,         /**< USB PHY0 Receiver signal lock post filtering. */
    CY_FX_DBG_FUNC_USBPHY0_RX_DFE_OSA_DONE    = 0x008D,         /**< USB PHY0 DFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY0_RX_REFCLK_BY_2     = 0x008E,         /**< USB PHY0 reference clock divided by 2. */

    CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING      = 0x0102,         /**< USB PHY1 in RxEq training. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_RATE_10G      = 0x0103,         /**< USB PHY1 PIPE data rate set to 10Gbps. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_RX_TERM       = 0x0104,         /**< USB PHY1 RX termination enable. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_RX_ELEC_IDLE  = 0x0105,         /**< USB PHY1 PIPE RX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_TX_ELEC_IDLE  = 0x0106,         /**< USB PHY1 PIPE TX electrical idle signal. */
    CY_FX_DBG_FUNC_USBPHY1_PIPE_CLK_EN        = 0x0107,         /**< USB PHY1 PIPE clock enable. */
    CY_FX_DBG_FUNC_USBPHY1_P3_ENTRY           = 0x0108,         /**< USB PHY1 P3 entry PHY status. */
    CY_FX_DBG_FUNC_USBPHY1_P3_EXIT            = 0x0109,         /**< USB PHY1 P3 exit PHY status. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P0           = 0x010A,         /**< USB PHY1 in P0 state. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P1           = 0x010B,         /**< USB PHY1 in P1 state. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P2           = 0x010C,         /**< USB PHY1 in P2 state. */
    CY_FX_DBG_FUNC_USBPHY1_STATE_P3           = 0x010D,         /**< USB PHY1 in P3 state. */
    CY_FX_DBG_FUNC_USBPHY1_TX_SER_EN          = 0x010E,         /**< USB PHY1 Transmit Serialiser enable. */
    CY_FX_DBG_FUNC_USBPHY1_TX_ELEC_IDLE       = 0x010F,         /**< USB PHY1 Transmit Electrical Idle. */
    CY_FX_DBG_FUNC_USBPHY1_TX_LFPS_EN         = 0x0110,         /**< USB PHY1 LFPS Transmit enable. */
    CY_FX_DBG_FUNC_USBPHY1_TX_HS_CLKDRV_EN    = 0x0111,         /**< USB PHY1 Transmit HS clock driver enable. */
    CY_FX_DBG_FUNC_USBPHY1_CDR_LOCK_TO_REF    = 0x0112,         /**< USB PHY1 CDR in Lock to Reference mode. */
    CY_FX_DBG_FUNC_USBPHY1_PROT_LPBK_EN       = 0x0113,         /**< USB PHY1 Protocol level loopback enabled. */
    CY_FX_DBG_FUNC_USBPHY1_TX_PULLUP_EN       = 0x0114,         /**< USB PHY1 TX pull-up enable for Rx Detection. */
    CY_FX_DBG_FUNC_USBPHY1_TX_RXDET_OP        = 0x0115,         /**< USB PHY1 Rx Detection Output. */
    CY_FX_DBG_FUNC_USBPHY1_RX_PRESENT         = 0x0116,         /**< USB PHY1 Receiver Present status. */
    CY_FX_DBG_FUNC_USBPHY1_AFE_ZCAL_COMP_OUT  = 0x0117,         /**< USB PHY1 TX impedance calibration output. */
    CY_FX_DBG_FUNC_USBPHY1_WARM_RST_DET       = 0x0118,         /**< USB PHY1 detecting Warm reset. */
    CY_FX_DBG_FUNC_USBPHY1_MASK_LFPS_DET      = 0x0119,         /**< USB PHY1 LFPS detect status masked. */
    CY_FX_DBG_FUNC_USBPHY1_HS_DATA_PRESENT    = 0x011A,         /**< USB PHY1 High Speed Data Present output. */
    CY_FX_DBG_FUNC_USBPHY1_HS_DATA_IN_U0      = 0x011B,         /**< USB PHY1 HS data in U0 state. */
    CY_FX_DBG_FUNC_USBPHY1_RX_HUNT_FOR_ALIGN  = 0x011C,         /**< USB PHY1 receiver hunting for alignment. */
    CY_FX_DBG_FUNC_USBPHY1_RX_ALIGN_FORCED    = 0x011D,         /**< USB PHY1 receiver data alignment forced. */
    CY_FX_DBG_FUNC_USBPHY1_G1_ELBUF_UNDERFLOW = 0x011E,         /**< USB PHY1 Gen1 Elastic Buffer Underflow. */
    CY_FX_DBG_FUNC_USBPHY1_G1_ELBUF_OVERFLOW  = 0x011F          /**< USB PHY1 Gen1 Elastic Buffer Overflow. */,
    CY_FX_DBG_FUNC_USBPHY1_RX_ALIGNED         = 0x0120,         /**< USB PHY1 Receiver Aligned status. */
    CY_FX_DBG_FUNC_USBPHY1_RX_LOCKED          = 0x0121,         /**< USB PHY1 Receiver Locked status. */
    CY_FX_DBG_FUNC_USBPHY1_G1_8B10B_DISP      = 0x0122,         /**< USB PHY1 active Gen1 8B10B Disparity. */
    CY_FX_DBG_FUNC_USBPHY1_G2_ELBUF_UNDERFLOW = 0x0123          /**< USB PHY1 Gen2 Elastic Buffer Underflow. */,
    CY_FX_DBG_FUNC_USBPHY1_G3_ELBUF_OVERFLOW  = 0x0124          /**< USB PHY1 Gen2 Elastic Buffer Overflow. */,
    CY_FX_DBG_FUNC_USBPHY1_ADC_COMP_OUT       = 0x0125,         /**< USB PHY1 ADC comparator output. */
    CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT     = 0x0181,         /**< USB PHY1 Receiver LFPS detector output. */
    CY_FX_DBG_FUNC_USBPHY1_RX_SIGNAL_LOCK     = 0x0183,         /**< USB PHY1 Receiver Signal lock status. */
    CY_FX_DBG_FUNC_USBPHY1_RX_AFE_OSA_DONE    = 0x0185,         /**< USB PHY1 AFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY1_RX_FILTERED_LOCK   = 0x018B,         /**< USB PHY1 Receiver signal lock post filtering. */
    CY_FX_DBG_FUNC_USBPHY1_RX_DFE_OSA_DONE    = 0x018D,         /**< USB PHY1 DFE offset calibration done. */
    CY_FX_DBG_FUNC_USBPHY1_RX_REFCLK_BY_2     = 0x018E,         /**< USB PHY1 reference clock divided by 2. */

    CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE       = 0x0204,         /**< USB Egress Socket #0 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK1_ACTIVE       = 0x0205,         /**< USB Egress Socket #1 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK2_ACTIVE       = 0x0206,         /**< USB Egress Socket #2 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK3_ACTIVE       = 0x0207,         /**< USB Egress Socket #3 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK4_ACTIVE       = 0x0208,         /**< USB Egress Socket #4 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK5_ACTIVE       = 0x0209,         /**< USB Egress Socket #5 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK6_ACTIVE       = 0x020A,         /**< USB Egress Socket #6 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK7_ACTIVE       = 0x020B,         /**< USB Egress Socket #7 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK8_ACTIVE       = 0x020C,         /**< USB Egress Socket #8 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK9_ACTIVE       = 0x020D,         /**< USB Egress Socket #9 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK10_ACTIVE      = 0x020E,         /**< USB Egress Socket #10 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK11_ACTIVE      = 0x020F,         /**< USB Egress Socket #11 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK12_ACTIVE      = 0x0210,         /**< USB Egress Socket #12 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK13_ACTIVE      = 0x0211,         /**< USB Egress Socket #13 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK14_ACTIVE      = 0x0212,         /**< USB Egress Socket #14 active. */
    CY_FX_DBG_FUNC_USBSS_EG_SCK15_ACTIVE      = 0x0213,         /**< USB Egress Socket #15 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK0_ACTIVE       = 0x0214,         /**< USB Ingress Socket #0 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK1_ACTIVE       = 0x0215,         /**< USB Ingress Socket #1 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK2_ACTIVE       = 0x0216,         /**< USB Ingress Socket #2 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK3_ACTIVE       = 0x0217,         /**< USB Ingress Socket #3 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK4_ACTIVE       = 0x0218,         /**< USB Ingress Socket #4 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK5_ACTIVE       = 0x0219,         /**< USB Ingress Socket #5 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK6_ACTIVE       = 0x021A,         /**< USB Ingress Socket #6 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK7_ACTIVE       = 0x021B,         /**< USB Ingress Socket #7 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK8_ACTIVE       = 0x021C,         /**< USB Ingress Socket #8 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK9_ACTIVE       = 0x021D,         /**< USB Ingress Socket #9 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK10_ACTIVE      = 0x021E,         /**< USB Ingress Socket #10 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK11_ACTIVE      = 0x021F,         /**< USB Ingress Socket #11 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK12_ACTIVE      = 0x0220,         /**< USB Ingress Socket #12 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK13_ACTIVE      = 0x0221,         /**< USB Ingress Socket #13 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK14_ACTIVE      = 0x0222,         /**< USB Ingress Socket #14 active. */
    CY_FX_DBG_FUNC_USBSS_IN_SCK15_ACTIVE      = 0x0223,         /**< USB Ingress Socket #15 active. */
    CY_FX_DBG_FUNC_USBSS_EP0_T1_SCH_RDY       = 0x0224,         /**< USB Endpoint 0 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP1_T1_SCH_RDY       = 0x0225,         /**< USB Endpoint 1 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP2_T1_SCH_RDY       = 0x0226,         /**< USB Endpoint 2 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP3_T1_SCH_RDY       = 0x0227,         /**< USB Endpoint 3 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP4_T1_SCH_RDY       = 0x0228,         /**< USB Endpoint 4 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP5_T1_SCH_RDY       = 0x0229,         /**< USB Endpoint 5 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP6_T1_SCH_RDY       = 0x022A,         /**< USB Endpoint 6 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP7_T1_SCH_RDY       = 0x022B,         /**< USB Endpoint 7 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP8_T1_SCH_RDY       = 0x022C,         /**< USB Endpoint 8 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP9_T1_SCH_RDY       = 0x022D,         /**< USB Endpoint 9 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP10_T1_SCH_RDY      = 0x022E,         /**< USB Endpoint 10 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP11_T1_SCH_RDY      = 0x022F,         /**< USB Endpoint 11 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP12_T1_SCH_RDY      = 0x0230,         /**< USB Endpoint 12 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP13_T1_SCH_RDY      = 0x0231,         /**< USB Endpoint 13 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP14_T1_SCH_RDY      = 0x0232,         /**< USB Endpoint 14 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP15_T1_SCH_RDY      = 0x0233,         /**< USB Endpoint 15 ready for Type-1 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP0_T2_SCH_RDY       = 0x0234,         /**< USB Endpoint 0 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP1_T2_SCH_RDY       = 0x0235,         /**< USB Endpoint 1 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP2_T2_SCH_RDY       = 0x0236,         /**< USB Endpoint 2 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP3_T2_SCH_RDY       = 0x0237,         /**< USB Endpoint 3 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP4_T2_SCH_RDY       = 0x0238,         /**< USB Endpoint 4 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP5_T2_SCH_RDY       = 0x0239,         /**< USB Endpoint 5 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP6_T2_SCH_RDY       = 0x023A,         /**< USB Endpoint 6 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP7_T2_SCH_RDY       = 0x023B,         /**< USB Endpoint 7 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP8_T2_SCH_RDY       = 0x023C,         /**< USB Endpoint 8 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP9_T2_SCH_RDY       = 0x023D,         /**< USB Endpoint 9 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP10_T2_SCH_RDY      = 0x023E,         /**< USB Endpoint 10 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP11_T2_SCH_RDY      = 0x023F,         /**< USB Endpoint 11 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP12_T2_SCH_RDY      = 0x0240,         /**< USB Endpoint 12 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP13_T2_SCH_RDY      = 0x0241,         /**< USB Endpoint 13 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP14_T2_SCH_RDY      = 0x0242,         /**< USB Endpoint 14 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_EP15_T2_SCH_RDY      = 0x0243,         /**< USB Endpoint 15 ready for Type-2 transfer. */
    CY_FX_DBG_FUNC_USBSS_PROT_T2_HDR_FULL     = 0x0275,         /**< USB protocol layer Type-2 transmit header full. */
    CY_FX_DBG_FUNC_USBSS_PROT_HDR_FULL        = 0x0276,         /**< USB protocol layer transmit header full. */
    CY_FX_DBG_FUNC_USBSS_LCW_READY            = 0x0277,         /**< Link Command Word ready for transmit. */
    CY_FX_DBG_FUNC_USBSS_DPP_RETRY            = 0x0278,         /**< USB Data packet payload retry. */
    CY_FX_DBG_FUNC_USBSS_RX_HDR_EMPTY         = 0x0279,         /**< USB protocol receive header empty condition. */
    CY_FX_DBG_FUNC_USBSS_TX_ELECIDLE          = 0x027A,         /**< USB PIPE level signal for TX electrical idle. */
    CY_FX_DBG_FUNC_USBSS_TXDET_LPBK           = 0x027B,         /**< USB PIPE level signal indicating loopback. */
    CY_FX_DBG_FUNC_USBSS_RX_VALID             = 0x027C,         /**< USB PIPE level signal indicating Rx.Valid */
    CY_FX_DBG_FUNC_USBSS_RX_ELECIDLE          = 0x027D,         /**< USB PIPE level signal for RX electrical idle. */
    CY_FX_DBG_FUNC_USBSS_RATE_10G             = 0x027E,         /**< USB PIPE level signal indicating 10G data rate */
    CY_FX_DBG_FUNC_USBSS_PWRDOWN_1            = 0x027F,         /**< USB PIPE power down signal #1 */
    CY_FX_DBG_FUNC_USBSS_PWRDOWN_0            = 0x0280,         /**< USB PIPE power down signal #0 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_COMP           = 0x0282,         /**< USB link state is Compliance. */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U3             = 0x0283,         /**< USB link state is U3 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U2             = 0x0284,         /**< USB link state is U2 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U1             = 0x0285,         /**< USB link state is U1 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_U0             = 0x0286,         /**< USB link state is U0 */
    CY_FX_DBG_FUNC_USBSS_LTSSM_LPBK_EXIT      = 0x0287,         /**< USB link state is Loopback.Exit */
    CY_FX_DBG_FUNC_USBSS_LTSSM_LPBK_ACTV      = 0x0288,         /**< USB link state is Loopback.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_HOTRST_EXIT    = 0x0289,         /**< USB link state is HotReset.Exit */
    CY_FX_DBG_FUNC_USBSS_LTSSM_HOTRST_ACTV    = 0x028A,         /**< USB link state is HotReset.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RECOV_IDLE     = 0x028B,         /**< USB link state is Recovery.Idle */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RECOV_CFG      = 0x028C,         /**< USB link state is Recovery.Configuration */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RECOV_ACTV     = 0x028D,         /**< USB link state is Recovery.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_IDLE      = 0x028E,         /**< USB link state is Polling.Idle */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_CFG       = 0x028F,         /**< USB link state is Polling.Configuration */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_ACTV      = 0x0290,         /**< USB link state is Polling.Active */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_RXEQ      = 0x0291,         /**< USB link state is Polling.RxEq */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_PORTCFG   = 0x0292,         /**< USB link state is Polling.PortConfig */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_PORTMATCH = 0x0293,         /**< USB link state is Polling.PortMatch */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_LFPSPLUS  = 0x0294,         /**< USB link state is Polling.LFPSPlus */
    CY_FX_DBG_FUNC_USBSS_LTSSM_POLL_LFPS      = 0x0295,         /**< USB link state is Polling.LFPS */
    CY_FX_DBG_FUNC_USBSS_LTSSM_RXDET_ACTV     = 0x0296,         /**< USB link state is RxDetect.Active */

    CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE     = 0x1000,         /**< LVDS/LVCMOS Link 0 training done. */
    CY_FX_DBG_FUNC_SIP_LNK1_TRAINING_DONE     = 0x1001,         /**< LVDS/LVCMOS Link 1 training done. */
    CY_FX_DBG_FUNC_SIP_LNK0_FIFO_OVERFLOW     = 0x1002,         /**< LVDS/LVCMOS Link 0 FIFO overflow */
    CY_FX_DBG_FUNC_SIP_LNK1_FIFO_OVERFLOW     = 0x1003,         /**< LVDS/LVCMOS Link 1 FIFO overflow */
    CY_FX_DBG_FUNC_SIP_LNK0_FIFO_UNDERFLOW    = 0x1004,         /**< LVDS/LVCMOS Link 0 FIFO underflow */
    CY_FX_DBG_FUNC_SIP_LNK1_FIFO_UNDERFLOW    = 0x1005,         /**< LVDS/LVCMOS Link 1 FIFO underflow */
    CY_FX_DBG_FUNC_SIP_LNK0_TRAIN_BLK_DETECT  = 0x1008,         /**< Training block detected on LVDS/LVCMOS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_TRAIN_BLK_FAILED  = 0x1009,         /**< Training block detection failed on link 0 */
    CY_FX_DBG_FUNC_SIP_LNK1_TRAIN_BLK_DETECT  = 0x100A,         /**< Training block detected on LVDS/LVCMOS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_TRAIN_BLK_FAILED  = 0x100B,         /**< Training block detection failed on link 1 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE0_TRAIN_BLK   = 0x100C,         /**< Training block detected on lane 0 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE1_TRAIN_BLK   = 0x100D,         /**< Training block detected on lane 1 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE2_TRAIN_BLK   = 0x100E,         /**< Training block detected on lane 2 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE3_TRAIN_BLK   = 0x100F,         /**< Training block detected on lane 3 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE4_TRAIN_BLK   = 0x1010,         /**< Training block detected on lane 4 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE5_TRAIN_BLK   = 0x1011,         /**< Training block detected on lane 5 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE6_TRAIN_BLK   = 0x1012,         /**< Training block detected on lane 6 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE7_TRAIN_BLK   = 0x1013,         /**< Training block detected on lane 7 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK0_LANE8_TRAIN_BLK   = 0x1014,         /**< Training block detected on lane 8 of LVDS link 0 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE0_TRAIN_BLK   = 0x1015,         /**< Training block detected on lane 0 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE1_TRAIN_BLK   = 0x1016,         /**< Training block detected on lane 1 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE2_TRAIN_BLK   = 0x1017,         /**< Training block detected on lane 2 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE3_TRAIN_BLK   = 0x1018,         /**< Training block detected on lane 3 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE4_TRAIN_BLK   = 0x1019,         /**< Training block detected on lane 4 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE5_TRAIN_BLK   = 0x101A,         /**< Training block detected on lane 5 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE6_TRAIN_BLK   = 0x101B,         /**< Training block detected on lane 6 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE7_TRAIN_BLK   = 0x101C,         /**< Training block detected on lane 7 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK1_LANE8_TRAIN_BLK   = 0x101D,         /**< Training block detected on lane 8 of LVDS link 1 */
    CY_FX_DBG_FUNC_SIP_LNK0_DATA_IOS_OFF      = 0x101F,         /**< Data IOs of LVDS/LVCMOS link 0 switched off */
    CY_FX_DBG_FUNC_SIP_LNK0_CTRL_IOS_OFF      = 0x1020,         /**< Control IOs of LVDS/LVCMOS link 0 switched off */
    CY_FX_DBG_FUNC_SIP_LNK0_L1_STATE          = 0x1021,         /**< LVDS/LVCMOS link 0 in L1 state. */
    CY_FX_DBG_FUNC_SIP_LNK0_L3_STATE          = 0x1022,         /**< LVDS/LVCMOS link 0 in L3 state. */
    CY_FX_DBG_FUNC_SIP_LNK1_DATA_IOS_OFF      = 0x1023,         /**< Data IOs of LVDS/LVCMOS link 1 switched off */
    CY_FX_DBG_FUNC_SIP_LNK1_CTRL_IOS_OFF      = 0x1024,         /**< Control IOs of LVDS/LVCMOS link 1 switched off */
    CY_FX_DBG_FUNC_SIP_LNK1_L1_STATE          = 0x1025,         /**< LVDS/LVCMOS link 1 in L1 state. */
    CY_FX_DBG_FUNC_SIP_LNK1_L3_STATE          = 0x1026          /**< LVDS/LVCMOS link 1 in L3 state. */
} cy_en_fxusb_dbg_func_sel_t;

/*******************************************************************************
 * Function name: Cy_USB_SelectDFTFunctions
 ****************************************************************************//**
 *
 * Function used to select the Design-For-Test debug functions which are to
 * be driven on to the P11.0 (K7), P11.1 (K8), P9.2 (J7) and P9.3 (J8) pins
 * of the EZ-USB FX device. The Cy_USB_ConfigureIOs function must have
 * been called with dftEnable parameter set to true for these options to take
 * effect.
 *
 * \param dft0_func
 * Select the debug function to be driven on the P11.0 (K7) pin. Any USB/LVDS function
 * is supported.
 * \param dft1_func
 * Select the debug function to be driven on the P11.1 (K8) pin. Any USB/LVDS function
 * is supported.
 * \param usbdft0_func
 * Select the debug function to be driven on the P9.2 (J7) pin. Only USB functions
 * are supported.
 * \param usbdft1_func
 * Select the debug function to be driven on the P9.3 (J8) pin. Only USB functions
 * are supported.
 *
 * \return
 * true if DFT configuration is done, false in case of error.
 *******************************************************************************/
bool Cy_USB_SelectDFTFunctions (
        uint32_t dft0_func,
        uint32_t dft1_func,
        uint32_t usbdft0_func,
        uint32_t usbdft1_func)
{
    uint32_t mainSel = 0;
    uint32_t usbSel  = 0, usbGpioSel = 0;
    uint32_t phySel  = 0, phyRxSel = 0;
    uint32_t lvdsSel = 0;

    if (dft0_func != CY_FX_DBG_FUNC_NONE) {
        if (dft0_func >= CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE) {
            mainSel |= 0x0002UL;
            lvdsSel = ((dft0_func - 0x1000UL) & 0x0FFFUL);
        } else {
            if (dft0_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
                usbSel |= ((dft0_func - 0x0200UL) & 0x00FFUL);
            } else {
                if (dft0_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (dft0_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((dft0_func - 0x0180UL) & 0x000FUL));
                    } else {
                        phySel |= ((dft0_func - 0x0100UL) & 0x3FUL);
                    }
                } else {
                    if (dft0_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((dft0_func - 0x0080UL) & 0x000FUL));
                    } else {
                        phySel |= (dft0_func & 0x3FUL);
                    }
                }
            }
        }
    }

    if (dft1_func != CY_FX_DBG_FUNC_NONE) {
        if (dft1_func >= CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE) {
            mainSel |= 0x0300UL;
            lvdsSel |= (((dft1_func - 0x1000UL) & 0x0FFFUL) << 15U);
        } else {
            mainSel |= 0x0100UL;

            if (dft1_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
                usbSel |= (((dft1_func - 0x0200UL) & 0x00FFUL) << 8U);
            } else {
                usbSel |= 0x0100UL;
                if (dft1_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (dft1_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((dft1_func - 0x0180UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= (((dft1_func - 0x0100UL) & 0x3FUL) << 8U);
                    }
                } else {
                    if (dft1_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((dft1_func - 0x0080UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= ((dft1_func & 0x3FUL) << 8U);
                    }
                }
            }
        }
    }

    if ((usbdft0_func != CY_FX_DBG_FUNC_NONE) && (usbdft0_func < CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE)) {
        if (usbdft0_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
            usbGpioSel |= ((usbdft0_func - 0x0200UL) & 0x00FFUL);
        } else {
            if ((phySel & 0x00FFUL) != 0) {
                /* Too many USB PHY signals requested. Cannot route. */
                return false;
            } else {
                if (usbdft0_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (usbdft0_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((usbdft0_func - 0x0180UL) & 0x000FUL));
                    } else {
                        phySel |= ((usbdft0_func - 0x0100UL) & 0x3FUL);
                    }
                } else {
                    if (usbdft0_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x0028UL;
                        phyRxSel |= (0x8000UL | ((usbdft0_func - 0x0080UL) & 0x000FUL));
                    } else {
                        phySel |= (usbdft0_func & 0x3FUL);
                    }
                }
            }
        }
    }

    if ((usbdft1_func != CY_FX_DBG_FUNC_NONE) && (usbdft1_func < CY_FX_DBG_FUNC_SIP_LNK0_TRAINING_DONE)) {
        if (usbdft1_func >= CY_FX_DBG_FUNC_USBSS_EG_SCK0_ACTIVE) {
            usbGpioSel |= (((usbdft1_func - 0x0200UL) & 0x00FFUL) << 8U);
        } else {
            if ((phySel & 0xFF00UL) != 0) {
                /* Too many USB PHY signals requested. Cannot route. */
                return false;
            } else {
                usbGpioSel |= 0x0100UL;
                if (usbdft1_func >= CY_FX_DBG_FUNC_USBPHY1_RXEQ_TRAINING) {
                    if (usbdft1_func >= CY_FX_DBG_FUNC_USBPHY1_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((usbdft1_func - 0x0180UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= (((usbdft1_func - 0x0100UL) & 0x3FUL) << 8U);
                    }
                } else {
                    if (usbdft1_func >= CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT) {
                        phySel   |= 0x2900UL;
                        phyRxSel |= (0x8000UL | (((usbdft1_func - 0x0080UL) & 0x000FUL) << 8U));
                    } else {
                        phySel |= ((usbdft1_func & 0x3FUL) << 8U);
                    }
                }
            }
        }
    }

    /* Apply the DDFT settings. */
    MAIN_REG->DDFT_MUX                          = mainSel;
    USB32DEV->USB32DEV_MAIN.DDFT_MUX            = usbSel;
    USB32DEV->USB32DEV_MAIN.GPIO_DDFT_MUX       = usbGpioSel;
    LVDSSS_LVDS->DDFT_MUX_SEL                   = lvdsSel;

    USB32DEV->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.PHYSS_DDFT_MUX_SEL = phySel;
    USB32DEV->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.PHYSS_DDFT_MUX_SEL = phySel;
    USB32DEV->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_RX.RX_DTEST_CFG        = phyRxSel;
    USB32DEV->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_RX.RX_DTEST_CFG        = phyRxSel;

    return true;
}

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

#if (FORCE_USBHS)
    Cy_USBD_ConnectDevice(pAppCtxt->pUsbdCtxt, CY_USBD_USB_DEV_HS);
#else
    Cy_USBD_ConnectDevice(pAppCtxt->pUsbdCtxt, USBSS_TGT_SPEED);
#endif /* (FORCE_USBHS) */

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
     * DDFT1      (P11.1): U3
     * GPIO_DDFT0 (P9.2) : RX_LFPS from PHY RX
     * GPIO_DDFT1 (P9.3) : TX_LFPS from PHY
     */
    Cy_USB_SelectDFTFunctions(CY_FX_DBG_FUNC_USBSS_LTSSM_U0, CY_FX_DBG_FUNC_USBSS_LTSSM_U3,
            CY_FX_DBG_FUNC_USBPHY0_RX_LFPSDET_OUT, CY_FX_DBG_FUNC_USBPHY0_TX_LFPS_EN);

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
