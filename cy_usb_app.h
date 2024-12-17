/***************************************************************************//**
* \file cy_usb_app.h
* \version 1.0
*
* Provides definition used by the FX10 USB Echo Device application.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT           (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN            (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR           (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE               (0u)

/*Related to WINUSB*/
extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;

/* 
 * USB applicatio data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains information about device functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    bool gen2Enabled;                                               /** Whether Gen2 operation is enabled. */
    uint8_t firstInitDone;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    uint8_t currentAltSetting;
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    cy_en_usb_enum_method_t enumMethod;

    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_NUM_ENDP_CONFIGURED];
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_NUM_ENDP_CONFIGURED];
    uint8_t intfAltSetEndp[4][4];    /* Max 4INTF AND EACH INTERFACE 4 alt setting */ 
    /* Next three are related to central DMA */
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;

    /* related to high bandwidth DMA */
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;

    /* Global Task handles */
    TaskHandle_t echoDevicetaskHandle;
    QueueHandle_t xQueue;
    void *pDevFuncCtxt;
    /* Timer functionality for endp0Rcvd */
    TimerHandle_t endp0OutTimerHandle;
    uint32_t endp0OuttimerExpiry;

    bool hbChannelCreated;
    bool dataXferIntrEnabled;
    /* VBus detect status */
    bool vbusPresent;
    bool usbConnected;                                              /** Whether USB connection is enabled. */
    /* USB connection status */
    bool usbConnectDone;
};

/*******************************************************************************
* Function name: Cy_USB_AppSetupEndpDmaParams
****************************************************************************//**
*
* This Function will setup Endpoint and DMA related parameters before transfer
* initiated. Based on current speed, respective function will be called.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* None
*
*******************************************************************************/
void Cy_USB_AppSetupEndpDmaParams(cy_stc_usb_app_ctxt_t *pUsbApp,
                                  uint8_t *pEndpDscr);

/*******************************************************************************
* Function name: Cy_USB_AppQueueRead
****************************************************************************//**
*
*  Function to queue read operation on an OUT endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \param pBuffer
* pointer to buffer where data will be stored.
*
* \param dataSize
* expected data size.
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppQueueRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                         uint8_t *pBuffer, uint16_t dataSize);

/*******************************************************************************
* Function name: Cy_USB_AppReadShortPacket
****************************************************************************//**
*
*  Function to modify an ongoing DMA read operation to take care of a short
*  packet.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param pktSize
* Size of the short packet to be read out. Can be zero in case of ZLP.
*
* \return
* 0x00 or Data size.
*
********************************************************************************/
uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                   uint8_t endpNumber, uint16_t pktSize);

/*******************************************************************************
* Function name: Cy_USB_AppQueueWrite
****************************************************************************//**
*
*  Function to queue write operation on an IN endpoint
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \param pBuffer
* pointer to buffer where data is available.
*
* \param dataSize
* size of data available in buffer.
*
* \return
* None
*
********************************************************************************/
void Cy_USB_AppQueueWrite(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                          uint8_t *pBuffer, uint16_t dataSize);

void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, 
                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
                    DMAC_Type *pCpuDmacBase,
                    DW_Type *pCpuDw0Base,
                    DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USB_RecvEndp0TimerCallback(TimerHandle_t xTimer);
void Cy_USB_AppSetCfgCallback(void *pAppCtxt,
                              cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetCallback(void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt, 
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetupCallback(void *pAppCtxt, 
                             cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSuspendCallback(void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppResumeCallback (void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1SleepCallback(void *pUsbApp,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1ResumeCallback(void *pUsbApp,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);

/* Functions to be provided at the application level to do USB connect/disconnect. */
bool UsbConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);
void UsbConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);
void lightDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);


/* Function to print a string to UART console. */
void Cy_PrintString(const char *string, uint8_t length);

/* Function to print a 32-bit unsigned integer value to the UART console in HEX format. */
void PrintDword(uint32_t data);

/* Function to print an 8-bit unsigned integer value to the UART console in HEX format. */
void Cy_PrintByte(uint8_t data);

/* Function to print the contents of a 8-bit unsigned data buffer to UART console. */
void PrintBuffer(uint8_t *buf_p, uint16_t len);

/* Function to print a register name and value (in HEX format) to the UART console. */
void PrintReg(const char *name, uint8_t namelen, uint32_t value);

void HbDma_Cb(cy_stc_hbdma_channel_t *pHandle,
              cy_en_hbdma_cb_type_t type,
              cy_stc_hbdma_buff_status_t* pbufStat,
              void *userCtx);
void Cy_USB_AppInitEndpCpuDmaDscrConfig(cy_stc_dma_descriptor_config_t *pEndpCpuDmaDscrConfig,
                                        uint32_t *pSrcAddr, uint32_t *pDstAddr, uint32_t endpSize,
                                        cy_en_usb_endp_dir_t endpDirection);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

