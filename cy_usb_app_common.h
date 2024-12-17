/***************************************************************************//**
* \file cy_usb_app_common.h
* \version 1.0
*
* Provides interface definitions of the functions used to configure USB endpoint
* and DMA resources.
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

#ifndef _CY_USB_APP_COMMON_H_
#define _CY_USB_APP_COMMON_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_debug.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Function name: Cy_USB_AppConfigureEndp
****************************************************************************//**
*
* This Function is used by application to configure endpoints after set
* configuration. This function should be used for all endpoints except endp0.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* None.
*
*******************************************************************************/
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pEndpDscr);

/*******************************************************************************
* Function name: Cy_USB_AppDestroyEndpDmaParams
****************************************************************************//**
*
* This Function will destroy Endpoint and DMA related association.
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
void Cy_USB_AppDestroyEndpDmaParams(cy_stc_usb_app_ctxt_t *pUsbApp,
                                    uint8_t *pEndpDscr);

/*******************************************************************************
* Function name: Cy_USB_AppHbDmaDisableEndpDmaSetAll
****************************************************************************//**
*
* This function de-inits all active USB HBDMA DMA channels.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_USB_AppHbDmaDisableEndpDmaSetAll(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*******************************************************************************
* Function name: Cy_USB_AppHbDmaDestroyEndpDmaSetAll
****************************************************************************//**
*
* This function destroy all active USB DMA channels.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_USB_AppHbDmaDestroyEndpDmaSetAll(cy_stc_usb_app_ctxt_t *pAppCtxt);

/*******************************************************************************
* Function name: Cy_USB_AppDestroyHbDmaChannel
****************************************************************************//**
*
* This function disable and destroy HBW dma channel.
*
* \param pHandle
* pointer to handle associated with high bandwidth DMA channel.
*
* \param pEndpDscr
* pointer to endpoint descriptor.
*
* \return
* The status code of the function execution \ref cy_en_hbdma_mgr_status_t
*
*******************************************************************************/
cy_en_hbdma_mgr_status_t Cy_USB_AppDestroyHbDmaChannel(cy_stc_hbdma_channel_t *pHandle);


/*******************************************************************************
* Function name: Cy_USB_AppTerminateCpuDma
****************************************************************************//**
*
* Function will disable associate central DMA channel.
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
* \return
* None
*
********************************************************************************/
void Cy_USB_AppTerminateCpuDma(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNum,
                               cy_en_usb_endp_dir_t endpDir);

/*******************************************************************************
* Function name: Cy_USB_AppTerminateDma
****************************************************************************//**
*
* Function will disable associate DMA channel.
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
* \return
* None
*
*******************************************************************************/
void Cy_USB_AppTerminateDma(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                            cy_en_usb_endp_dir_t endpDirection);

/*******************************************************************************
* Function name: Cy_USB_AppFindValidInEndpNumber
****************************************************************************//**
*
* Find valid IN endpoint number.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* 0x00 or endpoint number.
*
*******************************************************************************/
uint8_t Cy_USB_AppFindValidInEndpNumber(cy_stc_usb_app_ctxt_t *pUsbApp);

/*******************************************************************************
* Function name: Cy_USB_AppFindValidOutEndpNumber
****************************************************************************//**
*
* Find valid OUT endpoint number.
*
* \param pAppCtxt
* application layer context pointer.
*
* \return
* 0x00 or endpoint number.
*
*******************************************************************************/
uint8_t Cy_USB_AppFindValidOutEndpNumber(cy_stc_usb_app_ctxt_t *pUsbApp);

/*******************************************************************************
* Function name: Cy_USB_AppIsOutEndpValidEnable
****************************************************************************//**
*
* Valid bit is set or not for given OUT endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \return
* TRUE if valid bit is set. FALSE if valid bit is cleared.
*
*******************************************************************************/
bool Cy_USB_AppIsOutEndpValidEnable (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum);

/*******************************************************************************
* Function name: Cy_USB_AppIsInEndpValidEnable
****************************************************************************//**
*
* Valid bit is set or not for given IN endpoint.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \return
* TRUE if valid bit is set. FALSE if valid bit is cleared.
*
*******************************************************************************/
bool Cy_USB_AppIsInEndpValidEnable (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum);

/*******************************************************************************
* Function name: Cy_USB_AppGetMaxPktSize
****************************************************************************//**
*
* Function finds max packet size of an endpoint.
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
* \return
* Max Packet size.
*
*******************************************************************************/
uint32_t Cy_USB_AppGetMaxPktSize(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum,
                                 cy_en_usb_endp_dir_t dir);

/*******************************************************************************
* Function name: Cy_USB_AppInitCpuDmaIntr
****************************************************************************//**
*
* Function to register an ISR for a USB endpoint DMA channel and enable
* the interrupt.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction
*
* \param userIsr
* user provided ISR function pointer.
*
* \return
* None
*
*******************************************************************************/
void Cy_USB_AppInitCpuDmaIntr(uint32_t endpNumber,
                              cy_en_usb_endp_dir_t endpDir,
                              cy_israddress userIsr);

/*******************************************************************************
* Function name: Cy_USB_AppClearCpuDmaInterrupt
****************************************************************************//**
*
* Function to clear the pending DMA interrupt associated with an endpoint.
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
* \return
* None
*
*******************************************************************************/
void Cy_USB_AppClearCpuDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                    uint32_t endpNumber,
                                    cy_en_usb_endp_dir_t endpDirection);

/*******************************************************************************
* Function name: Cy_USB_AppHandleSetCfgCommon
****************************************************************************//**
*
* Function handles common portion of set configuration call back to application.
*
* \param pAppCtxt
* application layer context pointer.
*
* \param pUsbdCtxt
* USBD layer  context pointer.
*
* \param pMsg
* pointer to message coming from lower layer.
*
* \return
* CY_USB_APP_STATUS_SUCCESS in case of set Config handle without error.
* CY_USB_APP_STATUS_FAILURE in all other case.
*
*******************************************************************************/
cy_en_usb_app_ret_code_t Cy_USB_AppHandleSetCfgCommon(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                                      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                      cy_stc_usb_cal_msg_t *pMsg);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_COMMON_H_ */

/* End of File */

