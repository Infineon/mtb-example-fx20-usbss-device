/***************************************************************************//**
* \file cy_usb_app.c
* \version 1.0
*
* Implements the USB data handling part of the FX10 USB Echo Device application.
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_usb_echo_device.h"
#include "cy_usb_app.h"
#include "cy_debug.h"
#include "cy_usb_app_common.h"
#include "cy_usbhs_dw_wrapper.h"

extern cy_israddress GetEPInDmaIsr(uint8_t epNum);
extern cy_israddress GetEPOutDmaIsr(uint8_t epNum);

__attribute__ ((section(".descSection"), used)) uint32_t Ep0TestBuffer[1024U] __attribute__ ((aligned (32)));
__attribute__ ((section(".descSection"), used)) uint32_t SetSelDataBuffer[8] __attribute__ ((aligned (32)));

/*
 * Function: Cy_USB_AppInit()
 * Description: This function Initializes application related data structures,
 *              register callback and creates queue and task for device
 *              function. Common function for FS/HS/SS.
 * Parameter: cy_stc_usb_app_ctxt_t, cy_stc_usb_usbd_ctxt_t, DMAC_Type
 *            DW_Type, DW_Type, cy_stc_hbdma_mgr_context_t*
 * return: None.
 * Note: This function should be called after USBD_Init()
 */
void
Cy_USB_AppInit (cy_stc_usb_app_ctxt_t *pAppCtxt,
                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
                DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    uint32_t index;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    cy_stc_app_endp_dma_set_t *pEndpOutDma;
    cy_en_hbdma_chn_state_t state;
    cy_en_hbdma_mgr_status_t  mgrStatus;

#if FREERTOS_ENABLE
    BaseType_t status = pdFALSE;
#endif /* FREERTOS_ENABLE */
    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;

    DBG_APP_INFO("Cy_USB_AppInit\r\n");
    /*
     * Initially application sees device speed as USBFS and during set
     * configuration application will update actual device speed.
     */
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->currentAltSetting = 0x00;
    pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;

    for (index = 0x01; index < CY_USB_NUM_ENDP_CONFIGURED; index++) {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        /*
         * 1. check state of hbDmaChannel and destroy if required.
         * 2. Memset required data structure.
         */
        state = Cy_HBDma_Channel_GetChannelState(&(pEndpInDma->hbDmaChannel));
        if (state != CY_HBDMA_CHN_NOT_CONFIGURED) {
            mgrStatus =
                    Cy_USB_AppDestroyHbDmaChannel(&(pEndpInDma->hbDmaChannel));
            if (mgrStatus != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("Cy_USB_AppInit failed 0x%x\r\n", mgrStatus);
                return;
            }
        }
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
        pEndpInDma->channel = index;
        pEndpInDma->firstRqtDone = false;

        /* Time to handle OUT endpoints. */
        pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
        state = Cy_HBDma_Channel_GetChannelState(&(pEndpOutDma->hbDmaChannel));
        if (state != CY_HBDMA_CHN_NOT_CONFIGURED) {
            mgrStatus =
                Cy_USB_AppDestroyHbDmaChannel(&(pEndpOutDma->hbDmaChannel));
            if (mgrStatus != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("Cy_USB_AppInit failed 0x%x\r\n", mgrStatus);
                return;
            }
        }
        memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
        pEndpOutDma->channel = index;
        pEndpOutDma->firstRqtDone = false;
    }

    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;
    pAppCtxt->dataXferIntrEnabled = false;
    pAppCtxt->hbChannelCreated = false;

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone)) {
        /*
         * Initialize echo device only once and this includes creating
         * thread and queue.
         */
        DBG_APP_INFO("EchoDeviceInit\r\n");
        pAppCtxt->pDevFuncCtxt = Cy_USB_EchoDeviceInit();

#if FREERTOS_ENABLE
        /* create queue and register it to kernel. */
        pAppCtxt->xQueue = xQueueCreate(CY_USB_ECHO_DEVICE_MSG_QUEUE_SIZE,
                                        CY_USB_ECHO_DEVICE_MSG_SIZE);
        DBG_APP_INFO("createdEchoQueue\r\n");
        vQueueAddToRegistry(pAppCtxt->xQueue, "EchoDeviceMsgQueue");

        /* Create task and check status to confirm task created properly. */
        status = xTaskCreate(Cy_USB_EchoDeviceTaskHandler, "EchoDeviceTask", 2048,
                        (void *)pAppCtxt, 5, &(pAppCtxt->echoDevicetaskHandle));
        if (status != pdPASS) {
            DBG_APP_ERR("TaskcreateFail\r\n");
            return;
        }
#else
        UsbConnectionEnable(pAppCtxt);
#endif /* FREERTOS_ENABLE */
        pAppCtxt->firstInitDone = 0x01;
        
    }

    /* Zero out the EP0 test buffer. */
    memset ((uint8_t *)Ep0TestBuffer, 0, sizeof(Ep0TestBuffer));

    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppRegisterCallback()
 * Description: This function will register all calback with USBD layer. 
 * Parameter: cy_stc_usb_app_ctxt_t.
 * return: None.
 */
void
Cy_USB_AppRegisterCallback (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET, 
                                               Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE, 
                                           Cy_USB_AppBusResetDoneCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED, 
                                               Cy_USB_AppBusSpeedCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP, 
                                                  Cy_USB_AppSetupCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND, 
                                                Cy_USB_AppSuspendCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME, 
                                                 Cy_USB_AppResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG, 
                                                 Cy_USB_AppSetCfgCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF, 
                                                Cy_USB_AppSetIntfCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_SLEEP,
                                                Cy_USB_AppL1SleepCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_RESUME,
                                                Cy_USB_AppL1ResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_ZLP,
                                                Cy_USB_AppZlpCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP,
                                                Cy_USB_AppSlpCallback);
    return;
}   /* end of function. */



/*
 * Function: Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig()
 * Description: It initializes DMA descriptor for endpoint 0 IN transfer.
 * Parameter: cy_stc_dma_descriptor_config_t, pSrcAddr, pDstAddr, endpSize,
 *            endpDirection.
 * return: None.
 */
void 
Cy_USB_AppInitEndpCpuDmaDscrConfig (
                      cy_stc_dma_descriptor_config_t *pEndpCpuDmaDscrConfig,
                      uint32_t *pSrcAddr, uint32_t *pDstAddr, uint32_t endpSize,
                      cy_en_usb_endp_dir_t endpDirection)
{
    pEndpCpuDmaDscrConfig->retrigger = CY_DMA_WAIT_FOR_REACT;
    pEndpCpuDmaDscrConfig->interruptType = CY_DMA_DESCR_CHAIN;
    pEndpCpuDmaDscrConfig->triggerOutType = CY_DMA_DESCR_CHAIN;
    pEndpCpuDmaDscrConfig->triggerInType = CY_DMA_DESCR_CHAIN;
    pEndpCpuDmaDscrConfig->dataSize = CY_DMA_WORD;
    pEndpCpuDmaDscrConfig->srcTransferSize = CY_DMA_TRANSFER_SIZE_DATA;
    pEndpCpuDmaDscrConfig->dstTransferSize = CY_DMA_TRANSFER_SIZE_DATA;
    pEndpCpuDmaDscrConfig->descriptorType = CY_DMA_1D_TRANSFER;
    pEndpCpuDmaDscrConfig->srcAddress = pSrcAddr;
    pEndpCpuDmaDscrConfig->dstAddress = pDstAddr;
    pEndpCpuDmaDscrConfig->srcXincrement = 1;
    pEndpCpuDmaDscrConfig->dstXincrement = 1;
    pEndpCpuDmaDscrConfig->xCount = (endpSize >> 2U);   /* Reduce to number of words. */
    pEndpCpuDmaDscrConfig->srcYincrement = 0; /* Not required for 1D transfer. */
    pEndpCpuDmaDscrConfig->dstYincrement = 0; /* Not required for 1D transfer. */
    pEndpCpuDmaDscrConfig->yCount = 0;        /* For 1D transfer we dont need this. */
    pEndpCpuDmaDscrConfig->nextDescriptor = NULL;
    pEndpCpuDmaDscrConfig->channelState = CY_DMA_CHANNEL_DISABLED;
    return;
}   /* End of function  */

/*******************************************************************************
* Function name: Cy_USB_AppSetupEndpDmaParamsSs
****************************************************************************//**
*
* This Function will setup Endpoint and DMA related parameters for SS device
* before transfer initiated.
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
static void
Cy_USB_AppSetupEndpDmaParamsSs (cy_stc_usb_app_ctxt_t *pUsbApp,
                                uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t  mgrStatus;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint32_t endpNum, endpDir;
    uint16_t maxPktSize;
    uint8_t *pCompDscr;
    uint8_t  burstSize;
    uint8_t  endpAddr;

    DBG_APP_TRACE("AppSetupEndpDmaParamsSs >> \r\n");
    Cy_USB_AppConfigureEndp(pUsbApp->pUsbdCtxt, pEndpDscr);

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNum, &maxPktSize, &endpDir);
    pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbApp->pUsbdCtxt, pEndpDscr);
    Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);
    if (endpDir) {
        endpAddr = CY_USBD_ENDP_DIR_MASK | endpNum;
    } else {
        endpAddr = endpNum;
    }

    DBG_APP_TRACE("endpDmaParamsSS endpAddr:0x%x maxPktSize:0x%x burstSize:0x%x\r\n",
                  endpAddr, maxPktSize, burstSize);
    dmaConfig.size           = (maxPktSize * burstSize);   /* DMA Buffer Sise in bytes */
    dmaConfig.prodBufSize    = (maxPktSize * burstSize);
    dmaConfig.count          = 2;                          /* DMA Buffer Count */
    dmaConfig.bufferMode     = false;                      /* DMA buffer mode disabled */
    dmaConfig.prodHdrSize    = 0;
    dmaConfig.prodSckCount   = 1;                          /* No. of producer sockets */
    dmaConfig.consSckCount   = 1;                          /* No. of consumer Sockets */
    dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
    dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
    dmaConfig.eventEnable    = 0;                          /* No events to be sent. */
    dmaConfig.cb             = HbDma_Cb;                   /* HB-DMA callback */
    dmaConfig.userCtx        = (void *)(pUsbApp);          /* Pass the application context as user context. */
    dmaConfig.endpAddr       = endpAddr;                   /* endp address will define association with channel. */

    if (endpDir) {
        /*
         * Its IN endpoint which means DATA device->Host.
         * Make sure endpoint is marked valid.
         */
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNum]);
        pEndpDmaSet->maxPktSize = maxPktSize;
        pEndpDmaSet->valid      = 0x01;

        /* Create channel which will move data from SRAM to USB endpoint. */
        dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;    /* DMA Channel type: from HB-RAM to USB3-IP */
        dmaConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;
        dmaConfig.consSck[0]     =
        (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNum);

        dmaConfig.intrEnable     =
        (USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_CONSUME_EVENT_Msk |
        USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk);

        mgrStatus = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                                          &(pUsbApp->endpInDma[endpNum].hbDmaChannel),
                                          &dmaConfig);

        if (mgrStatus != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("HBDmaInChannel create failed 0x%x\r\n", mgrStatus);
            return;
        }
    } else {
        /*
         * Its OUT endpoint which means data Host->Device.
         * Make sure endpoint is marked valid.
         */
        pEndpDmaSet = &(pUsbApp->endpOutDma[endpNum]);
        pEndpDmaSet->maxPktSize = maxPktSize;
        pEndpDmaSet->valid      = 0x01;

        /* Create channel which moves data from USB ingress endpoint into HBW SRAM. */
        dmaConfig.chType         = CY_HBDMA_TYPE_IP_TO_MEM;
        dmaConfig.prodSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + endpNum);
        dmaConfig.consSck[0]     = CY_HBDMA_VIRT_SOCKET_RD;
        dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_PRODUCE_EVENT_Msk |
                                   USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk;
        mgrStatus = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                                            &(pUsbApp->endpOutDma[endpNum].hbDmaChannel),
                                            &dmaConfig);
        if (mgrStatus != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("HBDmaOutChannel create failed 0x%x\r\n", mgrStatus);
            return;
        }
    }

    /* Set flag to indicate DMA channels have been created. */
    pUsbApp->hbChannelCreated = true;
    DBG_APP_TRACE("AppSetupEndpDmaParamsSs << \r\n\r\n");
    return;
}


/*******************************************************************************
* Function name: Cy_USB_AppSetupEndpDmaParamsHs
****************************************************************************//**
*
* This Function will setup Endpoint and DMA related parameters for high speed
* device before transfer initiated.
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
static void
Cy_USB_AppSetupEndpDmaParamsHs (cy_stc_usb_app_ctxt_t *pUsbApp,
                                uint8_t *pEndpDscr)
{
    DW_Type *pDW;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint32_t endpNum, channelNum;
    uint16_t maxPktSize = 0x00;
    bool stat;
    cy_en_usb_endp_dir_t endpDir;

    DBG_APP_TRACE("Cy_USB_AppSetupEndpDmaParamsHs >> \r\n");

    /* Configure endpoint in USB-IP */
    Cy_USB_AppConfigureEndp(pUsbApp->pUsbdCtxt, pEndpDscr);

    endpNum = ((*(pEndpDscr+CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);
    Cy_USBD_GetEndpMaxPktSize(pEndpDscr, &maxPktSize);

    channelNum = endpNum;
    if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & CY_USBD_ENDP_DIR_MASK) {
        endpDir = CY_USB_ENDP_DIR_IN;
        pEndpDmaSet   = &(pUsbApp->endpInDma[endpNum]);
        pDW           = pUsbApp->pCpuDw1Base;
    } else {
        endpDir = CY_USB_ENDP_DIR_OUT;
        pEndpDmaSet   = &(pUsbApp->endpOutDma[endpNum]);
        pDW           = pUsbApp->pCpuDw0Base;
    }

    stat = Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, pDW, channelNum,
                                       endpNum, endpDir, maxPktSize);
    DBG_APP_TRACE("Enable EPDmaSet: endp=%x dir=%x stat=%x\r\n",
                  endpNum, endpDir, stat);

    pEndpDmaSet->endpType = (cy_en_usb_endp_type_t)
                    ((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ATTRIBUTE)) & 0x03);

    /* Make the ISR registration and trigger connections from EPM to DMAC. */
    if (endpDir == CY_USB_ENDP_DIR_IN) {
        DBG_APP_TRACE("DIR-IN, ChannelNum:0x%x\r\n",pEndpDmaSet->channel);
        Cy_USB_AppInitCpuDmaIntr(endpNum, CY_USB_ENDP_DIR_IN,
                                 GetEPInDmaIsr(endpNum));
    } else {
        DBG_APP_TRACE("DIR-OUT, ChannelNum:0x%x\r\n",pEndpDmaSet->channel);
        Cy_USB_AppInitCpuDmaIntr(endpNum, CY_USB_ENDP_DIR_OUT,
                                 GetEPOutDmaIsr(endpNum));
    }

    DBG_APP_TRACE("Cy_USB_AppSetupEndpDmaParamsHs << \r\n");
    return;
}   /* end of function  */

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
void
Cy_USB_AppSetupEndpDmaParams (cy_stc_usb_app_ctxt_t *pUsbApp,
                              uint8_t *pEndpDscr)
{
    if (pUsbApp->devSpeed > CY_USBD_USB_DEV_HS) {
        Cy_USB_AppSetupEndpDmaParamsSs(pUsbApp, pEndpDscr);
    } else {
        Cy_USB_AppSetupEndpDmaParamsHs(pUsbApp, pEndpDscr);
    }
}

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
void
Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNum,
                     uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t      *pEndpDmaSet;
    cy_en_hbdma_mgr_status_t        hbDmaStatus;

    DBG_APP_TRACE("Cy_USB_AppQueueRead >>\r\n");
    DBG_APP_TRACE("pBuffer:0x%x \r\n",pBuffer);

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
       (pAppCtxt->pCpuDw0Base == NULL) || (pBuffer == NULL) ||
       (dataSize == 0)) {

        DBG_APP_ERR("QueueRead: BadParam NULL\r\n");
        return;
    }

    pEndpDmaSet  = &(pAppCtxt->endpOutDma[endpNum]);
    /* If endpoint not valid then dont go ahead. */
    if (pEndpDmaSet->valid == 0) {
        DBG_APP_ERR("QueueRead: EndpSetNotValid\r\n");
        return;
    }

    if (pAppCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        /* Use HBDMA function to queue the read operation. */
        hbDmaStatus = Cy_HBDma_Channel_ReceiveData(&(pEndpDmaSet->hbDmaChannel),
                                                   0, pBuffer, dataSize, NULL);
        if (hbDmaStatus != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("QueueRead: HBDMA error %x\r\n", hbDmaStatus);
        }
    } else {
        /* USB HS-FS data recieve case */
        DBG_APP_TRACE("CALLING Cy_USBHS_App_QueueRead\r\n");
        Cy_USBHS_App_QueueRead(pEndpDmaSet, pBuffer, dataSize);
        /* Update xfer count and then disable NAK for the endpoint. */
        Cy_USBD_UpdateXferCount(pAppCtxt->pUsbdCtxt, endpNum,
                                CY_USB_ENDP_DIR_OUT, dataSize);
        /*
         * When device not ready then it will enable NAK.
         * Now device is ready to recieve data so disable NAK.
         */
        Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNum,
                                        CY_USB_ENDP_DIR_OUT, false);
    }

    pEndpDmaSet->firstRqtDone = true;
    DBG_APP_TRACE("Cy_USB_AppQueueRead << \r\n");
    return;

} /* end of function */

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
uint16_t
Cy_USB_AppReadShortPacket (cy_stc_usb_app_ctxt_t *pAppCtxt,
                           uint8_t endpNum, uint16_t pktSize)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint16_t dataSize = 0;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
        (pAppCtxt->pCpuDw0Base == NULL)) {
        DBG_APP_ERR("ReadSLP: BadParam NULL\r\n");
        return 0;
    }

    pEndpDmaSet  = &(pAppCtxt->endpOutDma[endpNum]);
    /* Verify that the selected endpoint is valid. */
    if (pEndpDmaSet->valid == 0) {
        DBG_APP_ERR("ReadSLP: EndpSetNotValid\r\n");
        return 0;
    }

    dataSize = Cy_USBHS_App_ReadShortPacket(pEndpDmaSet, pktSize);
    return dataSize;
} /* end of function */


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
void
Cy_USB_AppQueueWrite (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNum,
                      uint8_t *pBuffer, uint16_t dataSize)
{

    cy_stc_app_endp_dma_set_t      *pEndpDmaSet;
    cy_en_hbdma_mgr_status_t        hbDmaStatus;

    DBG_APP_TRACE("Cy_USB_AppQueueWrite >>\r\n");

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
       (pAppCtxt->pCpuDw0Base == NULL) || (pBuffer == NULL)) {
        DBG_APP_ERR("QueueWrite: BadParam NULL\r\n");
        return;
    }

    pEndpDmaSet  = &(pAppCtxt->endpInDma[endpNum]);
    /* If endpoint not valid then dont go ahead. */
    if (pEndpDmaSet->valid == 0) {
        DBG_APP_ERR("QueueWrite: EndpSetNotValid\r\n");
        return;
    }

    if (pAppCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        /* Use HBDMA function to queue the write operation. */
        hbDmaStatus = Cy_HBDma_Channel_SendData(&(pEndpDmaSet->hbDmaChannel),
                                                0, pBuffer, dataSize);
        if (hbDmaStatus != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("QueueWrite: HBDMA error %x\r\n", hbDmaStatus);
        }
    } else {
        /* In USB-HS, 0 length packet sent through USB controller */
        if (dataSize == 0)  {
            DBG_APP_ERR("QueueWrite:BadParam:DataSize-0\r\n");
            return;
        }
        DBG_APP_TRACE("CALLING Cy_USBHS_App_QueueWrite\r\n");
        Cy_USBHS_App_QueueWrite(pEndpDmaSet, pBuffer, dataSize);
     }
    DBG_APP_TRACE("Cy_USB_AppQueueWrite <<\r\n");
} /* end of function */

void
HbDma_Cb (cy_stc_hbdma_channel_t *pHandle, cy_en_hbdma_cb_type_t type,
          cy_stc_hbdma_buff_status_t* pbufStat, void *userCtx)
{
    cy_stc_usb_app_ctxt_t     *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;
    cy_en_hbdma_mgr_status_t   hbDmaStatus;
    cy_stc_hbdma_buff_status_t bufStatus;
    uint8_t endpAddr;
    uint8_t dir;

    (void)hbDmaStatus;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif /* FREERTOS_ENABLE */
    cy_stc_usbd_app_msg_t xMsg;

    endpAddr = pHandle->endpAddr;
    dir = (endpAddr & CY_USBD_ENDP_DIR_MASK);
    xMsg.type = 0;
    DBG_APP_TRACE("HbDma_Cb >>\r\n");

    switch (type) {
        case CY_HBDMA_CB_XFER_CPLT:
            DBG_APP_TRACE("HbdmaCb: CY_HBDMA_CB_XFER_CPLT\r\n");
            if (dir == CY_USB_ENDP_DIR_OUT) {
                DBG_APP_TRACE("DIR_OUT\r\n");
                /* OUT transfer completed. Notify the ECHO thread. */
                xMsg.type = CY_USB_ECHO_DEVICE_MSG_READ_COMPLETE;
                xMsg.data[0] = endpAddr;
                xMsg.data[1] = 0;
                DBG_APP_TRACE("HbdmaCb: endpAddr:0x%x\r\n", endpAddr);
            } else {
                DBG_APP_TRACE("DIR_IN\r\n");
                /* IN transfer completed. Notify the ECHO thread. */
                xMsg.type = CY_USB_ECHO_DEVICE_MSG_WRITE_COMPLETE;
                xMsg.data[0] = endpAddr;
                xMsg.data[1] = 0;
                DBG_APP_TRACE("HbdmaCb: endpAddr:0x%x\r\n", endpAddr);
            }
            if (xMsg.type != 0) {
#if FREERTOS_ENABLE
                xQueueSendFromISR(pAppCtxt->xQueue, &xMsg, &xHigherPriorityTaskWoken);
#else
                Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */
            }
            break;

        case CY_HBDMA_CB_PROD_EVENT:
            DBG_APP_TRACE("HbdmaCb: CY_HBDMA_CB_PROD_EVENT\r\n");
            hbDmaStatus = Cy_HBDma_Channel_DiscardBuffer(pHandle, &bufStatus);
            break;

        case CY_HBDMA_CB_CONS_EVENT:
            DBG_APP_TRACE("HbdmaCb: CY_HBDMA_CB_CONS_EVENT\r\n");
            hbDmaStatus = Cy_HBDma_Channel_GetBuffer(pHandle, &bufStatus);
            if (hbDmaStatus == CY_HBDMA_MGR_SUCCESS)  {
                bufStatus.count = bufStatus.size;
                hbDmaStatus = Cy_HBDma_Channel_CommitBuffer(pHandle, &bufStatus);
            } else {
                DBG_APP_ERR("HbDma_Cb: Get status %x\r\n", hbDmaStatus);
            }
            break;
        default:
            DBG_APP_ERR("HbDma_Cb: Default\r\n");
            break;
    }
    DBG_APP_TRACE("HbDma_Cb <<\r\n");
    return;
}   /* end of function  */

cy_stc_hbdma_buff_status_t bufStat;


#if FREERTOS_ENABLE
/*
 * Function: Cy_USB_RecvEndp0TimerCallback()
 * Description: This Function will be called when timer expires.
 *              This function posts TIMER Expiry message to ECHO device.
 * Parameter: xTimer
 * return: void
 */
void
Cy_USB_RecvEndp0TimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("TIMER CALLBACK called!!\r\n");
    /* retrieve pAppCtxt and post message to echo device */
    pAppCtxt = ( cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);

    xMsg.type = CY_USB_ENDP0_READ_TIMEOUT;
    status = xQueueSendFromISR(pAppCtxt->xQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    if (status != pdPASS) {
        DBG_APP_INFO("AppMsgSETUPSndFail\r\n");
    }
    return;
}
#endif /* FREERTOS_ENABLE */


/*
 * Function: Cy_USB_AppSetCfgCallback()
 * Description: This Function will be called by USBD  layer when 
 *              set configuration command successful. This function 
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppSetCfgCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_stc_usbd_app_msg_t xMsg;
    cy_en_usb_speed_t devSpeed;
    cy_en_usb_app_ret_code_t retCode;

#if FREERTOS_ENABLE
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif /* FREERTOS_ENABLE */

    DBG_APP_TRACE("Cy_USB_AppSetCfgCallback >>\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    pUsbApp->devSpeed = devSpeed;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;

    retCode = Cy_USB_AppHandleSetCfgCommon(pUsbApp, pUsbdCtxt, pMsg);
    if (retCode == CY_USB_APP_STATUS_FAILURE) {
        DBG_APP_TRACE("Cy_USB_ApphandleSetCfgCommon Failed\r\n");
        return;
    }
    /* specific to src-snk and loopback function. */
    Cy_DevSpeedBasedfxQueueUpdate(pAppCtxt, devSpeed);

#if FREERTOS_ENABLE
    /* Set timer expiry to 5 second*/
    pUsbApp->endp0OuttimerExpiry = 5000;
    /* Create timer with 5Sec Expiry and recurring = FALSE. */
    pUsbApp->endp0OutTimerHandle =
        xTimerCreate("Endp0OutTimer", pUsbApp->endp0OuttimerExpiry, pdFALSE,
                     (void *)pUsbApp, Cy_USB_RecvEndp0TimerCallback);
#endif /* FREERTOS_ENABLE */
    /*
     * first send message to configure channel and then transfer data.
     * can be optimized with single message later.
     */

    DBG_APP_TRACE("send CY_USB_ECHO_DEVICE_MSG_SETUP_DATA_XFER\r\n");
    xMsg.type = CY_USB_ECHO_DEVICE_MSG_SETUP_DATA_XFER;
#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pUsbApp->xQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    if (status != pdPASS) {
        DBG_APP_ERR("AppMsgSETUPSndFail\r\n");
    }
#else
    Cy_USB_EchoDeviceTaskHandler(pUsbApp, &xMsg);
#endif /* FREERTOS_ENABLE */

    DBG_APP_TRACE("send CY_USB_ECHO_DEVICE_MSG_START_DATA_XFER\r\n");
    xMsg.type = CY_USB_ECHO_DEVICE_MSG_START_DATA_XFER;
#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pUsbApp->xQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    if (status != pdPASS) {
        DBG_APP_ERR("AppMsgSTARTSndFail\r\n");
    }
#else
    Cy_USB_EchoDeviceTaskHandler(pUsbApp, &xMsg);
#endif /* FREERTOS_ENABLE */

#if (!USB3_LPM_ENABLE)
    /* Explicitly disable L1 entry */
    DBG_APP_INFO("Disabling LPM\r\n");
    Cy_USBD_LpmDisable(pUsbApp->pUsbdCtxt);
#else
    /* Enable L1 entry */
    DBG_APP_INFO("Enabling LPM\r\n");
    Cy_USBD_LpmEnable(pUsbApp->pUsbdCtxt);
#endif /* (!USB3_LPM_ENABLE) */

    /*
     * Register DMA transfer completion interrupt for OUT
     * endpoint 0 and enable respective mask also.
     */
    DBG_APP_TRACE("RegisterDmaIntr for endp0-OUT\r\n");
    Cy_USB_AppInitCpuDmaIntr(CY_USB_ENDP_0, CY_USB_ENDP_DIR_OUT,
                             GetEPOutDmaIsr(CY_USB_ENDP_0));
    Cy_DMAC_Channel_SetInterruptMask(pUsbApp->pCpuDmacBase,
                                     pUsbApp->pUsbdCtxt->channel1,
                                     CY_DMAC_INTR_COMPLETION);
#if 0
        /* On USB 2.x connection, we can enable LPM-L1 transitions after
         * the device has been configured. Activate this code whenever
         * L1 need to be enabled.
         */
        if (pUsbApp->devSpeed <= CY_USBD_USB_DEV_HS) {
            DBG_USBD_INFO("Calling LPM Enable\r\n");
            Cy_USBD_LpmEnable(pUsbApp->pUsbdCtxt);
        }
#endif

    DBG_APP_TRACE("Cy_USB_AppSetCfgCallback <<\r\n\r\n");
    return;
}   /* end of function */


/*
 * Function: Cy_USB_AppBusResetCallback()
 * Description: This Function will be called by USBD when bus detects RESET.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void 
Cy_USB_AppBusResetCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("ResetCallback >>\r\n");

    /* NT TBD: comeback and review code.
     * Stop and destroy the high bandwidth DMA channel if present.
     * To be done before AppInit is called.
     */
    //Cy_USB_AppHbDmaDisableEndpDmaSetAll(pUsbApp);
    
    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase,
                   pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base,
                   pUsbApp->pHbDmaMgrCtxt);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;

    DBG_APP_INFO("ResetCallback <<\r\n\r\n");
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppBusResetDoneCallback()
 * Description: This Function will be called by USBD  layer when 
 *              set configuration command successful. This function 
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void 
Cy_USB_AppBusResetDoneCallback (void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppBusSpeedCallback()
 * Description: This Function will be called by USBD  layer when
 *              speed is identified or speed change is detected. 
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void 
Cy_USB_AppBusSpeedCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppSetupCallback()
 * Description: This Function will be called by USBD  layer when 
 *              set configuration command successful. This function 
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
volatile uint16_t pktType = 0;
volatile uint16_t pktLength = 1024;

void 
Cy_USB_AppSetupCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
#if FREERTOS_ENABLE
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t status;

    DBG_APP_INFO("SetupCb\r\n");
    /*
     * cy_stc_usbd_app_msg_t xMsg;
     * Send message to setup and start data transfer.
     */
    xMsg.type = CY_USB_ECHO_DEVICE_MSG_CTRL_XFER_SETUP;
    xMsg.data[0] = pMsg->data[0];
    xMsg.data[1] = pMsg->data[1];

#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pUsbApp->xQueue, &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
     Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */
    (void)status;

}   /* end of function. */

/*
 * Function: Cy_USB_AppSuspendCallback()
 * Description: This Function will be called by USBD  layer when 
 *              Suspend signal/message is detected. 
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void 
Cy_USB_AppSuspendCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
}   /* end of function. */

/*
 * Function: Cy_USB_AppResumeCallback()
 * Description: This Function will be called by USBD  layer when 
 *              Resume signal/message is detected.  
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void 
Cy_USB_AppResumeCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_en_usb_device_state_t tempState;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    tempState =  pUsbApp->devState;
    pUsbApp->devState = pUsbApp->prevDevState;
    pUsbApp->prevDevState = tempState;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppSetIntfCallback()
 * Description: This Function will be called by USBD  layer when 
 *              set interface is called.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void 
Cy_USB_AppSetIntfCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t intfNum, newAltSetting, numOfAltSetting;
    int8_t numOfEndp;
    uint8_t *pIntfDscr, *pEndpDscr;
    BaseType_t status;
    cy_stc_usbd_app_msg_t xMsg;

#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    DBG_APP_INFO("Cy_USB_AppSetIntfCallback >> \r\n");

    pSetupReq = &(pUsbdCtxt->setupReq);
    /*
     * Get interface and alt setting info. If new setting same as previous
     * then take action as per spec.
     * If new alt setting came then first Unconfigure previous settings
     * and then configure new settings.
     */
    intfNum = pSetupReq->wIndex;
    newAltSetting = pSetupReq->wValue;

    DBG_APP_INFO("Intf:%d currAltSet:%d NewAltSet:%d, \r\n", intfNum,
                 pUsbApp->currentAltSetting, newAltSetting);

    numOfAltSetting = Cy_USBD_GetNumOfAltSetting(pUsbdCtxt, intfNum);
    if (newAltSetting == pUsbApp->currentAltSetting) {
        if (numOfAltSetting == 0x01) {
            /*
             * Set interface request for same alt setting and only default is
             * available then send stall.
             */
            DBG_APP_INFO("SetIntf:SameAltSetting and only default so send STALL\r\n");
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                          CY_USB_ENDP_DIR_IN, TRUE);
        }
        DBG_APP_INFO("Cy_USB_AppSetIntfCallback << \r\n\r\n");
        return;
    }

    /* New altSetting is different than previous one so unconfigure previous. */
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, pUsbApp->currentAltSetting);
    if (pIntfDscr == NULL) {
        DBG_APP_ERR("pIntfDscr-Null\r\n");
        return;
    }

    /* first send stop data transfer message to stop data transfer. */
    xMsg.type = CY_USB_ECHO_DEVICE_MSG_STOP_DATA_XFER;
#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pUsbApp->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pUsbApp, &xMsg);
#endif /* FREERTOS_ENABLE */

    /* Now unconfigure things related to previous altSetting*/
    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    DBG_APP_INFO("NumOfEndp=%d in CurrIntf\r\n",numOfEndp);
    if (numOfEndp == 0x00) {
        /*
         * If previous interface does not have endpoint so no need to
         * unconfigure it. Just move to new setting.
         */
        DBG_APP_INFO("SetIntf:prevIntNumEp=0\r\n");
    } else {
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);

        while (numOfEndp != 0x00) {
            Cy_USB_AppDestroyEndpDmaParams(pUsbApp, pEndpDscr);
            numOfEndp--;
            if (pUsbApp->devSpeed > CY_USBD_USB_DEV_HS) {
                /* Take care companion dscr for SS while pointer movement */
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)) + 6);
            } else {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            }
        }   /* End of while */
    }

    DBG_APP_INFO("Prev AltSet Unconfigured....... \r\n");

    /* Flush all EPM contents at this stage. */
    Cy_USBD_FlushEndpAll(pUsbdCtxt, CY_USB_ENDP_DIR_IN);

    /* Now take care of different config with new alt setting. */
    DBG_APP_INFO("New AltSet ConfigStarts........ \r\n");
    pUsbApp->currentAltSetting = newAltSetting;
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, newAltSetting);
    if (pIntfDscr == NULL) {
        DBG_APP_INFO("SetIntf:pIntfDscrNull\r\n");
        return;
    }

    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    pUsbApp->intfAltSetEndp[intfNum][newAltSetting] = numOfEndp;
    if (numOfEndp == 0x00) {
        DBG_APP_INFO("SetIntf:numEp 0\r\n");
    } else {
        pUsbApp->currentAltSetting = newAltSetting;
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            //Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParams(pAppCtxt, pEndpDscr);
            numOfEndp--;
            if(pUsbApp->devSpeed > CY_USBD_USB_DEV_HS) {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)) + 6);
            } else {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            }
        }
    }

    /*
     * Send message to setup and start data transfer.
     */
    xMsg.type = CY_USB_ECHO_DEVICE_MSG_SETUP_DATA_XFER;
#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pUsbApp->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pUsbApp, &xMsg);
#endif /* FREERTOS_ENABLE */

    xMsg.type = CY_USB_ECHO_DEVICE_MSG_START_DATA_XFER;
#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pUsbApp->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pUsbApp, &xMsg);
#endif /* FREERTOS_ENABLE */
    (void)status;

    DBG_APP_INFO("Cy_USB_AppSetIntfCallback << \r\n\r\n");
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppZlpCallback()
 * Description: This Function will be called by USBD layer when
 *              ZLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppZlpCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;

    /*
     * Get cal MSG.
     * Convert CAL msg to usbd/app message.
     * send it through queue.
     */
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    if (pMsg->type == CY_USB_CAL_MSG_OUT_ZLP) {
        xMsg.type = CY_USB_ECHO_DEVICE_MSG_ZLP_OUT;
    } else {
        xMsg.type = CY_USB_ECHO_DEVICE_MSG_ZLP_IN;
    }
    /* Just make sure data also copied which contains endp addr and dir. */
    xMsg.data[0] = pMsg->data[0];
    xMsg.data[1] = pMsg->data[1];

#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;

    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppL1SleepCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Sleep message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppL1SleepCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_TRACE("AppL1SleepCb\r\n");
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    xMsg.type = CY_USB_ECHO_DEVICE_MSG_L1_SLEEP;
    xMsg.data[0] = 0x00;
    xMsg.data[1] = 0x00;

#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppL1ResumeCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Resume message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppL1ResumeCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_TRACE("AppL1ResumeCb\r\n");
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    xMsg.type = CY_USB_ECHO_DEVICE_MSG_L1_RESUME;
    xMsg.data[0] = 0x00;
    xMsg.data[1] = 0x00;

#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppSlpCallback()
 * Description: This Function will be called by USBD layer when 
 *              SLP message comes.  
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSlpCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_TRACE("AppSlpCb\r\n");

    /*
     * Get cal MSG.
     * Convert CAL msg to usbd/app message.
     * send it through queue.
     */
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    if (pMsg->type == CY_USB_CAL_MSG_OUT_SLP) {
        xMsg.type = CY_USB_ECHO_DEVICE_MSG_SLP_OUT;
    } else {
        xMsg.type = CY_USB_ECHO_DEVICE_MSG_SLP_IN;
    }
    xMsg.data[0] = pMsg->data[0];
    xMsg.data[1] = pMsg->data[1];

#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppSetFeatureCallback()
 * Description: This Function will be called by USBD layer when
 *              set feature message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSetFeatureCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("AppSetFeatureCb\r\n");

    /*
     * Get cal MSG.
     * Convert CAL msg to usbd/app message.
     * send it through queue.
     */
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    xMsg.type = CY_USB_ECHO_DEVICE_SET_FEATURE;
    xMsg.data[0] = pMsg->data[0];
    xMsg.data[1] = pMsg->data[1];
#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppClearFeatureCallback()
 * Description: This Function will be called by USBD layer when
 *              clear feature message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppClearFeatureCallback (void *pUsbApp,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    BaseType_t status;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("AppClearFeatureCb\r\n");

    /*
     * Get cal MSG.
     * Convert CAL msg to usbd/app message.
     * send it through queue.
     */
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    xMsg.type = CY_USB_ECHO_DEVICE_CLEAR_FEATURE;
    xMsg.data[0] = pMsg->data[0];
    xMsg.data[1] = pMsg->data[1];
#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_HandleEchoDeviceReqs()
 * Description: This Function handles all request related to echo device.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_HandleEchoDeviceReqs (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t   bmRequest, bRequest;
    uint16_t wValue, wIndex;
    cy_en_usb_app_ret_code_t retCode = CY_USB_APP_STATUS_FAILURE;

    pSetupReq =  &(pUsbdCtxt->setupReq);

    /* Decode the fields from the setup request. */
    bmRequest = pSetupReq->bmRequest;
    bRequest = pSetupReq->bRequest;

    do {

        if ((bRequest == 0xE0) && (bmRequest == 0x40)) {
            /*
             * Device reset request:
             * 1. Get delay information.
             * 2. Initiate Status stage ACK.
             * 3. Do the reset functionality.
             */
            wValue = pSetupReq->wValue;
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            /* wait for wValue+1 ms  */
            Cy_SysLib_DelayUs(((wValue+1) * 1000));
            NVIC_SystemReset();
            retCode = CY_USB_APP_STATUS_SUCCESS;
            break;
        }

        if ((bRequest == 0xE1) && (bmRequest == 0x40)) {
            /*
             * Device re-enumeration request:
             * 1. Get delay information (wValue) and Speed
             *    information (wIndex).
             * 2. Initiate Status stage ACK.
             * 3. Set Speed and then diconnect and connect device.
             */
            wValue = pSetupReq->wValue;
            wIndex = pSetupReq->wIndex;
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            Cy_SysLib_DelayUs(((wValue+1) * 1000));
            Cy_USBD_DisconnectDevice(pUsbdCtxt);
            Cy_SysLib_DelayUs(((wValue+1) * 1000));
            /*
             * Make sure wIndex value of device speed should match with
             * cy_en_usb_speed_t
             */
            Cy_USBD_SetDeviceSpeed(pUsbdCtxt, (cy_en_usb_speed_t)wIndex);
            Cy_USBD_ConnectDevice(pUsbdCtxt,(cy_en_usb_speed_t)wIndex);
            retCode = CY_USB_APP_STATUS_SUCCESS;
            break;
        }

        if ((bRequest == 0xF0) && (bmRequest == 0xC0)) {
            /* Device capability request */
            /* TBD: Prepare data and then send the same. */
            retCode = CY_USB_APP_STATUS_SUCCESS;
            break;
        }
    } while(0);

    if (retCode == CY_USB_APP_STATUS_FAILURE) {
        /* If request is not handled then need to send STALL */
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                      CY_USB_ENDP_DIR_IN, TRUE);
    }
}   /* end of function. */



