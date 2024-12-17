/***************************************************************************//**
* \file cy_usb_echo_device.c
* \version 1.0
*
* Implements the data loopback and source/sink logic for the FX10 USB Echo
* Device application.
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
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usb_echo_device.h"
#include "cy_usb_app.h"
#include "cy_fault_handlers.h"
#include "cy_debug.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_app_common.h"

extern bool UsbConnectionEnable (cy_stc_usb_app_ctxt_t *pAppCtxt);
/*
 * Every device Function should have it's ctxt  and it should
 * go to app context as void *.
 * Void * should be converted to right context in device function
 * related files ie echo_device as one example.
 */
cy_stc_usb_echo_dev_ctxt_t echoDevCtxt;

extern void OutEpDma_ISR(uint8_t endpNumber);
extern void InEpDma_ISR(uint8_t endpNumber);

cy_israddress GetEPInDmaIsr(uint8_t epNum);
cy_israddress GetEPOutDmaIsr(uint8_t epNum);

#if APP_SRC_SNK_EN
unsigned int echodevice = ECHO_DEVICE_SRC_SNK;
#else
unsigned int echodevice = ECHO_DEVICE_LOOPBACK;
#endif /* APP_SRC_SNK_EN */

extern volatile uint16_t pktType;
extern volatile uint16_t pktLength;

extern uint32_t Ep0TestBuffer[1024U];
extern uint32_t SetSelDataBuffer[8];

/*
* Function     : Cy_USB_IsValidMMIOAddr()
* Description :  Check if the passed address is within the valid MMIO address range.
*                Note that this is not an exhaustive check as the MMIO range is not contiguous.
* Parameters  :  const uint32_t address
* Return      :  bool
*/

static bool Cy_USB_IsValidMMIOAddr (const uint32_t address)
{
    uint32_t periLastAddr = 0x700000;

    if((address >= PERI_BASE) && (address < PERI_BASE + periLastAddr))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * Function: Cy_USB_InitializeWriteBuffer()
 * Description: Initialize egress/write buffer to a value.
 * Parameter: pWriteBuffer, len
 * return: void
 */
void 
Cy_USB_InitializeWriteBuffer (uint8_t *pWriteBuffer, uint32_t len)
{
    uint32_t i;

    if (pktType == 0) {
        for (i = 0x00; i < len; i++) {
            *(pWriteBuffer + i) = 0x55;
        }
    }
    else if (pktType == 1) {
        for (i = 0x00; i < len; i++) {
            *(pWriteBuffer + i) = i;
        }
    }
}   /* End of function */

/*
 * Function: Cy_USB_InitializeReadBuffer()
 * Description: Initialize ingress/read buffer to a value.
 * Parameter: pWriteBuffer, len
 * return: void
 */
void 
Cy_USB_InitializeReadBuffer (uint8_t *pReadBuffer, uint32_t len)
{
    uint32_t i;

    for (i = 0x00; i < len; i++) {
        *(pReadBuffer + i) = 0x00;
    }
    return;
}   /* End of function */


extern cy_stc_hbdma_buf_mgr_t HBW_BufMgr;

/*
 * Function: Cy_InitIfxQueue()
 * Description: Initialize queue which is part of echodevice context.
 *              For each endpoint one queue is allocated. Queue for
 *              endpoint 0 should be ignored so always starts for endpoint
 *              one.
 * Parameter: cy_stc_usb_echo_dev_ctxt_t
 * return: void
 */
void
Cy_InitIfxQueue (cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt)
{
    uint32_t endpNum,i;
    Cy_IfxQueue_t *pIfxQueue;

    /*
     * application or echo device should ignore endpoint 0 so index
     * always starts with 0x01
     */
    for(endpNum = 0x01; endpNum < CY_USB_NUM_ENDP_CONFIGURED; endpNum++) {
        pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
        pIfxQueue->readIndex = 0x00;
        pIfxQueue->writeIndex = 0x00;
        pIfxQueue->numElem = 0x00;

        for (i = 0x00; i < CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE; i++) {
            pIfxQueue->elem[i].dataLen = 0x00;
            pIfxQueue->elem[i].pHbData =
            (uint8_t *)Cy_HBDma_BufMgr_Alloc(&HBW_BufMgr,
                                             CY_USB_MAX_DATA_BUFFER_SIZE);
            /* Point to the HS data array by default. */
            pIfxQueue->elem[i].pData = pIfxQueue->elem[i].pHbData;
        }
        /* In src-sink element 0 used for read and element 1 used for write */
        pIfxQueue->pReadBuffer = pIfxQueue->elem[0].pData;
        pIfxQueue->pWriteBuffer = pIfxQueue->elem[1].pData;
        pIfxQueue->readActive = false;
        pIfxQueue->writeActive = false;
        if (echodevice == ECHO_DEVICE_SRC_SNK) {
            /* Initializing data for src/sink device. */
            Cy_USB_InitializeWriteBuffer(pIfxQueue->pWriteBuffer,
                                         CY_USB_MAX_DATA_BUFFER_SIZE);
            Cy_USB_InitializeReadBuffer(pIfxQueue->pReadBuffer,
                                        CY_USB_MAX_DATA_BUFFER_SIZE);
        }
    }
    return;
}   /* End of function */


/*
 * Function: Cy_DevSpeedBasedfxQueueUpdate()
 * Description: Based on speed, need to update different pointer including
 *              data pointer.
 * Parameter: cy_stc_usb_echo_dev_ctxt_t, cy_en_usb_speed_t
 * return: void
 */
void
Cy_DevSpeedBasedfxQueueUpdate (void *pApp,
                               cy_en_usb_speed_t devSpeed)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pApp;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    uint32_t endpNum, index;
    Cy_IfxQueue_t *pIfxQueue;

    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;

    for (endpNum = 0x01; endpNum < CY_USB_NUM_ENDP_CONFIGURED; endpNum++) {
        pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
        pIfxQueue->readIndex = 0x00;
        pIfxQueue->writeIndex = 0x00;
        pIfxQueue->numElem = 0x00;

        for (index = 0x00; index < CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE; index++) {
            pIfxQueue->elem[index].dataLen = 0x00;
            pIfxQueue->elem[index].pData = pIfxQueue->elem[index].pHbData;
        }
        pIfxQueue->pReadBuffer = pIfxQueue->elem[0].pData;
        pIfxQueue->pWriteBuffer = pIfxQueue->elem[1].pData;
        pIfxQueue->readActive = false;
        pIfxQueue->writeActive = false;
    }
    return;
}   /* End of function */


#if !FREERTOS_ENABLE
uint32_t xferLength;
Cy_IfxQueue_t *pDataQ;
uint32_t writeIndex = 0x00, readIndex = 0x00, numElem = 0x00;
uint8_t *pBuffer;
uint16_t ssDataSize;
#endif /* !FREERTOS_ENABLE */

/*
 * Function: Cy_USB_EchoDeviceInit()
 * Description: Initialize EchoDevice for source/sink.
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: void
 */
void  *
Cy_USB_EchoDeviceInit (void)
{
    Cy_InitIfxQueue(&echoDevCtxt);
    return ((void *)(&echoDevCtxt));
}   /* End of function. */


/*
 * Function: Cy_USB_EchoDevicePrepareToStartDataXfer()
 * Description: It takes care of all preparation to start data transfer.
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: void
 */
void
Cy_USB_EchoDevicePrepareToStartDataXfer (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    Cy_IfxQueue_t *pIfxQueue;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    uint32_t endpNum;
    uint8_t *pBuffer;
    uint32_t readIndex = 0x00;

    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;
    /*
     * For loopback:
     * It is expected that data transfer should be started by host so
     * just call DMAread. Whatever data comes from host will be given
     * back t host.
     *
     * For Src/Snk:
     * This is infinite src and sink implementation so call DMAread/Write both.
     */
    DBG_APP_TRACE("Cy_USB_EchoDevicePrepareToStartDataXfer\r\n");
    switch (echodevice) {

        case ECHO_DEVICE_LOOPBACK:
        default:
            DBG_APP_TRACE("LoopBack\r\n");
            /* Loopback device will be default option */
            /* Endpoint 0 will be taken care by USBD layer. */
            for(endpNum = 0x01; endpNum < CY_USB_NUM_ENDP_CONFIGURED; endpNum++) {
                pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
                readIndex = pIfxQueue->readIndex;

                pBuffer = pIfxQueue->elem[readIndex].pData;
                pIfxQueue->elem[readIndex].dataLen = pIfxQueue->readLength;
                Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut, pBuffer,
                                    pIfxQueue->readLength);
                pIfxQueue->readActive = true;
            }
            break;

        case ECHO_DEVICE_SRC_SNK:
            DBG_APP_TRACE("SRC-SNK\r\n");
            /* Endpoint 0 will be taken care by USBD layer. */
            for(endpNum = 0x01; endpNum < CY_USB_NUM_ENDP_CONFIGURED; endpNum++) {
                pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
                Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                    (uint8_t *)(pIfxQueue->pReadBuffer),
                                    pIfxQueue->readLength);
                if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                    if ((USB32DEV->USB32DEV_EPM.IEPM_ENDPOINT[pIfxQueue->endpNumOut] & 0x00400000UL) == 0) {
                        DBG_APP_INFO("OUT EP %d not ready after socket enable\r\n", pIfxQueue->endpNumOut);
                    }
                }

                Cy_USB_AppQueueWrite(pAppCtxt, pIfxQueue->endpNumIn,
                                     (uint8_t *)(pIfxQueue->pWriteBuffer),
                                     pIfxQueue->writeLength);
                if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                    if ((USB32DEV->USB32DEV_EPM.EEPM_ENDPOINT[pIfxQueue->endpNumIn] & 0x40000000UL) == 0) {
                        DBG_APP_INFO("IN EP %d not ready after socket enable\r\n", pIfxQueue->endpNumIn);
                    }
                }
            }
            break;
        }
    return;
}   /* end of function */


/*
 * Function: Cy_USB_EchoDeviceHandleWriteComplete()
 * Description: It takes care of WRITE-COMPLETE message.
 * Parameter: cy_stc_usb_app_ctxt_t, endpAddr
 * return: void
 */
void
Cy_USB_EchoDeviceHandleWriteComplete (cy_stc_usb_app_ctxt_t *pAppCtxt,
                                     uint8_t endpAddr)
{
    Cy_IfxQueue_t *pIfxQueue;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint8_t endpNum, endpDir;
    uint8_t *pBuffer;
    uint32_t writeIndex= 0x00, readIndex = 0x00, numElem = 0x00;

    /*
     * For loopback:::
     * update number of elements, readindex and if space is
     * available then initiate DMA read.
     * If space is not available then activate NAK/NRDY.
     * If write was not initiated earlier due to non-availability
     * of data then initiate write DMA.
     * Before activating write, need to clear NAK/NRDY bit.
     *
     * For Src/Snk:::
     * Read complete so initiate another DMA read.
     */
    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleWriteComplete >>\r\n");
    DBG_APP_TRACE("EndpAddr:0x%x \r\n", endpAddr);

    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;

    endpNum = (endpAddr & CY_USBD_ENDP_NUM_MASK);
    endpDir = (endpAddr & CY_USBD_ENDP_DIR_MASK);

    if (endpDir) {
        pEndpDmaSet = &(pAppCtxt->endpInDma[endpNum]);
    } else {
        pEndpDmaSet = &(pAppCtxt->endpOutDma[endpNum]);
    }
    pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);

    switch (echodevice) {

        case ECHO_DEVICE_LOOPBACK:
        default:
            DBG_APP_TRACE("ECHO DEVICE\r\n");
            if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                Cy_HBDma_Channel_WaitForSendCplt(&(pEndpDmaSet->hbDmaChannel), 0);
            }

            /* Loopback device */
            pIfxQueue->numElem -= 1;

            pIfxQueue->writeIndex =
                (pIfxQueue->writeIndex + 1) % CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE;

            writeIndex = pIfxQueue->writeIndex;
            numElem = pIfxQueue->numElem;
            if (numElem != 0x00) {
                DBG_APP_TRACE("NumElemNon-Zero\r\n");
                /* device has some element so send back to host */
                pIfxQueue->writeLength =
                           pIfxQueue->elem[writeIndex].dataLen;
                /*
                 * For USBHS, ZLP need to send by controller explicitly and
                 * it can not be sent through DMA. So special arrangement for
                 * USBHS ZLP.
                 */
                if ((pIfxQueue->writeLength == 0x00) &&
                    (pAppCtxt->pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS)) {
                    DBG_APP_TRACE("SendZlp\r\n");
                    Cy_USBD_SendEgressZLP(pAppCtxt->pUsbdCtxt,
                                          pIfxQueue->endpNumIn);
                } else {
                    pBuffer = pIfxQueue->elem[writeIndex].pData;
                    Cy_USB_AppQueueWrite(pAppCtxt, pIfxQueue->endpNumIn,
                                         pBuffer,  pIfxQueue->writeLength);
                }
            } else {
                /* Nothing to send so initiate NAK */
                DBG_APP_TRACE("numElem-0. Make WriteActive-False\r\n");
                pIfxQueue->writeActive = false;
            }

            /*
             * Boundry case where read waiting for space and
             * after write completion space is available.
             * First clear NAK then start usual activity.
             */
            if (pIfxQueue->readActive == false) {
                DBG_APP_TRACE("ReadActive-False...activate\r\n");
                readIndex = pIfxQueue->readIndex;
                pBuffer = pIfxQueue->elem[readIndex].pData;
                pIfxQueue->elem[readIndex].dataLen = pIfxQueue->readLength;
                Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                    pBuffer, pIfxQueue->readLength);
                pIfxQueue->readActive = true;
            }
            break;

        case ECHO_DEVICE_SRC_SNK:
            DBG_APP_TRACE("SRC_SNK \r\n");
            if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                Cy_HBDma_Channel_WaitForSendCplt(&(pEndpDmaSet->hbDmaChannel), 0);
            }
            Cy_USB_InitializeWriteBuffer(pIfxQueue->pWriteBuffer,
                                         CY_USB_MAX_DATA_BUFFER_SIZE);
            Cy_USB_AppQueueWrite(pAppCtxt, pIfxQueue->endpNumIn,
                                 pIfxQueue->pWriteBuffer,
                                 pIfxQueue->writeLength);
            break;
        }
    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleWriteComplete <<\r\n\r\n");
    return;
}   /* End of function */


/*
 * Function: Cy_USB_EchoDeviceHandleReadComplete()
 * Description: It takes care of READ-COMPLETE message.
 * Parameter: cy_stc_usb_app_ctxt_t, endpAddr
 * return: void
 */
void
Cy_USB_EchoDeviceHandleReadComplete (cy_stc_usb_app_ctxt_t *pAppCtxt,
                                    uint8_t endpAddr)
{
    Cy_IfxQueue_t *pIfxQueue;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint8_t endpNum, endpDir;
    uint8_t *pBuffer;
    uint32_t ssDataSize;
    uint32_t writeIndex= 0x00, readIndex = 0x00, numElem = 0x00;

    /*
     * For loopback:::
     * update number of elements, readindex and if space is
     * available then initiate DMA read.
     * If space is not available then activate NAK/NRDY.
     * If write was not initiated earlier due to non-availability
     * of data then initiate write DMA.
     * Before activating write, need to clear NAK/NRDY bit.
     *
     * For Src/Snk:::
     * Read complete so initiate another DMA read.
     */

    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleReadComplete >>\r\n");
    DBG_APP_TRACE("EndpAddr:0x%x \r\n",endpAddr);

    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;

    endpNum = (endpAddr & CY_USBD_ENDP_NUM_MASK);
    endpDir = (endpAddr & CY_USBD_ENDP_DIR_MASK);

    if (endpDir) {
        pEndpDmaSet = &(pAppCtxt->endpInDma[endpNum]);
    } else {
        pEndpDmaSet = &(pAppCtxt->endpOutDma[endpNum]);
    }
    pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);

    switch (echodevice) {

        case ECHO_DEVICE_LOOPBACK:
        default:
            DBG_APP_TRACE("ECHO DEVICE\r\n");
            /*
             * In case of USB-SS, the size of data read is received
             * as part of the message.
             * In Case of USB-HS CUPSS-DMA will read requested data or short
             * intr will come and then need to read exact data.
             */
            if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                /* Update amount of data recv before move to next read indx */
                DBG_APP_TRACE("Calling WaitForReceieveCplt\r\n");
                Cy_HBDma_Channel_WaitForReceiveCplt(&(pEndpDmaSet->hbDmaChannel),
                                                    0, &ssDataSize);
                pIfxQueue->elem[pIfxQueue->readIndex].dataLen = ssDataSize;
                DBG_APP_TRACE("ssDataSize:%d\r\n", ssDataSize);
            }

            pIfxQueue->numElem += 1;
            pIfxQueue->readIndex =
                (pIfxQueue->readIndex + 1) % CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE;

            numElem = pIfxQueue->numElem;
            readIndex = pIfxQueue->readIndex;
            if (numElem < CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE) {
                 /* You can initiate read again */
                 pBuffer = pIfxQueue->elem[readIndex].pData;
                 pIfxQueue->elem[readIndex].dataLen = pIfxQueue->readLength;
                 DBG_APP_TRACE("HndleReadCmplt:Next: endp::%d, pBuffer:0x%x, len:0x%x\r\n",
                               pIfxQueue->endpNumOut,
                               pBuffer, pIfxQueue->readLength);
                 Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                     pBuffer, pIfxQueue->readLength);
                 pIfxQueue->readActive = true;
            } else {
                /* Space not available. Device will automatically send NAK. */
                pIfxQueue->readActive = false;
                DBG_APP_ERR("QueueSpace NotAvailabale \r\n");
            }

            /* If write is not submitted then  go ahead */
            if (pIfxQueue->writeActive == false) {
                DBG_APP_TRACE("WriteActive-False...activate\r\n");
                /*
                 * There is possibility that writeActive  was false
                 * because data was not available so disable NAK first.
                 */
                writeIndex = pIfxQueue->writeIndex;
                pIfxQueue->writeLength = pIfxQueue->elem[writeIndex].dataLen;
                pBuffer = pIfxQueue->elem[writeIndex].pData;

                DBG_APP_TRACE("inEndp::%d, pBuffer:0x%x, len:0x%x\r\n",
                              pIfxQueue->endpNumIn, pBuffer,
                              pIfxQueue->writeLength);
                /*
                 * For USBHS, ZLP need to send by controller explicitly and
                 * it can not be sent through DMA. So special arrangement for
                 * USBHS ZLP.
                 */
                if ((pIfxQueue->writeLength == 0x00) &&
                    (pAppCtxt->pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS)) {
                    DBG_APP_TRACE("SendZlp\r\n");
                    Cy_USBD_SendEgressZLP(pAppCtxt->pUsbdCtxt,
                                          pIfxQueue->endpNumIn);
                } else {
                    DBG_APP_TRACE("Calling QueueWrite\r\n");
                    Cy_USB_AppQueueWrite(pAppCtxt, pIfxQueue->endpNumIn,
                                         pBuffer, pIfxQueue->writeLength);
                }
                pIfxQueue->writeActive = true;
            }
            break;

        case ECHO_DEVICE_SRC_SNK:
            DBG_APP_TRACE("SRC_SNK \r\n");
            if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                Cy_HBDma_Channel_WaitForReceiveCplt(&(pEndpDmaSet->hbDmaChannel),
                                                    0, &ssDataSize);
                pIfxQueue->elem[pIfxQueue->readIndex].dataLen = ssDataSize;
            }
            /*
             * SRC-SNK does not care about ZLP/SLP treatment. It has to discard
             * all the data...dont need to send back anything.
             */
            Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                pIfxQueue->pReadBuffer,
                                pIfxQueue->readLength);
            break;
        }

    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleReadComplete <<\r\n\r\n");
    return;
}   /* end of function */


/*
 * Function: Cy_USB_EchoDeviceHandleZlpIn()
 * Description: It takes care of IN-ZLP message.
 * Parameter: cy_stc_usb_app_ctxt_t, endpAddr
 * return: void
 */
void
Cy_USB_EchoDeviceHandleZlpIn (cy_stc_usb_app_ctxt_t *pAppCtxt,
                             uint8_t endpAddr)
{
    Cy_IfxQueue_t *pIfxQueue;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    uint8_t endpNum, endpDir;
    uint8_t *pBuffer;
    uint32_t writeIndex= 0x00, readIndex = 0x00, numElem = 0x00;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;

    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleZlpIn >>\r\n");
    /*
     * For loopback:::
     * Update required index and if data is available then send
     * data otherwise activate NAK/NRDY.
     * For SrcSnk:::
     * DMA already active which is submitted and with zlp it is
     * not affected so nothing is required.
     *
     * For SrcSnk:::
     * Activate DMA write again. If DMA write was not activated
     * it will be fresh activation. IF DMA write was active
     * earlier then previous one will be cleared and new one
     * will be in active state.
     *
     * Need to clear interrupt for both device.
     */

    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;

    endpNum = (endpAddr & CY_USBD_ENDP_NUM_MASK);
    endpDir = (endpAddr & CY_USBD_ENDP_DIR_MASK);

    if (endpDir) {
        pEndpDmaSet = &(pAppCtxt->endpInDma[endpNum]);
    } else {
        pEndpDmaSet = &(pAppCtxt->endpOutDma[endpNum]);
    }

    pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
    switch (echodevice) {

        case ECHO_DEVICE_LOOPBACK:
        default:
            if ((pIfxQueue->numElem == 0x00) &&
                (pEndpDmaSet->endpType == CY_USB_ENDP_TYPE_ISO)) {
                /*
                 * ISO is special case. If data is not available
                 * then USB controller will send ZLP and interrupt
                 * will be generated which need to be ignored.
                 */
                return;
            }
            pIfxQueue->numElem -= 1;
            pIfxQueue->writeIndex =
               (pIfxQueue->writeIndex + 1) % CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE;

            writeIndex = pIfxQueue->writeIndex;
            numElem = pIfxQueue->numElem;
            if (numElem != 0x00) {
                /* device has some element/data so send back to host */
                pIfxQueue->writeLength = pIfxQueue->elem[writeIndex].dataLen;
                /*
                 * For USBHS, ZLP need to send by controller explicitly and
                 * it can not be sent through DMA. So special arrangement for
                 * USBHS ZLP.
                 */
                if ((pIfxQueue->writeLength == 0x00) &&
                    (pAppCtxt->pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS)) {
                    DBG_APP_TRACE("SendZlp\r\n");
                    Cy_USBD_SendEgressZLP(pAppCtxt->pUsbdCtxt,
                                              pIfxQueue->endpNumIn);
                } else {
                    pBuffer = pIfxQueue->elem[writeIndex].pData;
                    Cy_USB_AppQueueWrite(pAppCtxt, pIfxQueue->endpNumIn,
                                         pBuffer,  pIfxQueue->writeLength);
                }
                /*
                 * Boundry case where read waiting for space and
                 * after write completion space is available.
                 * First clear NAK then start usual activity.
                 */
                if (pIfxQueue->readActive == false) {
                    readIndex = pIfxQueue->readIndex;
                    pBuffer = pIfxQueue->elem[readIndex].pData;
                    pIfxQueue->elem[readIndex].dataLen = pIfxQueue->readLength;
                    Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                        pBuffer, pIfxQueue->readLength);
                    pIfxQueue->readActive = true;
                }
            } else {
                pIfxQueue->writeActive = false;
            }

            break;

        case ECHO_DEVICE_SRC_SNK:
            /* SRC/SINK DEVICE */
            if ((pIfxQueue->numElem == 0x00) &&
                (pEndpDmaSet->endpType == CY_USB_ENDP_TYPE_ISO)) {
                /*
                 * ISO is special case. If data is not available
                 * then USB controller will send ZLP and interrupt
                 * will be generated which need to be ignored.
                 */
                DBG_APP_TRACE("Cy_USB_EchoDeviceHandleZlpIn <<\r\n");
                return;
            }
            Cy_USB_AppQueueWrite(pAppCtxt, pIfxQueue->endpNumIn,
                                 (uint8_t *)(pIfxQueue->pWriteBuffer),
                                 pIfxQueue->writeLength);
             break;
        }
    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleZlpIn <<\r\n");
    return;
}   /* end of function */


/*
 * Function: Cy_USB_EchoDeviceHandleZlpOut()
 * Description: It takes care of OUT-ZLP message.
 * Parameter: cy_stc_usb_app_ctxt_t, endpAddr
 * return: void
 */
void
Cy_USB_EchoDeviceHandleZlpOut (cy_stc_usb_app_ctxt_t *pAppCtxt,
                              uint8_t endpAddr)
{
    Cy_IfxQueue_t *pIfxQueue;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    uint8_t endpNum, endpDir;
    uint8_t *pBuffer;
    uint32_t writeIndex= 0x00, readIndex = 0x00, numElem = 0x00;

    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleZlpOut >>\r\n");

    /*
     * For loopback:::
     * 1. Terminate DMA operation to avoid any race condition where
     *    immediately data comes after ZLP OUT.
     * 2. Since DMA read was active so there is space for ZLP hence
     *    update buffer, Index and make length =0 for present read index.
     *    Same data will be sent in loopback.
     * 3. if space is available to read data then initiate DMA operation otherwise
     *    activate NAK/NRDY.
     *
     * For SrcSnk:::
     * 1. In Src-sink you just need to discard data so nothing
     *     needs to be done.
     */

    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;

    endpNum = (endpAddr & CY_USBD_ENDP_NUM_MASK);
    endpDir = (endpAddr & CY_USBD_ENDP_DIR_MASK);
    (void)endpDir;

    pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
    DBG_APP_INFO("ZLP-OUT endpNum::0x%x\r\n", endpNum);

    switch (echodevice) {

        case ECHO_DEVICE_LOOPBACK:
        default:
            /* Terminate function will check devic speed. */
            Cy_USB_AppTerminateDma(pAppCtxt, pIfxQueue->endpNumOut,
                                   CY_USB_ENDP_DIR_OUT);
            pIfxQueue->elem[pIfxQueue->readIndex].dataLen = 0x00;
            pIfxQueue->numElem += 1;

            pIfxQueue->readIndex =
                (pIfxQueue->readIndex + 1) % CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE;

            numElem = pIfxQueue->numElem;
            readIndex = pIfxQueue->readIndex;
            /*
             * If space is available then prepare for next xfer.
             * Now read buffer is changed so call AppQueueRead
             * again with new set of parameter considering space is
             * available.
             */
            if (numElem < CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE) {
                /* initiate read again with Full Data*/
                pBuffer = pIfxQueue->elem[readIndex].pData;
                pIfxQueue->elem[readIndex].dataLen = pIfxQueue->readLength;
                Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                    pBuffer, pIfxQueue->readLength);
                pIfxQueue->readActive = true;
            } else {
                pIfxQueue->readActive = false;
            }

            /* If write is not submitted then  go ahead */
            if (pIfxQueue->writeActive == false) {
                /*
                 * There is possibility that writeActive  was false
                 * because data was not available so disable NAK first.
                 */
                writeIndex = pIfxQueue->writeIndex;
                pIfxQueue->writeLength = pIfxQueue->elem[writeIndex].dataLen;
                pBuffer = pIfxQueue->elem[writeIndex].pData;
                /*
                 * if length is 0x00 then send ZLP through USB
                 * controller otherwise use DMA.
                 */
                if ((pIfxQueue->writeLength == 0x00) &&
                    (pAppCtxt->pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS)) {
                   DBG_APP_TRACE("SendZlp\r\n");
                   Cy_USBD_SendEgressZLP(pAppCtxt->pUsbdCtxt,
                                         pIfxQueue->endpNumIn);
                } else {
                    DBG_APP_TRACE("ActInDma\r\n");
                    Cy_USB_AppQueueWrite(pAppCtxt, pIfxQueue->endpNumIn,
                                         pBuffer,  pIfxQueue->writeLength);
                }
                pIfxQueue->writeActive = true;
            }
            /* after handling zlp, now clear zlp interrupt. */
            Cy_USBD_ClearZlpSlpIntrEnableMask(pAppCtxt->pUsbdCtxt,
                                              pIfxQueue->endpNumOut,
                                              CY_USB_ENDP_DIR_OUT,
                                              CY_USB_ARG_ZLP);
            break;



        case ECHO_DEVICE_SRC_SNK:
            /* after handling zlp, now clear zlp interrupt. */
            Cy_USBD_ClearZlpSlpIntrEnableMask(pAppCtxt->pUsbdCtxt,
                                              pIfxQueue->endpNumOut,
                                              CY_USB_ENDP_DIR_OUT,
                                              CY_USB_ARG_ZLP);
            break;
        }
    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleZlpOut <<\r\n");
    return;
}   /* end of function */


/*
 * Function: Cy_USB_EchoDeviceTaskHandler()
 * Description: This function handles data transfer for source/sink echo device.
 * Parameter: pTaskParam
 * return: void
 */
#if FREERTOS_ENABLE
void 
Cy_USB_EchoDeviceTaskHandler (void *pTaskParam)
#else
void 
Cy_USB_EchoDeviceTaskHandler (void *pTaskParam, void* qMsg)
#endif /* FREERTOS_ENABLE */
{
    Cy_IfxQueue_t *pIfxQueue;
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    cy_stc_usbd_app_msg_t queueMsg;

    /* endpAddress  will have endpNum and direction. */
    uint8_t endpAddr;
    uint32_t endpNum;
    cy_en_usbss_lnk_power_mode_t curLinkState;
    uint32_t lpEntryTime = 0;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;

    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;

#if FREERTOS_ENABLE
    BaseType_t xStatus;
    uint32_t xferLength;
    uint8_t *pBuffer;
    uint32_t idleLoopCnt = 0;

    DBG_APP_INFO("EchoDeviceThreadActive\r\n");

    /* Enable USB-3 connection and wait until it is stable. */
    vTaskDelay(500);

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent =
    (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

    if (pAppCtxt->vbusPresent) {
        UsbConnectionEnable(pAppCtxt);
    }

    do {
#if WATCHDOG_RESET_EN
        /* Kick The WDT to prevent RESET */
        KickWDT();
#endif /* WATCHDOG_RESET_EN */

        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->xQueue, &queueMsg, 1);
        if (xStatus != pdPASS) {
            idleLoopCnt++;
            if (idleLoopCnt >= 10000UL) {
                idleLoopCnt = 0;
                DBG_APP_INFO("TaskIdle\r\n");
            }

            continue;
        }
        idleLoopCnt = 0;
#else /* !FREERTOS_ENABLE */

        memcpy((uint8_t *)&queueMsg, (uint8_t *)qMsg,
                sizeof(cy_stc_usbd_app_msg_t));
#endif /* FREERTOS_ENABLE */

        /*
         * Make sure that the USB link is brought into active state
         * periodically to avoid stuck data transfers.
         */
        curLinkState = CY_USBSS_LPM_UNKNOWN;
        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USBSS_Cal_GetLinkPowerState(pAppCtxt->pUsbdCtxt->pSsCalCtxt, &curLinkState);
        }

        /*
         * If the link has been in USB2-L1 or in USB3-U2 for more than 0.5 seconds, initiate LPM exit so that
         * transfers do not get delayed significantly.
         */
        if (
                (curLinkState == CY_USBSS_LPM_U2) ||
                (
                 (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) &&
                 ((MXS40USBHSDEV_USBHSDEV->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0)
                )
           ) {
            if ((Cy_USBD_GetTimerTick() - lpEntryTime) >= 500UL) {
                lpEntryTime = Cy_USBD_GetTimerTick();
                Cy_USBD_GetUSBLinkActive(pAppCtxt->pUsbdCtxt);
            }
        } else {
            lpEntryTime = Cy_USBD_GetTimerTick();
        }

        endpAddr = queueMsg.data[0];
        endpNum = (endpAddr & CY_USBD_ENDP_NUM_MASK);

        switch (queueMsg.type) {
            case CY_USB_VBUS_DETECT_PRESENT:
                if ((pAppCtxt->vbusPresent == true) &&
                     (pAppCtxt->usbConnectDone == false)) {
                    DBG_APP_INFO("Connect due to VBus presence\r\n");
                    while (pAppCtxt->usbConnectDone == false) {
                        if (!UsbConnectionEnable(pAppCtxt)) {
                            DBG_APP_INFO("Calling\r\n");
                            lightDisable(pAppCtxt);
                            Cy_SysLib_Delay(100);
                        }
                    }
                }
                break;

            case CY_USB_VBUS_DETECT_ABSENT:
                if ((pAppCtxt->usbConnectDone == true) &&
                    (pAppCtxt->vbusPresent == false)) {
                    DBG_APP_INFO("Disconnect due to VBus loss\r\n");
                    UsbConnectionDisable(pAppCtxt);
                }
                break;

            case CY_USB_ECHO_DEVICE_MSG_SETUP_DATA_XFER:
                /*
                 * setup_data should be common for src/snk and loopback.
                 * Initialize endpoint number, max packet size.
                 * Device endpoint related to interface/functionality starts 
                 * from endpoint 1 (if there).
                 * For loopback endpoint pair will be 1-1,2-2...15-15.
                 */
                DBG_APP_TRACE("MsgSetupDataXfer\r\n");
                for (endpNum = 0x01; endpNum < CY_USB_NUM_ENDP_CONFIGURED; endpNum++) {
                    pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
                    pIfxQueue->endpNumOut = endpNum;

                    pIfxQueue->maxPktSizeOut =
                    Cy_USB_AppGetMaxPktSize(pAppCtxt, endpNum,
                                            CY_USB_ENDP_DIR_OUT);

                    pIfxQueue->readLength = pIfxQueue->maxPktSizeOut;
                    pIfxQueue->endpNumIn = endpNum;

                    pIfxQueue->maxPktSizeIn =
                    Cy_USB_AppGetMaxPktSize(pAppCtxt, endpNum,
                                            CY_USB_ENDP_DIR_IN);

                    pIfxQueue->writeLength = pIfxQueue->maxPktSizeIn;
                }
                break;

            case CY_USB_ECHO_DEVICE_MSG_START_DATA_XFER:
                DBG_APP_TRACE("MsgStartDataXfer\r\n");
                Cy_USB_EchoDevicePrepareToStartDataXfer(pAppCtxt);
                break;
 
            case CY_USB_ECHO_DEVICE_MSG_STOP_DATA_XFER:
                /* same for loopback and src/sink */
                for (endpNum = 0x01; endpNum < CY_USB_NUM_ENDP_CONFIGURED; endpNum++) {
                    pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
                    pIfxQueue->readActive = false;
                    pIfxQueue->writeActive = false;
                    pIfxQueue->readIndex  = 0x00;
                    pIfxQueue->writeIndex = 0x00;
                    pIfxQueue->numElem    = 0x00;
                }
                break;

            case CY_USB_ECHO_DEVICE_MSG_READ_COMPLETE:
                DBG_APP_TRACE("MsgReadComp\r\n");
                Cy_USB_EchoDeviceHandleReadComplete(pAppCtxt, endpAddr);
                break;

            case CY_USB_ECHO_DEVICE_MSG_WRITE_COMPLETE:
                DBG_APP_TRACE("MsgWriteComp\r\n");
                Cy_USB_EchoDeviceHandleWriteComplete(pAppCtxt, endpAddr);
                break;

            case CY_USB_ECHO_DEVICE_MSG_ZLP_OUT:
                DBG_APP_TRACE("MsgZlpOut\r\n");
                Cy_USB_EchoDeviceHandleZlpOut(pAppCtxt, endpAddr);
                break;

            case CY_USB_ECHO_DEVICE_MSG_SLP_OUT:
                /*
                 * Common:
                 * Get new Xfer Length.
                 * At the end clear interrupt.
                 *
                 * Loopback:::
                 * One DMA channel is already active (otherwise device will
                 * send NAK) so with SLP interrupt only xfer length is
                 * updated.
                 * No change in any Index.
                 * Then initiate DMA xfer.
                 *
                 * Src/Snk:::
                 * Just initiate read with new xfer length.
                 */
                DBG_APP_TRACE("EchoMsgSlpOut for  endp:0x%x\r\n", endpNum);
                pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
                xferLength = queueMsg.data[1];

                if (echodevice == ECHO_DEVICE_LOOPBACK) {
                    /*
                     * It means means DMA already submitted but due to SLP
                     * need to re-submit DMA. all other index should
                     * remain same. Only length should be changed.
                     */
                    pBuffer = pIfxQueue->elem[pIfxQueue->readIndex].pData;
                    pIfxQueue->elem[pIfxQueue->readIndex].dataLen = xferLength;
                    DBG_APP_INFO("SLP-OUT endpNum::0x%x xferLen:0x%x\r\n",
                                 pIfxQueue->endpNumOut, xferLength);
                    Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                        pBuffer, xferLength);
                    pIfxQueue->readActive = true;
                } else {
                    Cy_USB_AppQueueRead(pAppCtxt, pIfxQueue->endpNumOut,
                                        pIfxQueue->pReadBuffer, xferLength);
                }

                /* Send trigger to DMA channel so that DMA transfer is completed. */
                Cy_TrigMux_SwTrigger(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + pIfxQueue->endpNumOut,
                                     CY_TRIGGER_TWO_CYCLES);
                break;

            case CY_USB_ECHO_DEVICE_MSG_ZLP_IN:
                DBG_APP_TRACE("MsgZlpIn\r\n");
                Cy_USB_EchoDeviceHandleZlpIn(pAppCtxt, endpAddr);
                break;

            case CY_USB_ECHO_DEVICE_MSG_SLP_IN:
                /*
                 * Loopback:::
                 * In SLP also some data are sent so WRITE_COMPLETE message
                 * will be sent through DMA interrupt. In WRITE_COMPLETE case,
                 * Initiate next write. Here just clear interrupt.
                 *
                 * Src/Snk:::
                 * Same case as loopback. Just need to clear SLP interrupt.
                 */
                DBG_APP_TRACE("EchoMsgSlpIN\r\n");
                pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
                /* clear SLP interrupt. */
                Cy_USBD_ClearZlpSlpIntrEnableMask(pAppCtxt->pUsbdCtxt,
                                                  pIfxQueue->endpNumIn,
                                                  CY_USB_ENDP_DIR_IN,
                                                  CY_USB_ARG_SLP);
                break;

            case CY_USB_ECHO_DEVICE_MSG_CTRL_XFER_SETUP:
                DBG_APP_TRACE("CY_USB_ECHO_DEVICE_MSG_CTRL_XFER_SETUP\r\n");
                Cy_USB_EchoDeviceHandleCtrlSetup((void *)pAppCtxt, &queueMsg);
                break;

            case CY_USB_ECHO_DEVICE_SET_FEATURE:
                DBG_APP_INFO("EchoMsgSetFeature\r\n");
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                break;

            case CY_USB_ECHO_DEVICE_CLEAR_FEATURE:
                DBG_APP_INFO("EchoMsgClearFeature\r\n");
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                break;

            case CY_USB_ENDP0_READ_COMPLETE:
                DBG_APP_INFO("Endp0ReadComplete\r\n");
                /* Here application gets the data. */
                break;

            case CY_USB_ENDP0_READ_TIMEOUT:
                DBG_APP_INFO("Endp0ReadTimeout\r\n");
                /*
                 * When application layer wants to recieve data from
                 * host through endpoint 0 then device initiate RcvEndp0
                 * function call and start timer. When timer ends and still
                 * data is not recieved then TIMER interrupt will send
                 * CY_USB_ENDP0_READ_TIMEOUT message. If data is recieved then
                 * case which handles data should stop timer.
                 */
                Cy_USB_USBD_RetireRecvEndp0Data(pAppCtxt->pUsbdCtxt);
                break;

            case CY_USB_ECHO_DEVICE_MSG_L1_SLEEP:
                DBG_APP_TRACE("CY_USB_ECHO_DEVICE_MSG_L1_SLEEP\r\n");
                /* Ass off now nothing needs to be done here. */
                break;

            case CY_USB_ECHO_DEVICE_MSG_L1_RESUME:
                DBG_APP_TRACE("CY_USB_ECHO_DEVICE_MSG_L1_RESUME\r\n");
                /* Ass off now nothing needs to be done here. */
                break;

            default:
                DBG_APP_ERR("EchoMsgDefault %d\r\n", queueMsg.type);
                break;
        }   /* end of switch() */
#if FREERTOS_ENABLE
    } while (1);
#endif /* FREERTOS_ENABLE */
}   /* End of function  */

/*
 * Function     :  Cy_USB_ResetDMAChannel()
 * Description  :  Reset appropriate DMA Channel based on device speed (HBDMA channel for USBSS and above and DW for USBHS and below speeds)
 * Parameters   :  cy_stc_app_endp_dma_set_t *pEpDmaCfg, cy_en_usb_speed_t speed
 * Return       :  void
 */

static void Cy_USB_ResetDMAChannel (cy_stc_app_endp_dma_set_t *pEpDmaCfg, cy_en_usb_speed_t speed)
{
    if ((pEpDmaCfg != NULL) && (pEpDmaCfg->valid))
    {
        if(speed > CY_USBD_USB_DEV_HS)
        {
            Cy_HBDma_Channel_Reset(&(pEpDmaCfg->hbDmaChannel));
        }
        else
        {
            Cy_USBHS_App_ResetEpDma(pEpDmaCfg);
        }
    }
}


/*
 * Function: Cy_USB_EchoDeviceHandleCtrlSetup()
 * Description: This function handles control command given to application.
 * Parameter: pApp, pMsg
 * return: void
 */
void
Cy_USB_EchoDeviceHandleCtrlSetup (void *pApp, cy_stc_usbd_app_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    cy_en_usb_endp_dir_t endpDir = CY_USB_ENDP_DIR_INVALID;
    uint8_t endpNum = 0;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    uint32_t  setupData0;
    uint32_t  setupData1;
    uint8_t bmRequest, bRequest, bTarget;
    uint16_t wValue, wIndex, wLength;
    uint8_t   reqType;
    bool isReqHandled = false;
    uint32_t baseAddr;
    uint16_t i;
    uint16_t loopCnt = 1000u;
    cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt;
    Cy_IfxQueue_t *pIfxQueue;
    uint8_t *pBuffer;
    uint32_t readIndex = 0;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pApp;
    pEchoDevCtxt = (cy_stc_usb_echo_dev_ctxt_t *)pAppCtxt->pDevFuncCtxt;

    setupData0 = pMsg->data[0];
    setupData1 = pMsg->data[1];

    DBG_APP_TRACE("Cy_USB_EchoDeviceHandleCtrlSetup\r\n");
    /* Decode the fields from the setup request. */
    bmRequest = (uint8_t)((setupData0 & CY_USB_BMREQUEST_SETUP0_MASK) >>
                           CY_USB_BMREQUEST_SETUP0_POS);
    bRequest =  (uint8_t)((setupData0 & CY_USB_BREQUEST_SETUP0_MASK) >>
                           CY_USB_BREQUEST_SETUP0_POS);
    wValue = (uint16_t)((setupData0 & CY_USB_WVALUE_SETUP0_MASK) >>
                         CY_USB_WVALUE_SETUP0_POS);
    wIndex = (uint16_t)((setupData1 & CY_USB_WINDEX_SETUP1_MASK) >>
                         CY_USB_WINDEX_SETUP1_POS);
    wLength = (uint16_t)((setupData1 & CY_USB_WLENGTH_SETUP1_MASK) >>
                          CY_USB_WLENGTH_SETUP1_POS);

    reqType = ((bmRequest & CY_USB_CTRL_REQ_TYPE_MASK) >>
                                                CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK);

    switch (reqType) {

        case CY_USB_CTRL_REQ_STD:
            DBG_APP_TRACE("StdReq\r\n");
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)) {
                DBG_APP_INFO("SetFeatureReq: EndpHalt\r\n");
                endpDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                         (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt,
                                              ((uint32_t)wIndex & 0x7FUL),
                                               endpDir, true);
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }
            
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE)) {
                switch (wValue) {
                    case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
                        DBG_APP_INFO("SetFeature:CY_USB_FEATURE_DEVICE_REMOTE_WAKE\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    case CY_USB_FEATURE_U1_ENABLE:
                        DBG_APP_INFO("SetFeature:CY_USB_FEATURE_U1_ENABLE\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U2_ENABLE:
                        DBG_APP_INFO("SetFeature:CY_USB_FEATURE_U2_ENABLE\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    
                    default:
                    /* Unknown feature selector: Request will be stalled below. */
                    break;
                }
            }

            /* Handle FUNCTION_SUSPEND here */
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) &&
                (wValue == 0x00)) {

                /* TODO: Send a queue Msg to set the link to U2 */
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }

            if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)) {

                endpDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                         (CY_USB_ENDP_DIR_OUT));
                endpNum = (uint32_t)wIndex & 0x7FUL;

                pIfxQueue = &(pEchoDevCtxt->dataQueue[endpNum]);
                pIfxQueue->readActive = false;
                pIfxQueue->writeActive = false;
                pIfxQueue->readIndex  = 0x00;
                pIfxQueue->writeIndex = 0x00;
                pIfxQueue->numElem    = 0x00;
                readIndex = pIfxQueue->readIndex;

                pBuffer = pIfxQueue->elem[readIndex].pData;
                pIfxQueue->elem[readIndex].dataLen = pIfxQueue->readLength;
                
                DBG_APP_INFO("ClearFeatureReq EP:0x%x Dir:%d \r\n",endpNum, endpDir);
                
                if(endpDir == CY_USB_ENDP_DIR_IN)
                {
                    Cy_USB_ResetDMAChannel (&(pAppCtxt->endpInDma[endpNum]), pAppCtxt->pUsbdCtxt->devSpeed);
                }
                else
                {
                    Cy_USB_ResetDMAChannel (&(pAppCtxt->endpOutDma[endpNum]), pAppCtxt->pUsbdCtxt->devSpeed);
                }

                Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt,endpNum, endpDir);
                Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt,endpNum, endpDir, false);
                Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, endpNum, endpDir, false);

                if(endpDir == CY_USB_ENDP_DIR_OUT)
                {
                  Cy_USB_AppQueueRead(pAppCtxt, endpNum, pBuffer,
                      pIfxQueue->readLength);
                }

                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }
            
            if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE)) {
                switch (wValue) {
                    case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
                        DBG_APP_INFO("ClrFeature:CY_USB_FEATURE_DEVICE_REMOTE_WAKE\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
            /* TBD NT Enablng LPM only for CV test */
                        DBG_APP_INFO("Enabling LPM\r\n");
            Cy_USBD_LpmEnable(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U1_ENABLE:
                        DBG_APP_INFO("ClearFeature:CY_USB_FEATURE_U1_ENABLE\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U2_ENABLE:
                        DBG_APP_INFO("ClearFeature:CY_USB_FEATURE_U2_ENABLE\r\n");
                        Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    
                    default:
                        /*
                         * Unknown feature selector so dont handle here.
                         * just send stall.
                         */
                        DBG_APP_INFO("default of wValue\r\n");
                        isReqHandled = false;
                    break;
                }
            }
            
            /* SET_SEL req is supposed to have an OUT data phase of 6 bytes. */
            if ((bRequest == CY_USB_SC_SET_SEL) && (wLength == 6)) {
                /* SET_SEL request is only received in USBSS case and the Cy_USB_USBD_RecvEndp0Data is blocking. */
                retStatus =
                Cy_USB_USBD_RecvEndp0Data(pAppCtxt->pUsbdCtxt,
                                          (uint8_t *)SetSelDataBuffer,
                                          wLength);
                DBG_APP_INFO("SET_SEL: EP0 recv stat = %d, Data=%x:%x\r\n",
                        retStatus, SetSelDataBuffer[0], SetSelDataBuffer[1]);
                isReqHandled = true;
            }

#if USE_WINUSB
            /* Handle Microsoft OS String Descriptor request. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

                /* Make sure we do not send more data than requested. */
                if (wLength > glOsString[0]) {
                    wLength = glOsString[0];
                }

                DBG_APP_INFO("OSString\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                      glOsString, wLength);
                if (retStatus != CY_USBD_STATUS_SUCCESS) {
                    DBG_APP_INFO("SendEp0Fail\r\n");
                }
                isReqHandled = true;
            }
#endif /* USE_WINUSB */
            break;

        case CY_USB_CTRL_REQ_CLASS:
        case CY_USB_CTRL_REQ_VENDOR:
            DBG_APP_INFO("classORVendorReq\r\n");

            if ((bRequest == 0xB8) && (wLength != 0) &&
                ((wValue & 0x3) == 0) && ((wValue + wLength) <= 4096U)) {
                if ((bmRequest & 0x80) != 0) {
                    retStatus =
                    Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                              ((uint8_t *)Ep0TestBuffer) + wValue,
                                              wLength);
                } else {
                    retStatus =
                    Cy_USB_USBD_RecvEndp0Data(pAppCtxt->pUsbdCtxt,
                                              ((uint8_t *)Ep0TestBuffer) + wValue,
                                              wLength);

                    /* Wait until receive DMA transfer has been completed.
                     * TODO: Timeout to be added.
                     */
                    while (!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) {
                        Cy_SysLib_DelayUs(10);
                    }
                }

                if (retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }

            /* Command to read or write SRAM content or MMIO registers for debug. */
            if (((bRequest == REG_MEMORY_READ_CODE) || (bRequest == REG_MEMORY_WRITE_CODE)) &&
                    (wLength != 0) && (wLength <= 4096U) &&  ((wLength & 0x03) == 0))
            {
               if((bmRequest & 0x80U) != 0)
               {
                   baseAddr = (wValue << 16U) | wIndex;

                   if ((baseAddr >= CY_HBW_SRAM_BASE_ADDR) &&
                           (baseAddr < CY_HBW_SRAM_LAST_ADDR)) 
                   {
                       Cy_HBDma_EvictReadCache(false);
                       retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                               (uint8_t *)baseAddr,
                               wLength);
                    } else if (Cy_USB_IsValidMMIOAddr(baseAddr)) {
                       DBG_APP_INFO("Vendor Command 0x%x: Read from 0x%x\r\n", bRequest, baseAddr);
                       for (i = 0; i < wLength / 4; i++) {
                           Ep0TestBuffer[i] = ((uint32_t *)baseAddr)[i];
                       }
                       retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                               (uint8_t *)Ep0TestBuffer,
                               wLength);
                   }
                   if (retStatus == CY_USBD_STATUS_SUCCESS) {
                       isReqHandled = true;
                   }
               }

               if ((bmRequest & 0x80U) == 0)
               {
                   baseAddr = (wValue << 16U) | wIndex;

                   if ((baseAddr < CY_HBW_SRAM_BASE_ADDR) || (baseAddr >= CY_HBW_SRAM_LAST_ADDR)) 
                   {
                       /* Since we cannot use High BandWidth DMA to get data directly into non
                        * High BandWidth RAM regions, get the data into Ep0TestBuffer first and then copy
                        * it where it is supposed to go.
                        */
                       retStatus = Cy_USB_USBD_RecvEndp0Data(pAppCtxt->pUsbdCtxt, (uint8_t *)Ep0TestBuffer, wLength);

                       if (retStatus == CY_USBD_STATUS_SUCCESS) {
                           /* Wait until receive DMA transfer has been completed. */
                           while ((!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) && (loopCnt--)) {
                              vTaskDelay(1);
                           }

                           if (!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) {
                               Cy_USB_USBD_RetireRecvEndp0Data(pAppCtxt->pUsbdCtxt);
                           } else {
                               isReqHandled = true;

                               for (i = 0; i < wLength / 4; i++) {
                                   ((uint32_t *)baseAddr)[i] = Ep0TestBuffer[i];
                               }
                           }
                       }
                   } 
                   else if (Cy_USB_IsValidMMIOAddr(baseAddr))
                   {
                       DBG_APP_INFO("Vendor Command 0x%x: Write to 0x%x\r\n", bRequest, baseAddr);
                       retStatus = Cy_USB_USBD_RecvEndp0Data(pAppCtxt->pUsbdCtxt, (uint8_t *)baseAddr, wLength);

                       if (retStatus == CY_USBD_STATUS_SUCCESS) {
                           /* Wait until receive DMA transfer has been completed. */
                           while ((!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) && (loopCnt--)) {
                              vTaskDelay(1);
                           }

                           if (!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) {
                               Cy_USB_USBD_RetireRecvEndp0Data(pAppCtxt->pUsbdCtxt);
                           } else {
                               isReqHandled = true;
                           }
                       }
                   }
               }
            }

            if ((bRequest == 0xC8) && (wIndex != 0)) {
                pktType = wValue;
                if (pktLength > 1024) {
                    pktLength = 1024;
                }
                pktLength = wIndex;
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }

            if ((bRequest == 0xE0) && (wLength == 0)) {
                Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;

                Cy_SysLib_Delay(100);
                NVIC_SystemReset();
            }

#if USE_WINUSB
            /* Handle OS Compatibility and OS Feature requests */
            if (bRequest == MS_VENDOR_CODE) {
                /*
                 * this one is VENDOR request. As off now class and vendor
                 * request under fallback case statement.
                 */
                if (wIndex == 0x04) {
                    if (wLength > glOsCompatibilityId[0]) {
                        wLength = glOsCompatibilityId[0];
                    }
                    DBG_APP_INFO("OSCompat\r\n");
                    retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                 glOsCompatibilityId, wLength);
                    if (retStatus != CY_USBD_STATUS_SUCCESS) {
                        DBG_APP_INFO("SendEp0Fail\r\n");
                    }
                    isReqHandled = true;

                } else if (wIndex == 0x05) {

                    if (wLength > glOsFeature[0]) {
                        wLength = glOsFeature[0];
                    }
                    DBG_APP_INFO("OSFeature\r\n");
                    retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                          glOsFeature, wLength);
                    if (retStatus != CY_USBD_STATUS_SUCCESS) {
                        DBG_APP_INFO("SendEp0Fail\r\n");
                    }
                    isReqHandled = true;
                }
            }
#endif /* USE_WINUSB */
            break;

        default:
            DBG_APP_INFO("EchoDeviceHandleCtrlSetup:Default\r\n");
            break;
    }

    if(!isReqHandled) {
        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
    return;

}   /* end of function() */


/*
 * Function: Cy_USB_Endp0ReadComplete()
 * Description:  Handler for DMA transfer completion on endpoint 0 OUT Transfer.
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: void
 */
void
Cy_USB_Endp0ReadComplete (void *pApp)
{
    BaseType_t status;
    cy_stc_usbd_app_msg_t xMsg;
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;
    xMsg.type = CY_USB_ENDP0_READ_COMPLETE;
    xMsg.data[0] = 0x00; /* Out and endp 0 */
    xMsg.data[1] = 0;

#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
    DBG_APP_TRACE("Sent CY_USB_ENDP0_READ_COMPLETE\r\n");
#else

    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function */

/*
 * Function: Cy_USB_EchoDeviceDmaReadCompletion()
 * Description:  Handler for DMA transfer completion on OUT endpoint 
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: uint8_t
 */
void 
Cy_USB_EchoDeviceDmaReadCompletion (void *pApp, uint8_t endpAddr)
{
    BaseType_t status;
    cy_stc_usbd_app_msg_t xMsg;
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;

    xMsg.type = CY_USB_ECHO_DEVICE_MSG_READ_COMPLETE;
    xMsg.data[0] = endpAddr;
    xMsg.data[1] = 0;

#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
    DBG_APP_TRACE("Sent CY_USB_ECHO_DEVICE_MSG_READ_COMPLETE\r\n");
#else
    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function */

/*
 * Function: Cy_USB_EchoDeviceDmaWriteCompletion()
 * Description:  Handler for DMA transfer completion on OUT endpoint 
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: uint8_t
 */
void 
Cy_USB_EchoDeviceDmaWriteCompletion (void *pApp, uint8_t endpAddr)
{
    BaseType_t status;
    cy_stc_usbd_app_msg_t xMsg;
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;

    xMsg.type = CY_USB_ECHO_DEVICE_MSG_WRITE_COMPLETE;
    xMsg.data[0] = endpAddr;
    xMsg.data[1] = 0;
#if FREERTOS_ENABLE
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    status = xQueueSendFromISR(pAppCtxt->xQueue,  &(xMsg),
                               &(xHigherPriorityTaskWoken));
    DBG_APP_TRACE("Sent CY_USB_ECHO_DEVICE_MSG_WRITE_COMPLETE\r\n");
#else

    Cy_USB_EchoDeviceTaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */

    (void)status;
    return;
}   /* end of function */
