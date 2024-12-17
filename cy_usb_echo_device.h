/***************************************************************************//**
* \file cy_usb_echo_device.h
* \version 1.0
*
* Defines the messages and constants used in the USB Echo Device implementation.
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

#ifndef _CY_USB_ECHO_DEVICE_H_
#define _CY_USB_ECHO_DEVICE_H_

#if defined(__cplusplus)
extern "C" {
#endif

#define CY_USB_ECHO_DEVICE_MSG_SETUP_DATA_XFER      (0x01)
#define CY_USB_ECHO_DEVICE_MSG_START_DATA_XFER      (0x02)
#define CY_USB_ECHO_DEVICE_MSG_STOP_DATA_XFER       (0x03)
#define CY_USB_ECHO_DEVICE_MSG_READ_COMPLETE        (0x04) 
#define CY_USB_ECHO_DEVICE_MSG_WRITE_COMPLETE       (0x05)
#define CY_USB_ECHO_DEVICE_MSG_ZLP_IN               (0x06)
#define CY_USB_ECHO_DEVICE_MSG_SLP_IN               (0x07)
#define CY_USB_ECHO_DEVICE_MSG_ZLP_OUT              (0x08)
#define CY_USB_ECHO_DEVICE_MSG_SLP_OUT              (0x09)
#define CY_USB_ECHO_DEVICE_MSG_L1_SLEEP             (0x0A)
#define CY_USB_ECHO_DEVICE_MSG_L1_RESUME            (0x0B)
#define CY_USB_ECHO_DEVICE_SET_FEATURE              (0x0C)
#define CY_USB_ECHO_DEVICE_CLEAR_FEATURE            (0x0D)
#define CY_USB_ENDP0_READ_COMPLETE                 (0x0E)
#define CY_USB_ENDP0_READ_TIMEOUT                  (0x0F)
#define CY_USB_VBUS_DETECT_PRESENT                 (0x10)
#define CY_USB_VBUS_DETECT_ABSENT                  (0x11)
#define CY_USB_ECHO_DEVICE_MSG_CTRL_XFER_SETUP      (0x12)

typedef enum {
    REG_MEMORY_READ_CODE   = 0xA0,      /* Request used to read registers and/or memory. */
    REG_MEMORY_WRITE_CODE  = 0xA1,      /* Request used to write to SRAM locations. */
    MS_VENDOR_CODE         = 0xF0,      /* Request used to fetch MS-OS feature and capability descriptors. */
} cy_en_usb_app_vendor_opcode_t;

#define CY_USB_ECHO_DEVICE_MSG_QUEUE_SIZE  16
#define CY_USB_ECHO_DEVICE_MSG_SIZE  (sizeof (cy_stc_usbd_app_msg_t))

#define CY_USB_MAX_DATA_BUFFER_SIZE    (1024)

#define CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE 8

/* Used by application layer based on number of endp configured.*/
#define CY_USB_NUM_ENDP_CONFIGURED 6

/*
 * Echo device can work in two different way.
 * 1. Loopback: Whatever data comes from Host will be given back to
 *    host.
 * 2. Soruce/Sink: Device will consuming data coming from host and it will
 *    be sending data whenever host ask for the same.
 * variable "unsigned int echodevice" should be initialize to one of them.
 */
#define ECHO_DEVICE_LOOPBACK 0x00
#define ECHO_DEVICE_SRC_SNK  0x01

#define BULK_IN_ENDPOINT    (0x01)
#define BULK_OUT_ENDPOINT    (0x01)

typedef struct cy_stc_usb_echo_dev_ctxt_ cy_stc_usb_echo_dev_ctxt_t;
typedef struct Cy_IfxQueue_ Cy_IfxQueue_t;
typedef struct Cy_IfxQueueElem_ Cy_IfxQueueElem_t;

/* QueueElem and IfxQueue data structurs defined for Loopback device. */
struct Cy_IfxQueueElem_ {
    uint8_t  *pData;   /* Based on speed of device pointer will be changed. */
    uint8_t  *pHbData; /* High bandwidth data allocated and stored. */
    uint32_t dataLen;
};

/*
 * SRC-SINK mode dont need to maintain data so next two pointers are used
 * instead complete queue.
 * pReadBuffer: Used when device in SRC-SINK mode. Gets 0th
 *              location from dataQueue.
 * pWriteBuffer: Used when device in SRC-SINK mode. Gets 1th
 *               location from dataQueue.
 */
struct Cy_IfxQueue_ {
    Cy_IfxQueueElem_t elem[CY_IFX_ECHO_LOOPBACK_MAX_QUEUE_SIZE];
    uint32_t readIndex;
    uint32_t writeIndex;
    uint32_t numElem;
    uint8_t *pReadBuffer;
    uint32_t readLength;
    uint8_t *pWriteBuffer;
    uint32_t writeLength;
    uint32_t maxPktSizeOut;
    uint32_t maxPktSizeIn;
    uint32_t endpNumOut;
    uint32_t endpNumIn;
    bool readActive;
    bool writeActive;
};

/* ecp_dev_ctxt->dataQueue(PerPairEndpoint)->8element for each queue. */
struct cy_stc_usb_echo_dev_ctxt_
{
    Cy_IfxQueue_t dataQueue[CY_USB_NUM_ENDP_CONFIGURED];
    uint8_t activeAltSetEndps[12];
};

void  *Cy_USB_EchoDeviceInit(void);
#if FREERTOS_ENABLE
void Cy_USB_EchoDeviceTaskHandler(void *pTaskParam);
#else
void Cy_USB_EchoDeviceTaskHandler (void *pTaskParam, void *qMsg);
#endif /* FREETROS_ENABLE */
void Cy_USB_EchoDeviceDmaReadCompletion(void *pApp, uint8_t endpAddr);
void Cy_USB_EchoDeviceDmaWriteCompletion(void *pApp, uint8_t endpAddr);
void Cy_InitIfxQueue(cy_stc_usb_echo_dev_ctxt_t *pEchoDevCtxt);
void Cy_DevSpeedBasedfxQueueUpdate(void *pApp,
                                   cy_en_usb_speed_t devSpeed);
void Cy_USB_Endp0ReadComplete(void *pApp);
void Cy_USB_EchoDeviceHandleCtrlSetup(void *pApp, cy_stc_usbd_app_msg_t *pMsg);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_ECHO_DEVICE_H_ */

/* End of File */

