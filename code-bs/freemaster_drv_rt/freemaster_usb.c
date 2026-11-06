/*
 * Copyright (c) 2007-2015 Freescale Semiconductor, Inc.
 * Copyright 2018-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FreeMASTER Communication Driver - USB Example Implementation
 */

#include "freemaster.h"
#include "freemaster_serial_usb.h"

#include <stdio.h>
#include <stdlib.h>

#include "freemaster_usb.h"
#include "tusb.h"


/******************************************************************************
 * Local functions
 ******************************************************************************/

static FMSTR_BOOL _FMSTR_UsbTxFunction(FMSTR_U8 *data, FMSTR_U32 size);

/******************************************************************************
 * Type definitions
 ******************************************************************************/

/******************************************************************************
 * Local variables
 ******************************************************************************/

/* Shared Rx/Tx buffer used to exchange data with FreeMASTER driver */
static uint8_t s_usbRxTxBuff[USB_DATA_BUFF_SIZE];

/******************************************************************************
 * API functions
 ******************************************************************************/

/* Initializiation of the USB stack */
uint8_t FMSTR_ExampleUsbInit(void)
{

    FMSTR_SerialUsbRegisterAppFunctions(_FMSTR_UsbTxFunction);

    return 0;
}

void FMSTR_ExampleUsbLoop(void)
{
    uint32_t len; 

    tud_task();

    if(tud_cdc_available())
    {
        len = tud_cdc_read(s_usbRxTxBuff, USB_DATA_BUFF_SIZE);

        FMSTR_SerialUsbProcessEvent(FMSTR_SERIAL_USBCDC_EVENT_TYPE_RECEIVED, (FMSTR_U8 *)s_usbRxTxBuff,
                                    len);
    }
}

/******************************************************************************
 * Local functions
 ******************************************************************************/

/* This function is called from FreeMASTER, when any data to send. */
static FMSTR_BOOL _FMSTR_UsbTxFunction(FMSTR_U8 *data, FMSTR_U32 size)
{
    uint32_t len;

    /* Check if sane data used */
    if (!data || !size || size > USB_DATA_BUFF_SIZE)
        return -1;
    /* Check if USB attached */
    if (!data || !size || size > USB_DATA_BUFF_SIZE )
        return -2;

    /* We use the shared rx/tx memory to make sure the cached/uncached memory is used.
       FreeMASTER uses half-duplex communication anyway. */
    memcpy(s_usbRxTxBuff, data, size);

    /* Send data via USB CDC */
    len = tud_cdc_write(s_usbRxTxBuff, size);
    if (len != size)
        return -3;
    
    tud_cdc_write_flush();
    FMSTR_SerialUsbProcessEvent(FMSTR_SERIAL_USBCDC_EVENT_TYPE_SENT, NULL, 0);
    

    return 0;
}
