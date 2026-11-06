/*
 * Copyright (c) 2007-2015 Freescale Semiconductor, Inc.
 * Copyright 2018-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FreeMASTER Communication Driver - USB Example Implementation
 */

#ifndef __FREEMASTER_EXAMPLE_USB_H
#define __FREEMASTER_EXAMPLE_USB_H
#include "stdint.h"
#define USB_DATA_BUFF_SIZE    64


/******************************************************************************
 * Functions definitions
 ******************************************************************************/
uint8_t FMSTR_ExampleUsbInit(void);
void FMSTR_ExampleUsbLoop(void);

#endif /* __FREEMASTER_EXAMPLE_USB_H */
