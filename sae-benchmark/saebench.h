
/*
 * Copyright (C) 2021 Regents of University of Colorado
 * Written by Gedare Bloom <gbloom@uccs.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SAEBENCH_H_
#define SAEBENCH_H_

#include "driverlib/can.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

#define VC
//#define BRAKES
//#define DRIVER
//#define BATTERY
//#define IMC
//#define TRANS


/* Benchmark TX Objects */
#if defined(VC)
uint8_t g_ui8TXMsgData_5[] = { 0x99 };
uint8_t g_ui8TXMsgData_10[] = { 0xAF, 0xBE, 0xCD, 0xDC, 0xEB, 0xFA };
uint8_t g_ui8TXMsgData_1000[] = { 0x18 };
#define HIGH_PRIO_ID 0xA0
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xA0, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_5), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5 },
{ .ui32MsgID = 0xB0, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_10), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_10 },
{ .ui32MsgID = 0xD0, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_1000), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_1000 }
};
tCANMsgObject *g_sCAN0TxMessage_5 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_10 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_100 = NULL;
tCANMsgObject *g_sCAN0TxMessage_1000 = &g_sCAN0TxMessages[2];
#elif defined(BRAKES)
uint8_t g_ui8TXMsgData_5[] = { 0x66, 0x7C };
uint8_t g_ui8TXMsgData_10[] = { 0xAF, 0xBC };
uint8_t g_ui8TXMsgData_100[] = { 0xC0 };
#define HIGH_PRIO_ID 0xA1
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xA1, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_5), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5 },
{ .ui32MsgID = 0xB1, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_10), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_10 },
{ .ui32MsgID = 0xC1, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_100), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_100 }
};
tCANMsgObject *g_sCAN0TxMessage_5 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_10 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_100 = &g_sCAN0TxMessages[2];
tCANMsgObject *g_sCAN0TxMessage_1000 = NULL;
#elif defined(DRIVER)
uint8_t g_ui8TXMsgData_5[] = { 0x11 };
uint8_t g_ui8TXMsgData_10[] = { 0xAF, 0xBC };
#define HIGH_PRIO_ID 0xA3
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xA3, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_5), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5 },
{ .ui32MsgID = 0xB3, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_10), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_10 },
};
tCANMsgObject *g_sCAN0TxMessage_5 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_10 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_100 = NULL;
tCANMsgObject *g_sCAN0TxMessage_1000 = NULL;
#elif defined(BATTERY)
// TODO: add the other benchmark ECUs.
#error "Battery ECU not defined"
#elif defined(IMC)
#error "IMC ECU not defined"
#elif defined(TRANS)
#error "Trans ECU not defined"
#else
#error "Need to define a benchmark ECU"
#endif



















#endif
