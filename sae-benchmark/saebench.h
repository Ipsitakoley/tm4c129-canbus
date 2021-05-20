
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

//#define VC
//#define BRAKES
//#define DRIVER
#define BATTERY
//#define IMC
//#define TRANS


/* Benchmark TX Objects */
#if defined(VC)
uint8_t g_ui8TXMsgData_5[] = { 0x99 };
uint8_t g_ui8TXMsgData_10[] = { 0xAF, 0xBE, 0xCD, 0xDC, 0xEB, 0xFA };
uint8_t g_ui8TXMsgData_1000[] = { 0x18 };
#define HIGH_PRIO_ID 0xA0
#define TX_5_ID 0xA0
#define TX_10_ID 0xB0
#define TX_100_ID 0x00
#define TX_1000_ID 0xD0
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
uint8_t g_ui8TXMsgData_100[] = { 0xC0 };
#define HIGH_PRIO_ID 0xA1
#define TX_5_ID 0xA1
#define TX_10_ID 0x00
#define TX_100_ID 0xC1
#define TX_1000_ID 0x00
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xA1, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_5), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5 },
{ .ui32MsgID = 0xC1, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_100), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_100 }
};
tCANMsgObject *g_sCAN0TxMessage_5 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_10 = NULL;
tCANMsgObject *g_sCAN0TxMessage_100 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_1000 = NULL;
#elif defined(DRIVER)
uint8_t g_ui8TXMsgData_5[] = { 0x11 };
uint8_t g_ui8TXMsgData_10[] = { 0xAF, 0xBC };
#define HIGH_PRIO_ID 0xA3
#define TX_5_ID 0xA3
#define TX_10_ID 0xB3
#define TX_100_ID 0x00
#define TX_1000_ID 0x00
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xA3, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_5), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5 },
{ .ui32MsgID = 0xB3, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_10), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_10 },
};
tCANMsgObject *g_sCAN0TxMessage_5 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_10 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_100 = NULL;
tCANMsgObject *g_sCAN0TxMessage_1000 = NULL;
#elif defined(BATTERY)
uint8_t g_ui8TXMsgData_10[] = { 0x86 };
uint8_t g_ui8TXMsgData_100[] = { 0x24, 0x33, 0x41, 0xA2 };
uint8_t g_ui8TXMsgData_1000[] = { 0xB0, 0xB2, 0xB4 };
#define HIGH_PRIO_ID 0xB2
#define TX_5_ID 0x00
#define TX_10_ID 0xB2
#define TX_100_ID 0xC2
#define TX_1000_ID 0xD2
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xB2, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_10), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_10 },
{ .ui32MsgID = 0xC2, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_100), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_100 },
{ .ui32MsgID = 0xD2, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_1000), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_1000 }
};
tCANMsgObject *g_sCAN0TxMessage_5 = NULL;
tCANMsgObject *g_sCAN0TxMessage_10 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_100 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_1000 = &g_sCAN0TxMessages[2];
#elif defined(IMC)
uint8_t g_ui8TXMsgData_5[] = { 0xC0, 0x7F };
uint8_t g_ui8TXMsgData_10[] = { 0xC4, 0x1A };
#define HIGH_PRIO_ID 0xA4
#define TX_5_ID 0xA4
#define TX_10_ID 0xB4
#define TX_100_ID 0x00
#define TX_1000_ID 0x00
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xA4, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_5), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5 },
{ .ui32MsgID = 0xB4, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_10), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_10 }
};
tCANMsgObject *g_sCAN0TxMessage_5 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_10 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_100 = NULL;
tCANMsgObject *g_sCAN0TxMessage_1000 = NULL;
#elif defined(TRANS)
uint8_t g_ui8TXMsgData_5[] = { 0xF7 };
uint8_t g_ui8TXMsgData_100[] = { 0x1C };
uint8_t g_ui8TXMsgData_1000[] = { 0xA1 };
#define HIGH_PRIO_ID 0xA5
#define TX_5_ID 0xA5
#define TX_10_ID 0x00
#define TX_100_ID 0xC5
#define TX_1000_ID 0xD5
tCANMsgObject g_sCAN0TxMessages[] = {
{ .ui32MsgID = 0xA5, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_5), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5 },
{ .ui32MsgID = 0xC5, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_100), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_100 },
{ .ui32MsgID = 0xD5, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_1000), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_1000 }
};
tCANMsgObject *g_sCAN0TxMessage_5 = &g_sCAN0TxMessages[0];
tCANMsgObject *g_sCAN0TxMessage_10 = NULL;
tCANMsgObject *g_sCAN0TxMessage_100 = &g_sCAN0TxMessages[1];
tCANMsgObject *g_sCAN0TxMessage_1000 = &g_sCAN0TxMessages[2];
#else
#error "Need to define a benchmark ECU"
#endif



















#endif
