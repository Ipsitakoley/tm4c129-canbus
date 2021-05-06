
/*
 * Copyright (C) 2019, 2021 Regents of University of Colorado
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

#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

#include "driverlib/can.h"

//----------------------------------------------------------------------------
// Constants, do not change the following
//----------------------------------------------------------------------------
#define ATTACK_NONE         (0x0000)
#define ATTACK_CLASSIC      (0x0001)
#define ATTACK_RECESSIVE    (0x0002)
#define ATTACK_TRANSITIVE   (0x0004)
#define ATTACK_RESERVED1    (0x0008)
#define ATTACK_MASK         (0x000F)

#define SYNC_NONE           (0x0000)
#define SYNC_PERIOD         (0x0010)
#define SYNC_0PHASE         (0x0020)
#define SYNC_RESERVED1      (0x0040)
#define SYNC_RESERVED2      (0x0080)
#define SYNC_MASK           (0x00F0)

#define RESET_NONE          (0x0000)
#define RESET_IMMED         (0x0100)
#define RESET_DELAY         (0x0200)
#define RESET_RESERVED1     (0x0400)
#define RESET_RESERVED2     (0x0800)
#define RESET_MASK          (0x0F00)

#define DISABLE_RETRANS_NONE      (0x0000)
#define DISABLE_RETRANS_AUTO      (0x1000)
#define DISABLE_RETRANS_TXERR     (0x2000)
#define DISABLE_RETRANS_RXPM      (0x4000)
#define DISABLE_RETRANS_RESERVED1 (0x8000)
#define DISABLE_RETRANS_MASK      (0xF000)

#define HIGH_PRIO (3)
#define MID_PRIO  (4)
#define LOW_PRIO  (5)

//----------------------------------------------------------------------------
// Change the rest of this file as needed to define your experimental setup
//----------------------------------------------------------------------------
#define ATTACK      ATTACK_TRANSITIVE
#define SYNC        SYNC_0PHASE
#define RESET       RESET_IMMED
#define RETRANS     DISABLE_RETRANS_TXERR

// this inherits from the above block of defines.
// Don't change this variable, instead change the defines. they get used elsewhere too.
uint32_t g_ui32ExpCtrl = (ATTACK | SYNC | RESET | RETRANS);

// might need to tune these, based on your chosen attack
#define RXPM_DELAY_BITS (20) // how many bit times to delay after RXPM to abort

// the CAN bus bitrate in Mbps
#define BITRATE (500000)

// If using the skipping attack strategy, need to configure how many transmissions to skip.
// This is done by specifying which injection to transmit on, so skip n-1 transmissions, inject on n'th
// To not use skipping strategy, set these to 1.
#define SKIP_ATTACK (1)     // inject on the SKIP_ATTACK'th victim transmission
#define SKIP_ATTACK_2 (2)   // for transitive attacks, second victim injection skip

// Comment/Uncomment the following line to send the reset message, usually only one node does this.
#define SEND_RESET
#define RESET_PERIOD_SECONDS (10)

// Attack Message
#define TARGET_ID (0xA1)
#define PRECEDED_ID (0xA0)  // used in RXPM
#define TARGET_DATALEN (2)

#if (ATTACK == ATTACK_CLASSIC)
const uint8_t g_ui8TXMsgData_Target_1[TARGET_DATALEN] = { 0x46, 0x7C };    // dominant attack
#else
const uint8_t g_ui8TXMsgData_Target_1[TARGET_DATALEN] = { 0xE6, 0x7C };    // recessive attack
#endif

// Transitive Attack Message
#define TARGET_ID_2 (0xA3)
#define PRECEDED_ID_2 (0xA1)  // used in RXPM
#define TARGET_DATALEN_2 (3)
uint8_t g_ui8TXMsgData_Target_2[TARGET_DATALEN_2] = { 0xEE, 0xEE, 0xEE  }; // recessive attack

// Uncomment this to turn on some extra print statements.
// They can mess with timing, recommended to disable except when debugging.
//#define VERBOSE

// You shouldn't need to change these:
#define TARGET_XMIT_TIME ((44+(TARGET_DATALEN*8)*120000)/(BITRATE/1000)) // approx. and subject to stuff bits
#define TARGET_XMIT_TIME_2 ((44+(TARGET_DATALEN_2*8)*120000)/(BITRATE/1000)) // approx. and subject to stuff bits

int8_t g_ui8RXMsgData[8];
tCANMsgObject g_sCAN0RxMessage = { .ui32MsgID = 0, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER, .ui32MsgLen = sizeof(g_ui8RXMsgData), .pui8MsgData = (uint8_t *) &g_ui8RXMsgData };
tCANMsgObject g_sCAN0RxMessage2 = { .ui32MsgID = PRECEDED_ID, .ui32MsgIDMask = PRECEDED_ID, .ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER, .ui32MsgLen = sizeof(g_ui8RXMsgData), .pui8MsgData = (uint8_t *) &g_ui8RXMsgData };
tCANMsgObject g_sCAN0TxMessage_Target_1 = { .ui32MsgID = TARGET_ID, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_Target_1), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_Target_1 };
tCANMsgObject g_sCAN0TxMessage_Target_2 = { .ui32MsgID = TARGET_ID_2, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_Target_2), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_Target_2 };
uint8_t g_ui8TXMsgData_RESET[1] = { 0xFF };
tCANMsgObject g_sCAN0TxMessage_RESET = { .ui32MsgID = 0xFF, .ui32MsgIDMask = 0, .ui32Flags = MSG_OBJ_TX_INT_ENABLE, .ui32MsgLen = sizeof(g_ui8TXMsgData_RESET), .pui8MsgData = (uint8_t *)&g_ui8TXMsgData_RESET };

//----------------------------------------------------------------------------
// You should not need to change the following.
//----------------------------------------------------------------------------
// Determine the relative priorities of the TARGET_ID, TARGET_ID_2, and the highest priority message normally sent.
#if (HIGH_PRIO_ID < TARGET_ID && TARGET_ID < TARGET_ID_2)
#define PRIORITY_Target_1 (MID_PRIO)
#define HIGHEST_TX_PRIORITY (HIGH_PRIO)
#define PRIORITY_Target_2 (LOW_PRIO)
#elif (HIGH_PRIO_ID < TARGET_ID_2 && TARGET_ID_2 < TARGET_ID)
#define PRIORITY_Target_1 (LOW_PRIO)
#define HIGHEST_TX_PRIORITY (HIGH_PRIO)
#define PRIORITY_Target_2 (MID_PRIO)
#elif (TARGET_ID < HIGH_PRIO_ID  && HIGH_PRIO_ID < TARGET_ID_2)
#define PRIORITY_Target_1 (HIGH_PRIO)
#define HIGHEST_TX_PRIORITY (MID_PRIO)
#define PRIORITY_Target_2 (LOW_PRIO)
#elif (TARGET_ID_2 < HIGH_PRIO_ID  && HIGH_PRIO_ID < TARGET_ID)
#define PRIORITY_Target_1 (LOW_PRIO)
#define HIGHEST_TX_PRIORITY (MID_PRIO)
#define PRIORITY_Target_2 (HIGH_PRIO)
#elif (TARGET_ID < TARGET_ID_2)
#define PRIORITY_Target_1 (HIGH_PRIO)
#define HIGHEST_TX_PRIORITY (LOW_PRIO)
#define PRIORITY_Target_2 (MID_PRIO)
#else
#define PRIORITY_Target_1 (MID_PRIO)
#define HIGHEST_TX_PRIORITY (LOW_PRIO)
#define PRIORITY_Target_2 (HIGH_PRIO)
#endif



#endif /* EXPERIMENT_H_ */
