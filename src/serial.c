/*
  serial.c - low level functions for transmitting bytes via the serial port

  Driver code for simulator MCU

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>
#include <stdatomic.h>

#include "mcu.h"
#include "simulator.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static void uart_interrupt_handler (void);

//
// serialGetC - returns -1 if no data available
//
static int32_t serialGetC (void)
{
    int32_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    atomic_thread_fence(memory_order_acquire);      // Data written before head was published (UART ISR runs on the simulator thread)

    data = (int32_t)rxbuffer.data[bptr++];          // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return data;
}

static inline uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

static void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}

static void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

static bool serialPutC (const uint8_t c)
{
    uint_fast16_t next_head;

    // NOTE: If buffer and transmit register are empty buffering may be bypassed.
    //       See actual drivers for examples.

    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

    while(txbuffer.tail == next_head) {                         // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
    atomic_thread_fence(memory_order_release);                  // making sure it is visible before the head pointer (UART ISR reads it on the simulator thread)
    txbuffer.head = next_head;                                  // and update head pointer

    uart.tx_irq_enable = 1;                                     // Enable TX interrupts

    return true;
}

static void serialWriteS (const char *data)
{
    uint8_t c, *ptr = (uint8_t *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static uint16_t serialTxCount(void) {

    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

/* The MPG stream's registration refuses any stream that cannot be told to stop receiving, and on
 * a controller that is the point of a claim: the primary goes deaf while an app is in control.
 * There is one stream here - the bench has one sender and it is wired to this port - so there is
 * no second receiver to protect and nothing to switch off. It is answered rather than left out,
 * because without it stream_mpg_register refuses the stream and the simulator cannot be put into
 * MPG mode at all. */
static bool serialDisableRx (bool state)
{
    (void)state;

    /* Reports the request as applied. Nothing here is switched off - the one port serves both
     * roles - and saying so is what lets a claim succeed: a caller that checks this rather than
     * ignoring it would otherwise refuse the request and MPG mode with it. */
    return true;
}

/* The realtime handler the MPG stream is registered with, and with it the place a claim is
 * recognised: CMD_MPG_MODE_TOGGLE turns MPG mode on, and turning it on again gives control back.
 * The core's own handler cannot be used here, because it routes status reports to the MPG port -
 * and on this simulator the MPG port is the port the panel is already talking on, so its reports
 * would be written back into the same stream and never reach the panel. Everything except the
 * toggle goes to the core's parser, which is what the primary's handler would have done with it.
 *
 * The mode is changed here rather than scheduled, which is what a controller does with it: there
 * the handler runs in the receive interrupt and cannot touch the stream from it, while this one is
 * called from the simulator's own main loop (mcu.c feeds the emulated UART from it) and can. */
bool simMpgCheckEnable (const uint8_t c)
{
    if(c != CMD_MPG_MODE_TOGGLE)
        return protocol_enqueue_realtime_command(c);

    stream_mpg_enable(!hal.stream.state.is_mpg);

    return true;
}

/* The core accumulates realtime report flags on the MPG port (mpg_rt_report_add, in grbl/stream.c),
 * and on a controller that is where a panel's reports come from - the MPG: element reaches it from
 * there. Here the panel is on this one port, so the flags it has to see are this stream's. Chained
 * rather than replaced: the core's own hook still does its own accumulating. */
static on_rt_reports_added_ptr previous_report_add = NULL;

static void simMpgReportAdd (report_tracking_flags_t report)
{
    if(previous_report_add)
        previous_report_add(report);

    if(report.value == 0)
        hal.stream.report.flags.value = 0;
    else
        hal.stream.report.flags.value |= report.value;
}

void simMpgReportHookInstall (void)
{
    previous_report_add = grbl.on_rt_reports_added;
    grbl.on_rt_reports_added = simMpgReportAdd;
}

const io_stream_t *serialInit (void)
{
        static const io_stream_t stream = {
        .type = StreamType_Serial,
		.is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_enqueue_rt_handler = serialSetRtHandler,
        .disable_rx = serialDisableRx
    };

    mcu_register_irq_handler(uart_interrupt_handler, UART_IRQ);

    uart.rx_irq_enable = 1;
    
    return &stream;
}

static void uart_interrupt_handler (void)
{
    uint_fast16_t bptr;
    int32_t data;

    if(uart.tx_irq) {

        bptr = txbuffer.tail;

        if(txbuffer.head != bptr) {

            atomic_thread_fence(memory_order_acquire);        // Data written before head was published (serialPutC runs on the grbl thread)

            uart.tx_data =  txbuffer.data[bptr++];            // Put character in TXT register
            bptr &= (TX_BUFFER_SIZE - 1);                     // and update tmp tail pointer
            uart.tx_irq = 0;
            uart.tx_flag = 1;

            txbuffer.tail = bptr;                             //  Update tail pinter

            if(bptr == txbuffer.head)                         // Disable TX interrups
                uart.tx_irq_enable = 0;                       // when TX buffer empty
        }
    }

    if(uart.rx_irq) {

        bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

        if(bptr == rxbuffer.tail) {                         // If buffer full
            rxbuffer.overflow = 1;                          // flag overflow and
            uart.rx_irq = 0;                                // clear interrupt flag
        } else {
            data = uart.rx_data;
            uart.rx_irq = 0;
            if(data == 0x06)
                sim.exit = exit_REQ;
            if(!enqueue_realtime_command((uint8_t)data)) {
                rxbuffer.data[rxbuffer.head] = (uint8_t)data;  	// Add data to buffer
                atomic_thread_fence(memory_order_release);      // making sure it is visible before the head pointer (serialGetC reads it on the grbl thread)
                rxbuffer.head = bptr;                       	// and update pointer
            }
        }
    }
}
