#ifndef platform_h

#ifdef  PLATFORM_LINUX
#include "platform_linux.h"
#elif  PLATFORM_OSX
#include "platform_osx.h"
#else
#include "platform_windows.h"
#endif

#define platform_h
#include <inttypes.h>

// On the host, the grbl thread and the simulator thread share one (or few) real CPU cores. grbl's
// main loop would otherwise run at full host speed and burn a whole core, so the grbl thread waits on
// the handshake below and the simulator thread wakes it when there is something for it to do. Waiting
// does NOT change simulation timing: the simulated tick at which every event fires is driven by the
// simulator thread's clock and ISRs, not by how often the grbl thread polls.
//
// sim_yield() is still what to use for a wait that is expected to end within the same timeslice, such
// as mcu_disable_interrupts() waiting for an ISR already running on the other thread.
#ifndef WIN32
#include <sched.h>
#define sim_yield() sched_yield()
#else
// SwitchToThread() hands the rest of the timeslice to another ready thread
// (the simulator thread) and returns immediately when none is waiting - the
// Windows equivalent of sched_yield(). windows.h comes in via platform.h.
#define sim_yield() SwitchToThread()
#endif

void platform_init();
void platform_terminate();

// Set up the handshake between the two threads. Called from platform_init(), before either starts.
void platform_wake_init();

// Wake the grbl thread: something has happened that its main loop has to look at. Signals coalesce,
// so several raised before it runs are the one run they are all asking for.
void platform_wake_signal();

// Block until woken. Returns at once when a signal is already pending, so a signal raised between the
// caller's decision to wait and the wait itself is not lost.
void platform_wake_wait();

plat_thread_t* platform_start_thread(plat_threadfunc_t func);
void platform_stop_thread(plat_thread_t* thread);
void platform_kill_thread(plat_thread_t* thread);

uint32_t  platform_ns();  //monotonically increasing nanoseconds since program start.
void platform_sleep(long microsec); //sleep for suggested time in microsec.

uint8_t platform_poll_stdin(); //non-blocking stdin read - returns 0 if no char present, 0xFF for EOF

// Open a serial device as the simulator's link instead of stdin or a socket, so a board can be wired
// straight to it with a USB-serial adapter. Raw 115200 8N1 with no flow control, which is what the
// grblHAL link is; bytes are read as they arrive and written as they are produced.
void    platform_serial_open(const char *device);
uint8_t platform_poll_serial();            //non-blocking device read - 0 if no char present, 0xFF for EOF
void    platform_serial_out(uint8_t data); //one byte, written when the simulator produces it

#endif
