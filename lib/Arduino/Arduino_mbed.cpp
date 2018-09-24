/*
The MIT License (MIT)

Copyright (c) 2014 calima engineering

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
// millis as in Arduino...

#include "mbed.h"
#include "Arduino_mbed.h"

#ifdef DEBUG_MILLIS
#if DEBUG_MILLIS
DigitalOut DebugMillisPin(D2);
#endif
#endif

// Period of IRQ in µs. This is the minimum (!) granularity of micros().
#define TICKER_PERIOD_US (50uL)

static volatile uint32_t MilliSeconds_u32, MicroSeconds_u32, MicroSecondsLast_u32;

static  Ticker ArduinoTicker;

static void TickerIRQ()
{
#ifdef DEBUG_MILLIS
#if DEBUG_MILLIS
    DebugMillisPin = not DebugMillisPin;
#endif
#endif
    MicroSeconds_u32 += TICKER_PERIOD_US;
    if ((MicroSeconds_u32 - MicroSecondsLast_u32) >= 1000uL) {
        MilliSeconds_u32++;
        MicroSecondsLast_u32 += 1000uL;
    }
}

uint32_t millis ()
{
    // Not critical since variable is atomic
    return MilliSeconds_u32;
}

uint32_t micros ()
{
    // Not critical since variable is atomic
    return MicroSeconds_u32;
}

void StartArduinoTimer ()
{
    //arduino_timer.start();
    ArduinoTicker.attach_us (&TickerIRQ, TICKER_PERIOD_US);
}

// End of file