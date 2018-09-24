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

#ifndef ARDUINO_H_2
#define ARDUINO_H_2

#include "mbed.h"

// Get millis value similar to arduino
uint32_t millis ();

// Get micros value similar to arduino
uint32_t micros ();

// Must be called to start timer!
void StartArduinoTimer ();

/** Macro for delay() 
 *
 * @param void
 */
#define delay(x)                  (wait_ms(x))
/** Macro for delayMicroseconds()  
 *
 * @param void
 */
#define delayMicroseconds(x)      (wait_us(x))
 
#endif  // See #ifndef ARDUINO_H_
// End of file