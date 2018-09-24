#include <SPI.h>
#include "additional_serial.h"
#include <mbed.h>
typedef unsigned char Byte; 

// Instantiate the extra Serial classes

Serial pc2(PA_9, PA_10); // tx, rx

void serial_setup() {
    //pc.baud(57600);nh.getHardware()->setBaud(115200);
    pc2.baud(2000000);   // Begin Serial2
    //pc3.baud(2000000); // Begin Serial3
}

void serial_motor_controls(int motor_tag,
                           float DIR_PACK,
                           int serial_val)
{
    serial_val = abs(serial_val);
    pc2.putc((Byte)0xAA); // recognize baud rate 
    pc2.putc((Byte)motor_tag); // identify which motor to send packet to 
    pc2.putc((Byte)DIR_PACK); // forwards or backwards 
    pc2.putc(serial_val & 0x1F); // speed packet 1
    pc2.putc(serial_val >> 5); // speed packet 2 
}






