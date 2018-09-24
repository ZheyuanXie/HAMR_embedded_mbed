#include "as5048spi.h"

As5048Spi::As5048Spi(PinName mosi, PinName miso, PinName sclk, PinName chipselect, int ndevices) :
    _nDevices(ndevices),
    _chipSelectN(chipselect),
    _spi(mosi, miso, sclk)
{
    _chipSelectN.write(1);
    // AS5048 needs 16-bits for is commands
    // Mode = 1: 
    //  clock polarity = 0 --> clock pulse is high
    //  clock phase = 1 --> sample on falling edge of clock pulse
    _spi.format(16, 1);
    
    // Set clock frequency to 1 MHz (max is 10Mhz)
    _spi.frequency(1000000);
    
    _readBuffer = new int[ndevices];
}

As5048Spi::~As5048Spi()
{
    delete [] _readBuffer;
}

int As5048Spi::degrees(int sensor_result)
{
    return mask(sensor_result) * 36000 / 0x4000;
}


int As5048Spi::radian(int sensor_result)
{
    return mask(sensor_result) * 62832 / 0x4000;
}

bool As5048Spi::error(int device)
{
    if( device == -1 ) {
        for(int i = 0; i < _nDevices; ++i) {
            if( _readBuffer[i] & 0x4000 ) {
                return true;
            }
        }
    } else if( device < _nDevices ) {
        return (_readBuffer[device] & 0x4000) == 0x4000;
    }
    return false; 
}


void As5048Spi::frequency(int hz) 
{
    _spi.frequency(hz);
}

int As5048Spi::mask(int sensor_result)
{
    return sensor_result & 0x3FFF; // return lowest 14-bits
}


void As5048Spi::mask(int* sensor_results, int n)
{
    for(int i = 0; i < n; ++i) {
        sensor_results[i] &= 0x3FFF;
    }
}


bool As5048Spi::parity_check(int sensor_result)
{
    // Use the LSb of result to keep track of parity (0 = even, 1 = odd)
    int result = sensor_result;
    
    for(int i = 1; i <= 15; ++i) {
        sensor_result >>= 1;
        result ^= sensor_result;
    }
    // Parity should be even
    return (result & 0x0001) == 0;
}

const int* As5048Spi::read(As5048Command command)
{
    _read(command); // Send command to device(s)
    return _read(AS_CMD_NOP); // Read-out device(s)
}

const int*  As5048Spi::read_sequential(As5048Command command)
{
    return _read(command);
}

const int*  As5048Spi::read_angle()
{  
    _read(AS_CMD_ANGLE); // Send command to device(s)
    return _read(AS_CMD_NOP); // Read-out device(s)
}

const int*  As5048Spi::read_angle_sequential()
{
    return _read(AS_CMD_ANGLE); 
}


int* As5048Spi::_read(As5048Command command)
{
    if(_nDevices == 1)
    {
        // Give command to start reading the angle
        _chipSelectN.write(0);
        wait_us(1); // Wait at least 350ns after chip select
        _readBuffer[0] = _spi.write(command);
        _chipSelectN.write(1);
        wait_us(1); // Wait at least 350ns after chip select       
    } else
    {
        // Enable the sensor on the chain
        _chipSelectN.write(0);
         wait_us(1); // Wait at least 350ns after chip select
        for(int i = 0; i < _nDevices; ++i)
        {
            _readBuffer[i] = _spi.write(command);
        }
        _chipSelectN.write(1);
        wait_us(1); // Wait at least 350ns after chip select
    }
    return _readBuffer;
}





    