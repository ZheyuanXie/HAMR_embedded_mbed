#ifndef MBED_MLX90363_H
#define MBED_MLX90363_H
#include "mbed.h"
#include "inttypes.h"
/** MLX90363 class.
 *  Used for communication with the MLX90363 with SPI
 */
class MLX90363
{
public:
    MLX90363(PinName SlaveSelect, SPI* spi);
    bool Poll();
    bool NOP();
    int16_t Get_Angel();
    uint8_t* receiveBufferPt();
private:
//SPI
    SPI* spi;
    bool Send_SPI();
//Shared
    DigitalOut _SlaveSelect;
    uint8_t receiveBuffer[8];
    uint8_t sendBuffer[8];
    uint8_t crc;
    int16_t axis_Value;
    int i,j;
    bool Checksum(uint8_t *message);
};

#endif //MBED_MLX90363_H