#include "MLX90363.h"

MLX90363::MLX90363(PinName SlaveSelect, SPI* spi) : _SlaveSelect(SlaveSelect)
{
    this->spi = spi;
    _SlaveSelect = 1;
}

uint8_t* MLX90363::receiveBufferPt()
{
	return receiveBuffer;
}

int16_t MLX90363::Get_Angel()
{
    axis_Value = (uint16_t)(receiveBuffer[0] | ((receiveBuffer[1]&0x3F)<<8));
	if (axis_Value >= 8192)
		axis_Value -= 16384;
	return axis_Value;
}

bool MLX90363::Poll()
{
    memset(&sendBuffer,0,sizeof(uint8_t)*8);
    sendBuffer[2] = 0xFF;
    sendBuffer[3] = 0xFF;
    //sendBuffer[6] = 0x00 | MELEXIS_GET3;
	sendBuffer[6] = 0x13;
    return Send_SPI();
}

bool MLX90363::NOP()
{
    memset(&sendBuffer,0,sizeof(uint8_t)*8);
    sendBuffer[2] = 0xAA;
    sendBuffer[3] = 0xAA;
    sendBuffer[6] = 0xD0;
    return Send_SPI();
}

bool MLX90363::Send_SPI()
{
    Checksum(sendBuffer);
    _SlaveSelect = 0;
    // wait_ms(5);
    for (i=0; i<8; i++)
	{
		receiveBuffer[i] = spi->write(sendBuffer[i]);
	}
    // wait_ms(5);
    _SlaveSelect = 1;
    return Checksum(receiveBuffer);
}

bool MLX90363::Checksum(uint8_t *message)
{
    crc = message[7];
    message[7] = 0xFF;
    for (j=0; j<7; j++)
        message[7] = cba_256_TAB[ message[j] ^ message[7] ];
    message[7] = ~message[7];
    return !(message[7]==crc);
}