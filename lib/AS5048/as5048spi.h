#include "mbed.h"
typedef enum {
    AS_FLAG_PARITY = 0x8000,
    AS_FLAG_READ = 0x4000,
} As5048Flag;

typedef enum {
    AS_CMD_NOP = 0x0000,
    AS_CMD_ERROR = 0x0001 | AS_FLAG_READ,       // Reads error register of sensor and clear error flags
    AS_CMD_DIAGNOSTICS = 0x3FFD |  AS_FLAG_READ, // Reads automatic gain control and diagnostics info
    AS_CMD_MAGNITUDE = 0x3FFE | AS_FLAG_READ,
    
    AS_CMD_ANGLE = 0x3FFF| AS_FLAG_PARITY | AS_FLAG_READ,
} As5048Command;

// Masks for bits in the result of the AS_CMD_DIAGNOSTICS command
typedef enum {
    AS_DIAG_CORDIC_OVERFLOW = 0x0200,
    AS_DIAG_HIGH_MAGNETIC = 0x0400,
    AS_DIAG_LOW_MAGNETIC = 0x0800,
} As5048Diagnostics;


//! Class for interfacing with the AMS AS5048A magnetic rotary sensor over the SPI-interface.
class As5048Spi
{
public:
    As5048Spi(PinName mosi, PinName miso, PinName sclk, PinName chipselect, int nDevices = 1);
    ~As5048Spi();
    
    bool error(int device = -1);
    
    /// Sets the SPI clock frequency in Hz. Maximum tested frequency is 10MHz.
    void frequency(int frequency = 1000000);
    
    /// Sends a read command to the sensor.
    const int* read(As5048Command command);
    
    /// Sends a read command to the sensor. 
    /// A call to this function will not directly return the requested value. The
    /// requested value will be returned in a next read_sequential call. 
    /// Use this function to read sensor values with minimum speed impact on SPI-bus
    /// and microcontroller.
    const int* read_sequential(As5048Command command);
    
    /// Performs a single angle measurement on all sensors
    /// @return Array of raw angle data. To get the 14-bit value representing
    ///     the angle, apply the mask() to the result.
    const int* read_angle();
    
    /// Performs sequential angle measurements on all sensors. The first time this
    /// method is called the result is not usefull, the measurement data of the call
    /// will be returned by the next call to this method.
    /// @return Array of raw angle data. To get the 14-bit value representing
    ///     the angle, apply the mask() to the result.
    const int* read_angle_sequential();

    /// Returns lowest 14-bits
    static int mask(int sensor_result);

    /// Applies the mask to the first n bytes in the read buffer (for daisychained sensors).
    static void mask(int* sensor_results, int n);
    
    /// Checks if the return value from the sensor has the right parity
    /// @return true if ok
    static bool parity_check(int sensor_result);
    
    /// Returns an angle from 0 to 36000 (degrees times 100).
    /// @param sensor_result is one of the values returned by read_angle or read_angle_sequential
    static int degrees(int sensor_result);
    
    /// Returns an angle from 0 to 2*PI*100 
    /// @param sensor_result is one of the values returned by read_angle or read_angle_sequential
    static int radian(int sensor_result);
    

protected:
    int _nDevices;
    DigitalOut _chipSelectN;
    SPI _spi;
    
    int* _readBuffer; // Stores the results of the last sequential read
    
    int* _read(As5048Command command);
};
    