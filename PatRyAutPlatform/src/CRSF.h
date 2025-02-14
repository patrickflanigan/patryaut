#ifndef CRSF_H
#define CRSF_H

#include <Arduino.h>
#include <CircularBuffer.hpp>
#include <AceSorting.h>
#include <SoftwareSerial.h>

// -------- Configuration Macros -----------
#define CRSF_PACKET_SIZE             26    // total bytes in a CRSF packet (adjust as needed)
#define CRSF_FRAME_LENGTH            24    // length value expected in the packet header
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8 // start-of-frame value for CRSF FC packets
#define CRSF_MAX_CHANNEL             16
#define CRSF_CHANNEL_MID             1023  // default mid value for channels
#define SERIAL_BAUDRATE              38400 // baud rate used for CRSF communication
#define CRSF_MILLIS_TO_FAILSAFE_ACTIVE 200   // time (ms) to consider controller disconnected
#define SPURIOUS_SIZE                10    // size for circular filtering buffers

// For DEBUG builds, you must also provide a crsfdata2.c file that defines:
//   - a constant MOCK_SIZE (the number of bytes in your mock data)
//   - an array "crsf_mock_data" holding captured CRSF bytes.
// And include "crsfdata2.c" in your project.

#if defined(DEBUG)
  #include "crsfdata2.c"
  // Create a circular buffer for mock data (MOCK_SIZE must be defined in crsfdata2.c)
  extern CircularBuffer<uint8_t, MOCK_SIZE> mockbuff; 
#endif

// -------- Result type for parsing function --------
typedef enum {
    NO_FRAME,
    DATA_STARVED,
    BAD_LENGTH,
    BAD_CRC,
    PACKET_READY
} Result_t;

// -------- CRSF Packet structure --------
typedef union {
  struct {
    uint8_t address;
    uint8_t length;
    uint8_t type;
    uint8_t payload[CRSF_PACKET_SIZE - 4];  // remaining bytes except header (address, length, type) and crc
    uint8_t crc;
  } CRSF_Packet;
  uint8_t packet[CRSF_PACKET_SIZE];
} CRSF_Frame_t;

// -------- The CRSF class -----------
class CRSF {
  public:
    CRSF();

    // Initializes the CRSF system (serial port, internal buffers, etc.)
    void Begin();

    // Returns the value of channel number ch (0-based)
    int16_t Channel(uint8_t ch);

    // Sets a default (calibrated) value for channel ch
    void SetChannelDefault(uint8_t ch, int16_t defaultVal);

    // After a valid packet is received, decode channel values from crsfData
    void UpdateChannels(void);

    // Main packet parser – call this repeatedly in loop()
    Result_t GetCrsfPacket(void);

    // Returns true if the controller is “connected” (good packet received recently)
    bool ControllerConnected(void);

    // Filter a circular buffer of integer values by taking the median (to drop outliers)
    int FilterValues(CircularBuffer<int, SPURIOUS_SIZE> &buff);

    // Reads from the serial port (or from mock data in DEBUG) into our local circular buffer
    int ReadAhead(void);

    // In DEBUG builds, load mock CRSF data into the circular buffer
    void MockCRSFInput();

    // In DEBUG builds, dump a packet (for debugging purposes)
    void DumpFrame(uint8_t *buff, int size);

    // Public arrays for current raw CRSF packet and decoded channels
    uint8_t crsfData[CRSF_PACKET_SIZE];
    int16_t channels[CRSF_MAX_CHANNEL];

    // Timestamp (in millis) when the last good packet was received
    uint32_t lastGoodPacketMillis;

  private:
    CircularBuffer<uint8_t, 256> _circbuff;  // local buffer for incoming serial data
    uint8_t bufferIndex;
    uint8_t feedState;
};

// CRC8 function prototype
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);

#endif
