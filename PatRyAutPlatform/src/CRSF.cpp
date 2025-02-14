#include "CRSF.h"
#include <CircularBuffer.hpp>
#include <AceSorting.h>

// Remove any SoftwareSerial instance and instead use the hardware UART1.
#define SERIAL_PORT Serial1

// Global CRSF frame instance (used for temporary packet storage)
CRSF_Frame_t crsf_frame;

// ---------- CRC8 Table and Function ----------
static uint8_t crsf_crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

// ---------- CRSF Class Implementation ----------

CRSF::CRSF() : bufferIndex(0), feedState(0), lastGoodPacketMillis(0) {
  _circbuff.clear();
}

void CRSF::Begin() {
  uint8_t loc_crsfData[CRSF_PACKET_SIZE] = {0};
  int16_t loc_channels[CRSF_MAX_CHANNEL];
  for (int i = 0; i < CRSF_MAX_CHANNEL; i++) {
    loc_channels[i] = CRSF_CHANNEL_MID;
  }
  
  // Use Serial1 (hardware UART) instead of software serial.
  SERIAL_PORT.begin(SERIAL_BAUDRATE);
  
  memcpy(crsfData, loc_crsfData, CRSF_PACKET_SIZE);
  memcpy(channels, loc_channels, CRSF_MAX_CHANNEL * sizeof(int16_t));
  bufferIndex = 0;
  feedState = 0;
  
  _circbuff.clear();
  
#if defined(DEBUG)
  MockCRSFInput();
#endif
}

int16_t CRSF::Channel(uint8_t ch) {
  return (ch < CRSF_MAX_CHANNEL) ? channels[ch] : CRSF_CHANNEL_MID;
}

void CRSF::SetChannelDefault(uint8_t ch, int16_t defaultVal) {
  if(ch < CRSF_MAX_CHANNEL)
    channels[ch] = defaultVal;
}

void CRSF::UpdateChannels(void) {
  // If the packet length byte equals 24, decode channel data from crsfData.
  if(crsfData[1] == 24) {
    channels[0]  = ((crsfData[3] | (crsfData[4] << 8)) & 0x07FF);
    channels[1]  = (((crsfData[4] >> 3) | (crsfData[5] << 5)) & 0x07FF);
    channels[2]  = (((crsfData[5] >> 6) | (crsfData[6] << 2) | (crsfData[7] << 10)) & 0x07FF);
    channels[3]  = (((crsfData[7] >> 1) | (crsfData[8] << 7)) & 0x07FF);
    channels[4]  = (((crsfData[8] >> 4) | (crsfData[9] << 4)) & 0x07FF);
    channels[5]  = (((crsfData[9] >> 7) | (crsfData[10] << 1) | (crsfData[11] << 9)) & 0x07FF);
    channels[6]  = (((crsfData[11] >> 2) | (crsfData[12] << 6)) & 0x07FF);
    channels[7]  = (((crsfData[12] >> 5) | (crsfData[13] << 3)) & 0x07FF);
    // Decode additional channels as needed:
    channels[8]  = ((crsfData[14] | (crsfData[15] << 8)) & 0x07FF);
    channels[9]  = (((crsfData[15] >> 3) | (crsfData[16] << 5)) & 0x07FF);
    channels[10] = (((crsfData[16] >> 6) | (crsfData[17] << 2) | (crsfData[18] << 10)) & 0x07FF);
    channels[11] = (((crsfData[18] >> 1) | (crsfData[19] << 7)) & 0x07FF);
    channels[12] = (((crsfData[19] >> 4) | (crsfData[20] << 4)) & 0x07FF);
    channels[13] = (((crsfData[20] >> 7) | (crsfData[21] << 1) | (crsfData[22] << 9)) & 0x07FF);
    channels[14] = (((crsfData[22] >> 2) | (crsfData[23] << 6)) & 0x07FF);
    channels[15] = (((crsfData[23] >> 5) | (crsfData[24] << 3)) & 0x07FF);
  }
}

#if defined(DEBUG)
int CRSF::ReadAhead(void) {
  // In DEBUG mode, fill _circbuff from the mock buffer until at least 2 full packets are available.
  while(_circbuff.size() < (2 * CRSF_PACKET_SIZE)) {
    uint8_t nextbyte = mockbuff.shift();
    _circbuff.push(nextbyte);
    mockbuff.push(nextbyte); // make the mock data circular
  }
  return _circbuff.size();
}
#else
int CRSF::ReadAhead(void) {
  // Use SERIAL_PORT (Serial1) to read incoming bytes.
  while(SERIAL_PORT.available() && _circbuff.size() < (2 * CRSF_PACKET_SIZE))
    _circbuff.push( SERIAL_PORT.read() );
  return _circbuff.size();
}
#endif

Result_t CRSF::GetCrsfPacket(void) {
  uint8_t nextbyte = 0x00;
  
  if(ReadAhead() < (2 * CRSF_PACKET_SIZE))
    return DATA_STARVED;
    
  // Look for the start-of-frame byte.
  nextbyte = 0x00;
  while(_circbuff.size() > 0 && (nextbyte != CRSF_ADDRESS_FLIGHT_CONTROLLER)) {
    nextbyte = _circbuff.shift();
  }
  
  if((nextbyte != CRSF_ADDRESS_FLIGHT_CONTROLLER) || (_circbuff.size() < (CRSF_PACKET_SIZE - 1))) {
    if(nextbyte == CRSF_ADDRESS_FLIGHT_CONTROLLER)
      _circbuff.unshift(CRSF_ADDRESS_FLIGHT_CONTROLLER);
    else
      return NO_FRAME;
    return DATA_STARVED;
  }
  
  // Get the packet length byte and check it.
  nextbyte = _circbuff.shift();
  if(nextbyte != CRSF_FRAME_LENGTH)
    return BAD_LENGTH;
  
  int pktidx = 0;
  memset(crsf_frame.packet, 0, CRSF_PACKET_SIZE);
  crsf_frame.packet[pktidx++] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  crsf_frame.packet[pktidx++] = nextbyte;
  
  while(_circbuff.size() > 0 && pktidx < CRSF_PACKET_SIZE)
    crsf_frame.packet[pktidx++] = _circbuff.shift();
  
  if(_circbuff.size() == 0 && pktidx < CRSF_PACKET_SIZE)
    return DATA_STARVED;
    
  // Compute the expected CRC over packet bytes starting at index 2.
  uint8_t crc = crsf_crc8(&crsf_frame.packet[2], CRSF_FRAME_LENGTH - 1);
  
  if(crc != crsf_frame.CRSF_Packet.crc) {
    DumpFrame(crsf_frame.packet, CRSF_PACKET_SIZE);
    return BAD_CRC;
  }
  
  memcpy(crsfData, crsf_frame.packet, CRSF_PACKET_SIZE);
  lastGoodPacketMillis = millis();
  return PACKET_READY;
}

bool CRSF::ControllerConnected() {
  if(millis() < lastGoodPacketMillis)
    lastGoodPacketMillis = millis();
  uint32_t now = millis();
  return ((now - lastGoodPacketMillis) <= CRSF_MILLIS_TO_FAILSAFE_ACTIVE);
}

int CRSF::FilterValues(CircularBuffer<int, SPURIOUS_SIZE> &buff) {
  int modArray[SPURIOUS_SIZE];
  buff.copyToArray(modArray);
  ace_sorting::insertionSort(modArray, SPURIOUS_SIZE);
  return modArray[(SPURIOUS_SIZE / 2) - 1];
}

#if defined(DEBUG)
void CRSF::MockCRSFInput() {
  // In DEBUG, fill the mock buffer (and thereby _circbuff) from captured data.
  static int next_mock_idx = 0;
  while(mockbuff.size() < MOCK_SIZE) {
    // Optionally, print the byte:
    // Serial.print(crsf_mock_data[next_mock_idx], HEX); Serial.print(" ");
    mockbuff.push(crsf_mock_data[next_mock_idx++]);
  }
}

void CRSF::DumpFrame(uint8_t *buff, int size) {
  for(int i = 0; i < size; i++) {
    if(buff[i] < 16) Serial.print("0");
    Serial.print(buff[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
#else
void CRSF::MockCRSFInput() {}
void CRSF::DumpFrame(uint8_t *buff, int size) {}
#endif
