#include "mbed.h"
#include <cstdint>

//AMT21 encoder functions
uint16_t readAMT(RawSerial &ser, uint8_t id, DigitalOut flow_pin);
unsigned char calK0(uint16_t data);
unsigned char calK1(uint16_t data);


