#ifndef emulatorLib_h
#define emulatorLib_h

#include "mbed.h"
#include <cstdint>

//AMT21 encoder functions
uint16_t readAMT(RawSerial &ser, uint8_t id, DigitalOut flow_pin);
unsigned char calK0(uint16_t data);
unsigned char calK1(uint16_t data);

class Actuator {
    public:
        Actuator(PinName step_pin_in, PinName dir_pin_in, PinName en_pin_in);
        ~Actuator();
        void enable(void);
        void disable(void);
        void hold(void);
        void unHold(void);
        void setFrequency(float);
    private:
        DigitalOut  EN;
        DigitalOut  DIR;
        PwmOut      STEP;
        float       fq; //Hz
};

#endif