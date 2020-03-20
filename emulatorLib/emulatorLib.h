#ifndef emulatorLib_h
#define emulatorLib_h

#include "mbed.h"
#include <cstdint>

class Stepper {
    public:
        Stepper(PinName, PinName, PinName);
        ~Stepper();
        void enable(void);
        void disable(void);
        void hold(void);
        void unHold(void);
        void setFrequency(float);
    private:
        DigitalOut  EN;
        DigitalOut  DIR;
        PwmOut      STEP;
        float       frequency; //Hz
};

class AMT21{
    public:
        AMT21(RawSerial&, uint8_t, PinName);
        ~AMT21();
        uint8_t getID();
        void setID(uint8_t);
        uint16_t read();
        uint16_t read(uint8_t);
    private:
        RawSerial&  SER;
        DigitalOut  FLOW;
        uint8_t     ID;
        unsigned char calK0(uint16_t);
        unsigned char calK1(uint16_t);
};

class Actuator {
    public:
        Actuator(PinName, PinName, PinName, RawSerial&, uint8_t, PinName);
        ~Actuator();
        Stepper stepper;
        AMT21 encoder;
        uint16_t at();
        void operator=(float);
    // private:
};

#endif