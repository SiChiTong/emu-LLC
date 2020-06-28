#ifndef as5047_h
#define as5047_h
#include "mbed.h"
#include "rtos.h"
#define PI 3.14159265359

class AS5047{
    public:
        AS5047(SPI&, PinName);
        ~AS5047();
        void        setRatio(float);
        float       getRatio(void);
        uint16_t    getPosition(void);
        long        getPositionCon(void);
        float       getPositionRadian(void);
        float       getPositionDegree(void);
        void        continuousPositionLoop(void);
        void        zero(uint16_t);
        void        resetCon();
        long            conpo;

    private:
        SPI&            spi;
        DigitalOut      ssel;
        float           ratio;
        
        uint64_t        wrap = 0;
        unsigned char   calEvenParity(unsigned char);
};


#endif