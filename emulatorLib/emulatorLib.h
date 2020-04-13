#ifndef emulatorLib_h
#define emulatorLib_h

#include "mbed.h"
#include "emulatorPin.h"
#include <cstdint>
#include <Eigen/Dense.h>
#include "FIFO.hpp"
#include "crc.h"
#include "eeprom.h"
#include <cmath>
#define PI 3.14159265359

using namespace Eigen;

class Emuart{
    public:
        Emuart(RawSerial&, unsigned int);
        Emuart(RawSerial&, unsigned int, float);
        ~Emuart();
        void            init(void);
        uint8_t         command;
        uint8_t         data[256]; //command will be added at dataLen+1 when calculate checksum
        uint8_t         dataLen; //Excluding start,dataLen
        void            clearData(void);
        void            clearAll(void);
        int             parse(void);
        void            write(uint8_t, uint8_t, uint8_t*);
        void            write(uint8_t);
        void            print(char*);
        void            clearInputBuffer(void);
        void            setBufferSize(unsigned int size);
        unsigned int    getBufferSize(void);
        void            setSamplingTime(float Ts);
        float           getSamplingTime(void);
    private:
        RawSerial&      SER;
        FIFO<uint8_t>   fifo;
        unsigned int    bufferSize;
        unsigned int    samplingTime;
        uint32_t        checksum; //crc32
        void            rxCallback(void);
};

class JointState{
    public:
        JointState(){}
        JointState(double initial_pos, float initial_vel){
            this->q = initial_pos;
            this->qd = initial_vel;
        }
        ~JointState(){}
        void    positionIsAt(double pos){this->q = pos;}
        void    velocityIsAt(float vel){this->qd = vel;}
        double  position(){return q;}
        float   velocity(){return qd;}
    private:
        double  q = 0.0;
        float   qd = 0.0f;
};

class Stepper {
    public:
        Stepper(PinName, PinName, PinName);
        ~Stepper();
        void        enable(void);
        void        disable(void);
        void        setFrequency(float);
        void        setFrequency(int);
        void        setMaxFrequency(float);
    private:
        DigitalOut  EN;
        DigitalOut  DIR;
        PwmOut      STEP;
        float       frequency; //Hz
        float       max_frequency;
};

class AMT21{
    public:
        AMT21(RawSerial&, uint8_t, PinName);
        ~AMT21();
        JointState  state;
        void        setID(uint8_t);
        uint8_t     getID();
        void        setChecksum(bool);
        bool        getChecksum();
        uint16_t    read();
        uint16_t    read(uint8_t);
        double      readPosition();
        double      position();     //Filtered position
        float       velocity();     //Filtered position
        //Kalman Filter
        void        setKdt(float);
        float       getKdt();
        void        setSigmaW(float);
        float       getSigmaW();
        void        setSigmaA(float);
        float       getSigmaA();
        void        setCov1(float);
        float       getCov1();
        void        setCov2(float);
        float       getCov2();
        void        kmfInit();
        void        kmfEstimate();
    private:
        RawSerial&  SER;
        DigitalOut  FLOW;
        uint8_t     ID;
        float       ratio = 1.0f;   //Velocity ratio between encoder and destination
        bool        check = 1;      //Allow to calculate checksum
        //Kalman Filter
        bool        k_init = 0;
        float       cov1 = 1.0f;
        float       cov2 = 1.0f;
        float       k_prev_pos;
        float       kdt = 0.005f;   //Kalman Filter sampling time max: 0.0002883 sec(3468.208 Hz)
        float       sigma_a = 0.1f;
        float       sigma_w = 0.008f;
        float       Q, R;
        Eigen::Matrix2d    Pp;
        Eigen::Vector2d    x_hat, _x_hat, K, G;
        Eigen::Matrix2d    F;
        Eigen::RowVector2d H;
        unsigned char checksum(uint16_t);
};

class Actuator {
    public:
        Actuator(PinName, PinName, PinName, RawSerial&, uint8_t, PinName);
        ~Actuator();
        Stepper     stepper;
        AMT21       encoder;
        uint16_t    at();
        void        operator=(float);
        void        operator=(int);
    // private:
};

#endif