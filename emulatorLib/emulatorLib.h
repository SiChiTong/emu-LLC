#ifndef emulatorLib_h
#define emulatorLib_h

#include "mbed.h"
#include "emulatorPin.h"
#include <cstdint>
#include "FIFO.hpp"
#include "crc.h"
#include "eeprom.h"
#include <cmath>
#define PI 3.14159265359

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
        void        setRatio(float);
        void        setMicro(uint8_t);
        void        ols(float); //open-loop speed rad/s
    private:
        DigitalOut  EN;
        DigitalOut  DIR;
        PwmOut      STEP;
        float       frequency; //Hz
        float       max_frequency;
        float       microstep = 1;
        float       spr = 200.0f; //Step per rev
        float       ratio = 1.0f;
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
        void        kmfInit();
        void        kmfEstimate();
    private:
        RawSerial&  SER;
        DigitalOut  FLOW;
        uint8_t     ID;
        float       ratio = 1.0f;   //Velocity ratio between encoder and destination
        bool        check = 1;      //Allow to calculate checksum
        //Kalman Filter
        int32_t     k_wrap = 0;     
        bool        k_init = 0;
        float       p11, p12, p21, p22;
        float       x_hat_1, x_hat_2;
        float       k_prev_pos;
        float       kdt = 0.005f;   //Kalman Filter sampling time max: 0.0002883 sec(3468.208 Hz)
        float       sigma_a = 0.1f;
        float       sigma_w = 0.008f;
        unsigned char checksum(uint16_t);
};

class Controller{
    public:
        Controller(float, float, float);
        ~Controller();
        void        setKp(float _Kp);
        void        setKi(float _Ki);
        void        setKd(float _Kd);
        float       getKp(void);
        float       getKi(void);
        float       getKd(void);
        void        init();
        float       update(float, float);
        void        setSat(float, float);
        void        unsetSat();
    private:
        float       Kp;
        float       Ki;
        float       Kd;
        float       s, p;
        bool        isInit;
        float       sat = 0;
        float       u_lim;
        float       l_lim;
        float       saturate(float);
};

class Trajectory {
    public:
        Trajectory();
        ~Trajectory();
        void        setGoal(float qi, float qf, float vi, float vf, float duration);
        float       update(float time);
        bool        reached();
        float       getC0();
        float       getC1();
        float       getC2();
        float       getC3();
        float       getQi();
        float       getQf();
        float       getVi();
        float       getVf();
        float       getDuration();
        float       step(float stop_at);
    private:
        float       qi, qf;
        float       vi, vf;
        float       c0,c1,c2,c3;
        float       T;
        bool        atGoal;
};

class Actuator {
    public:
        Actuator(PinName, PinName, PinName, RawSerial&, uint8_t, PinName);
        Actuator(PinName, PinName, PinName, RawSerial&, uint8_t, PinName, float, float, float);
        ~Actuator();
        Stepper     stepper;
        AMT21       encoder;
        Controller  pcon;
        Trajectory  trajectory;
        float       at();
        void        operator=(float);
        void        operator=(int);
        float       update(float);
        void        setPconSat(float, float);
        void        unsetPconSat();
    // private:
};

#endif