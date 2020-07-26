#ifndef emulatorLib_h
#define emulatorLib_h

#include "mbed.h"
#include "emulatorPin.h"
#include <cstdint>
#include <cstring>
#include "FIFO.hpp"
#include <deque> 
#include "crc.h"
#include "eeprom.h"
#include <cmath>
#define PI 3.14159265359

class Emuart{
    public:
        Emuart(RawSerial& serialObject, unsigned int bufferSize);
        Emuart(RawSerial& serialObject, unsigned int bufferSize, float SamplingTime);
        ~Emuart();
        void            init(void);
        uint8_t         command;
        uint8_t         data[256]; //command will be added at dataLen+1 when calculate checksum
        uint8_t         dataLen; //Excluding start,dataLen
        void            clearData(void);
        void            clearAll(void);
        int             parse(void);
        void            write(uint8_t command, uint8_t dataLen, uint8_t* dataBuffer);
        void            write(uint8_t command);
        void            print(char* string);
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
        Stepper(PinName pulsePin, PinName directionPin, PinName enablePin);
        ~Stepper();
        void        enable(void);
        void        disable(void);
        void        setFrequency(float frequency);
        void        setFrequency(int frequency);
        void        setMaxFrequency(float max_frequency);
        void        setRatio(float reducingRatio);
        void        setMicro(uint8_t microstepDen);
        void        setDir(int8_t setDefaultDirection);
        void        ols(float speed); //open-loop speed rad/s
    private:
        DigitalOut  EN;
        DigitalOut  DIR;
        PwmOut      STEP;
        int8_t      dir;
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
        void        setRatio(float ratio);
        uint8_t     getID();
        void        setChecksum(bool checksumRequirement);
        bool        getChecksum();
        int16_t    read();
        int16_t    read(uint8_t);
        double      readPosition();
        double      position();     //Filtered position
        float       velocity();     //Filtered position
        //Kalman Filter
        void        setKdt(float kalmanDt);
        float       getKdt();
        void        setSigmaW(float covariance);
        float       getSigmaW();
        void        setSigmaA(float covariance);
        float       getSigmaA();
        void        kmfInit();
        void        kmfEstimate();
        bool        continuous = 0;
    private:
        RawSerial&  SER;
        DigitalOut  FLOW;
        uint8_t     ID;
        float       ratio = 1.0f;   //Velocity ratio between encoder and destination
        bool        check = 1;      //Allow to calculate checksum
        int32_t     k_wrap = 0;
        int8_t      initial_pose = 0;
        //Kalman Filter     
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
        void        init(void);
        float       updatePID(float setPoint, float feedback);
        float       updatePIDF(float setPoint, float feedforward, float feedback);
        void        setSat(float lb, float ub);
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
        float       saturate(float saturation);
};

class Trajectory {
    public:
        Trajectory();
        ~Trajectory();
        void        setGoal(float qi, float qf, float vi, float vf, float duration);
        void        setViaPoints(std::deque <float> _qr, std::deque <float> _vr, std::deque <float> _Tvec);
        float       getPosTraj(float time);
        float       getVelTraj(float time);
        float       getTime();
        bool        reached();
        float       getC0();
        float       getC1();
        float       getC2();
        float       getC3();
        float       getQi();
        float       getQf();
        float       getVi();
        float       getVf();
        uint8_t     pointLeft();
        float       getDuration();
        float       step(float stop_at);
    private:
        void                        nextSub();
        float                       qi, qf;
        float                       vi, vf;
        float                       c0,c1,c2,c3, T;
        std::deque <float>          qr, vr;
        std::deque <float>          Tvec;
        float                       Tlat;
        bool                        atGoal;
};

class Actuator {
    public:
        Actuator(PinName pulsePin, PinName directionPin, PinName enablePin, int8_t defaultDirection, RawSerial& serialObject, uint8_t encoderID, PinName flowControlPin);
        Actuator(PinName pulsePin, PinName directionPin, PinName enablePin, int8_t defaultDirection, RawSerial& serialObject, uint8_t encoderID, PinName flowControlPin, float Kp, float Ki, float Kd);
        ~Actuator();
        Stepper     stepper;
        AMT21       encoder;
        Controller  pcon;
        Trajectory  trajectory;
        float       at();
        float       vat();
        void        operator=(float speed);
        void        operator=(int speed);
        float       update(float setPoint, float feedforward, float feedback, uint8_t controllerType);
        void        setPconSat(float lb, float ub);
        void        unsetPconSat();
    private:
        int8_t      dir;
};

#endif