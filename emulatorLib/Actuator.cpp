#include "emulatorLib.h"

Actuator::Actuator(PinName _STEP, PinName _DIR, PinName _EN, int8_t _dir, RawSerial &_SER, uint8_t _ID, PinName _FLOW):
stepper(_STEP, _DIR, _EN),
encoder(_SER, _ID, _FLOW),
pcon(0.0f, 0.0f, 0.0f),
trajectory()
{
    this->dir = _dir;
    if (this->dir == 2 || this->dir == -2){
        this->encoder.continuous = 1;
    }else{
        this->encoder.continuous = 0;
    }
}

Actuator::Actuator(PinName _STEP, PinName _DIR, PinName _EN, int8_t _dir, RawSerial &_SER, uint8_t _ID, PinName _FLOW, float _Kp, float _Ki, float _Kd):
stepper(_STEP, _DIR, _EN),
encoder(_SER, _ID, _FLOW),
pcon(_Kp, _Ki, _Kd),
trajectory()
{
    this->dir = _dir;
    if (this->dir == 2 || this->dir == -2){
        this->encoder.continuous = 1;
    }else{
        this->encoder.continuous = 0;
    }
    pcon.init();
}

Actuator::~Actuator(){

}

void Actuator::operator=(float fd){
    this->stepper.ols(this->dir*fd);
}

void Actuator::operator=(int fd){
    this->stepper.ols(this->dir*fd);
}

float Actuator::at(){
    float ang;
    if (this->dir == -1)
        ang = this->dir*(this->encoder.position()-2*PI);
    if (this->dir == -2)
        ang = this->dir*(this->encoder.position()-2*PI);
    if (this->dir == 1)
        ang = this->encoder.position();
    if (this->dir == 2)
        ang = this->encoder.position();
    if (this->dir == 2 || this->dir == -2)
        return ang;
    if (ang >= 0 && ang <= PI)
        return ang;
    else
        return ang-2*PI;
    

}

float Actuator::vat(){
    return this->dir*this->encoder.velocity();
}

float Actuator::update(float setpoint, float ff, float fb, uint8_t conType){
    float u;
    if (conType == 0)
        u =this->pcon.updatePID(setpoint, fb);
    else if (conType == 1)
        u =this->pcon.updatePIDF(setpoint, ff, fb);
    this->stepper.ols(this->dir*u);
    return u;
}

void Actuator::setPconSat(float _l_lim, float _u_lim){
    this->pcon.setSat(_l_lim, _u_lim);
}

void Actuator::unsetPconSat(){
    this->pcon.unsetSat();
}