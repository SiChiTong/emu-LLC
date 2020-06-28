#include "emulatorLib.h"

Actuator::Actuator(PinName _STEP, PinName _DIR, PinName _EN, RawSerial &_SER, uint8_t _ID, PinName _FLOW):
stepper(_STEP, _DIR, _EN),
encoder(_SER, _ID, _FLOW),
pcon(0.0f, 0.0f, 0.0f),
trajectory()
{

}

Actuator::Actuator(PinName _STEP, PinName _DIR, PinName _EN, RawSerial &_SER, uint8_t _ID, PinName _FLOW, float _Kp, float _Ki, float _Kd):
stepper(_STEP, _DIR, _EN),
encoder(_SER, _ID, _FLOW),
pcon(_Kp, _Ki, _Kd),
trajectory()
{
    pcon.init();
}

Actuator::~Actuator(){

}

void Actuator::operator=(float fd){
    this->stepper.ols(fd);
}

void Actuator::operator=(int fd){
    this->stepper.ols(fd);
}

float Actuator::at(){
    return this->encoder.position();
}

float Actuator::update(float setpoint){
    float u =this->pcon.update(setpoint, this->encoder.position());
    this->stepper.ols(u);
    return u;
}

void Actuator::setPconSat(float _l_lim, float _u_lim){
    this->pcon.setSat(_l_lim, _u_lim);
}

void Actuator::unsetPconSat(){
    this->pcon.unsetSat();
}