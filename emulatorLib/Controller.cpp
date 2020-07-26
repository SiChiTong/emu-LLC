#include "emulatorLib.h"

Controller::Controller(float _Kp, float _Ki, float _Kd){
    this->setKp(_Kp);
    this->setKi(_Ki);
    this->setKd(_Kd);
    this->isInit = 0;
}

Controller::~Controller(){};

void Controller::setKp(float _Kp){this->Kp = _Kp;}
void Controller::setKi(float _Ki){this->Ki = _Ki;}
void Controller::setKd(float _Kd){this->Kd = _Kd;}
float Controller::getKp(void){return this->Kp;}
float Controller::getKi(void){return this->Ki;}
float Controller::getKd(void){return this->Kd;}

void Controller::init(){
    this->s = 0;
    this->p = 0;
    this->isInit = 1;
}

float Controller::updatePID(float setpoint, float feedback){
    float u;
    if (this->isInit){
        float e = setpoint-feedback;
        this->s += e;
        float i_term = this->Ki*this->s;
        if (this->sat)
            u = this->saturate(this->Kp*e + i_term + this->Kd*(e-this->p));
        else
            u = this->Kp*e + i_term + this->Kd*(e-this->p);
        this->p = e;
    } else {
        u = 0;
    }
    return u;
}

float Controller::updatePIDF(float setpoint, float vff, float feedback){
    float u;
    if (this->isInit){
        float e = setpoint-feedback;
        this->s += e;
        float i_term = this->Ki*this->s;
        if (this->sat)
            u = this->saturate(this->Kp*e + i_term + this->Kd*(e-this->p) + vff);
        else
            u = this->Kp*e + i_term + this->Kd*(e-this->p) + vff;
        this->p = e;
    } else {
        u = 0;
    }
    return u;
}

void Controller::setSat(float _l_lim, float _u_lim){
    this->u_lim = _u_lim;
    this->l_lim = _l_lim;
    this->sat = 1;
}

void Controller::unsetSat(){
    this->u_lim = 0;
    this->l_lim = 0;
    this->sat = 0;
}

float Controller::saturate(float input){
    float output;
    if (input<=this->l_lim)
        output = this->l_lim;
    else if (input>=this->u_lim)
        output = this->u_lim;
    else
        output = input;
    return output;
}
