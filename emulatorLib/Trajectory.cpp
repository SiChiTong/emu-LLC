#include "emulatorLib.h"

Trajectory::Trajectory(){
    this->atGoal = 1;
}

Trajectory::~Trajectory(){

}

void Trajectory::setViaPoints(std::deque <float> _qr, std::deque <float> _vr, std::deque <float> _Tvec){
    this->qr.resize(_qr.size());
    this->vr.resize(_vr.size());
    this->Tvec.resize(_Tvec.size());
    this->qr = _qr;
    this->vr = _vr;
    this->Tvec = _Tvec;
    this->atGoal = 0;

    this->qi = this->qr.front();
    this->qr.pop_front(); 
    this->qf = this->qr.front();
    this->vi = this->vr.front();
    this->vr.pop_front(); 
    this->vf = this->vr.front();
    this->Tlat = 0;
    this->T = this->Tvec.front();
    this->Tvec.pop_front();
    this->c0 = this->qi;
    this->c1 = this->vi;
    this->c2 = 3*(this->qf-this->qi)/pow(this->T,2)-(this->vf+2*this->vi)/(float)this->T;
    this->c3 = -2*(this->qf-this->qi)/pow(this->T,3)+(this->vf+this->vi)/pow(this->T,2);
}

void Trajectory::setGoal(float _qi, float _qf, float _vi, float _vf, float _T){
    this->qr.resize(2);
    this->vr.resize(2);
    this->Tvec.resize(1);
    this->qr = {_qi, _qf};
    this->vr = {_vi, _vf};
    this->Tvec = {_T};
    this->atGoal = 0;

    this->qi = this->qr.front();
    this->qr.pop_front(); 
    this->qf = this->qr.front();
    this->vi = this->vr.front();
    this->vr.pop_front(); 
    this->vf = this->vr.front();
    this->Tlat = 0;
    this->T = this->Tvec.front();
    this->Tvec.pop_front();
    this->c0 = this->qi;
    this->c1 = this->vi;
    this->c2 = 3*(this->qf-this->qi)/pow(this->T,2)-(this->vf+2*this->vi)/(float)this->T;
    this->c3 = -2*(this->qf-this->qi)/pow(this->T,3)+(this->vf+this->vi)/pow(this->T,2);
}

void Trajectory::nextSub(){
    this->qi = this->qr.front();
    this->qr.pop_front(); 
    this->qf = this->qr.front();
    this->vi = this->vr.front();
    this->vr.pop_front(); 
    this->vf = this->vr.front();
    this->Tlat += this->T;
    this->T = this->Tvec.front();
    this->Tvec.pop_front();
    this->c0 = this->qi;
    this->c1 = this->vi;
    this->c2 = 3*(this->qf-this->qi)/pow(this->T,2)-(this->vf+2*this->vi)/(float)this->T;
    this->c3 = -2*(this->qf-this->qi)/pow(this->T,3)+(this->vf+this->vi)/pow(this->T,2);
}

float Trajectory::getPosTraj(float t){
    float traj;
    if (t > this->T+this->Tlat){
        traj = this->qf;
        if (!Tvec.empty()){
            this->nextSub();
        }else{
            this->atGoal = 1;
        }
    }
    if (t <= this->T+this->Tlat){
        this->atGoal = 0;
        t = t-this->Tlat;
        traj = this->c0+this->c1*t+this->c2*t*t+this->c3*t*t*t;
    }
    return traj;
}

float Trajectory::getVelTraj(float t){
    float traj;
    if (t > this->T+this->Tlat){
        traj = this->vf;
        // if (!Tvec.empty()){
        //     this->nextSub();
        //     this->atGoal = 1;
        // }
    }
    if (t <= this->T+this->Tlat){
        t = t-this->Tlat;
        traj = this->c1+2*this->c2*t+3*this->c3*t*t;
    }
    return traj;
}

float Trajectory::step(float position){
    this->qf = position;
    return position;
}

float Trajectory::getTime(){
    float sum_time = 0;
    for (int i = 0; i<this->Tvec.size(); i++){
        sum_time += this->Tvec.at(i);
    }
    return sum_time;
}

uint8_t Trajectory::pointLeft(){
    return this->Tvec.size();
}

bool Trajectory::reached(){ return this->atGoal; }
float Trajectory::getC0(){ return this->c0; }
float Trajectory::getC1(){ return this->c1; }
float Trajectory::getC2(){ return this->c2; }
float Trajectory::getC3(){ return this->c3; }
float Trajectory::getQi(){ return this->qi; }
float Trajectory::getQf(){ return this->qf; }
float Trajectory::getVi(){ return this->vi; }
float Trajectory::getVf(){ return this->vf; }
float Trajectory::getDuration(){ return this->T; }

 