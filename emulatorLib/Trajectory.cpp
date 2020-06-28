#include "emulatorLib.h"

Trajectory::Trajectory(){
    this->atGoal = 1;
}

Trajectory::~Trajectory(){

}

void Trajectory::setGoal(float _qi, float _qf, float _vi, float _vf, float _T){
    this->qi = _qi;
    this->qf = _qf;
    this->vi = _vi;
    this->vf = _vf;
    this->T = _T;
    this->c0 = _qi;
    this->c1 = _vi;
    this->c2 = 3*(_qf-_qi)/pow(_T,2)-(_vf+2*_vi)/(float)_T;
    this->c3 = -2*(_qf-_qi)/pow(_T,3)+(_vf+_vi)/pow(_T,2);
    this->atGoal = 0;
}

float Trajectory::update(float t){
    float traj;
    if (t <= this->T){
        this->atGoal = 0;
        traj = this->c0+this->c1*t+this->c2*t*t+this->c3*t*t*t;
    }else{
        traj = this->qf;
        this->atGoal = 1;
    }
    return traj;
}

float Trajectory::step(float position){
    float traj = position;
    return traj;
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

