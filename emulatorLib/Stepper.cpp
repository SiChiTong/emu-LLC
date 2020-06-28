#include "emulatorLib.h"

Stepper::Stepper(PinName _STEP, PinName _DIR, PinName _EN):
STEP(_STEP), 
DIR(_DIR),
EN(_EN){
    this->enable();
    this->setMaxFrequency(10000000.0f);
    this->setFrequency(0.0f);
    this->DIR = 1;
}

Stepper::~Stepper(){

}

void Stepper::setMaxFrequency(float _frequency){
    this->max_frequency = _frequency;
}

void Stepper::enable(void){
    this->EN = 0;
}

void Stepper::disable(void){
    this->EN = 1;
}

void Stepper::setFrequency(float _frequency) {
    this->frequency = _frequency;
    float f;
    if (abs(this->frequency) <= 0.1f)
        f = 0.1f;
    else if (abs(this->frequency) >= this->max_frequency)
        f = this->max_frequency;
    else
        f = _frequency;

    if (this->frequency > 0.001f){
        this->STEP.write(0.50f);
        this->DIR = 1;
        this->STEP.period(1/f);
    } else if (this->frequency < 0.001f){
        this->STEP.write(0.50f);
        this->DIR = 0; 
        this->STEP.period(1/abs(f));
    }else{
        this->STEP.write(0.0f);
    }
}

void Stepper::setFrequency(int _frequency){
    this->setFrequency(float(_frequency));
}

void Stepper::setRatio(float _ratio){
    this->ratio = _ratio;
}

void Stepper::setMicro(uint8_t _microstep){
    this->microstep = _microstep;
}

void Stepper::ols(float speed){
    this->setFrequency((float)(speed*this->microstep*this->ratio*this->spr*1/(2.0f*PI)));
}