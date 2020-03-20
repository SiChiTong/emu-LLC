#include "emulatorLib.h"
#include "mbed.h"
#include <cmath>
#include <cstdint>

Stepper::Stepper(PinName _STEP, PinName _DIR, PinName _EN):
STEP(_STEP), 
DIR(_DIR),
EN(_EN){
    this->enable();
    // this->hold();
    this->DIR = 0;
    this->STEP.write(0.50f);
    this->setFrequency(100.0f);
}

Stepper::~Stepper(){

}

void Stepper::enable(void){
    this->EN = 0;
}

void Stepper::disable(void){
    this->EN = 1;
}

void Stepper::hold(void){
    this->STEP.suspend();
}

void Stepper::unHold(void){
    this->STEP.resume();
}

void Stepper::setFrequency(float _frequency) {
    this->frequency = _frequency;
    if (this->frequency == 0.0f){
        this->DIR = 0;
    }
    else if (this->frequency > 0.0f){
        this->DIR = 0;
        this->STEP.period(1/this->frequency);
    } else if (this->frequency < 0.0f){
        this->DIR = 1; 
        this->STEP.period(1/abs(this->frequency));
    }
}