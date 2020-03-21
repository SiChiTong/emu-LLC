#include "emulatorLib.h"
#include "mbed.h"
#include <cmath>
#include <cstdint>

Stepper::Stepper(PinName _STEP, PinName _DIR, PinName _EN):
STEP(_STEP), 
DIR(_DIR),
EN(_EN){
    this->enable();
    this->setMaxFrequency(1000.0f);
    this->setFrequency(0.0f);
    this->DIR = 0;
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

    if (this->frequency > 0.0f){
        this->STEP.write(0.50f);
        this->DIR = 0;
        this->STEP.period(1/f);
    } else if (this->frequency < 0.0f){
        this->STEP.write(0.50f);
        this->DIR = 1; 
        this->STEP.period(1/abs(f));
    }
}