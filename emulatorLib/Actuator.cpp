#include "emulatorLib.h"

Actuator::Actuator(PinName _STEP, PinName _DIR, PinName _EN, RawSerial &_SER, uint8_t _ID, PinName _FLOW):
stepper(_STEP, _DIR, _EN),
encoder(_SER, _ID, _FLOW){

}

Actuator::~Actuator(){

}

void Actuator::operator=(float fd){
    this->stepper.setFrequency(fd);
}

uint16_t Actuator::at(){
    return this->encoder.position();
}