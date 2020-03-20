#include "emulatorLib.h"
#include "mbed.h"
#include <math.h>
#include <cstdint>

AMT21::AMT21(RawSerial &_SER, uint8_t _ID, PinName _FLOW):
SER(_SER),
FLOW(_FLOW){
    this->ID = _ID;
    this->SER.baud(115200);
}

AMT21::~AMT21(){

}

void AMT21::setID(uint8_t _ID){
    this->ID = _ID;
}

uint8_t AMT21::getID(){
    return this->ID;
}

uint16_t AMT21::read(){
    this->FLOW = 1;
    this->SER.putc(this->ID);
    wait_us(75);
    this->FLOW = 0;
    // overwatch.flags_set(0x01);
    uint8_t lb = this->SER.getc();
    uint8_t hb = this->SER.getc();
    uint16_t cal_data = lb|((hb&0x3F)<<8);
    return cal_data;
}

uint16_t AMT21::read(uint8_t check){
    this->FLOW = 1;
    this->SER.putc(this->ID);
    wait_us(75);
    this->FLOW = 0;
    // overwatch.flags_set(0x01);
    uint8_t lb = this->SER.getc();
    uint8_t hb = this->SER.getc();
    uint16_t cal_data = lb|((hb&0x3F)<<8);
    if (check == 1){
        uint8_t K0 = (hb & 0x40)>>6;
        uint8_t K1 = (hb & 0x80)>>7;
        uint8_t K0_cal = this->calK0(cal_data);
        uint8_t K1_cal = this->calK1(cal_data);
        if((K0 == K0_cal) && (K1 == K1_cal)) return cal_data;
        else {
        // overwatch.flags_set(0xF1);
        return 0;
        }
    } else return cal_data;
}

unsigned char AMT21::calK0(uint16_t data){
    unsigned char parity=0;
    while(data){
        parity^=(data &1);
        data>>=2;
    }
    return !parity;
}

unsigned char AMT21::calK1(uint16_t data){
    unsigned char parity=0;
    data >>= 1;
    while(data){
        parity^=(data &1);
        data>>=2;
    }
    return !parity;
}
