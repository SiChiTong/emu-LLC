#include "emulatorLib.h"

AMT21::AMT21(RawSerial &_SER, uint8_t _ID, PinName _FLOW):
SER(_SER),
FLOW(_FLOW),
state(){
    this->ID = _ID;
    this->SER.baud(115200);
}

AMT21::~AMT21(){

}

void AMT21::setID(uint8_t _ID){this->ID = _ID;}
uint8_t AMT21::getID(){return this->ID;}

void AMT21::setChecksum(bool isChk){this->check = isChk;}
bool AMT21::getChecksum(){return this->check;}

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
    if (check){
        uint8_t checksum_read = ((hb & 0x80)>>6) | ((hb & 0x40)>>6);
        uint8_t checksum_cal = this->checksum(cal_data);
        if(checksum_read == checksum_cal) return cal_data;
        else {
        // overwatch.flags_set(0xF1);
        return 0;
        }
    } else return cal_data;
}

double AMT21::readPosition(){ //rad/s
    return (this->read(this->check)*this->ratio*2*PI)/16383.0;
}

void AMT21::setKdt(float num){
    if (num >= 0.0002883f)
        this->kdt = num;
    else
        this->kdt = 0.0002883f;
}
float AMT21::getKdt(){return this->kdt;}

void AMT21::setSigmaW(float num){this->sigma_w = num;}
float AMT21::getSigmaW(){return this->sigma_w;}

void AMT21::setSigmaA(float num){this->sigma_a = num;} 
float AMT21::getSigmaA(){return this->sigma_a;}

void AMT21::kmfInit(){
    this->p11 = 1.0f;
    this->p12 = 0;
    this->p21 = 0;
    this->p22 = 1.0f;
    this->k_prev_pos = this->readPosition();
    this->x_hat_1 = this->readPosition();
    this->x_hat_2 = 0;
    this->k_wrap = 0;
    this->k_init = 1;
}

void AMT21::kmfEstimate(){
    if (this->k_init){
        float Q = pow(this->sigma_a, 2);
        float R = pow(this->sigma_w, 2);
        float x_hat_new_1, x_hat_new_2;
        //Checking
        double position = this->readPosition()+(2*PI*this->k_wrap);
        //Wrapping
        if ((position-this->k_prev_pos) > PI){
            this->k_wrap--;
            position = position-(2*PI*(this->k_wrap+1))+(2*PI*this->k_wrap);
        }else if ((position-this->k_prev_pos) < -PI){
                this->k_wrap++;
                position = position-(2*PI*(this->k_wrap-1))+(2*PI*this->k_wrap);
        }
        float velocity = (position-this->k_prev_pos)/this->kdt;

        //Filtering
        float kdt4 = pow(abs(this->kdt), 4);
        float kdt2 = pow(abs(this->kdt), 2); 
        x_hat_new_1 = this->x_hat_1+this->x_hat_2*this->kdt;
        x_hat_new_2 = 0+this->x_hat_2;
        float ye = velocity-x_hat_new_2;
        this->p11 = this->p11 + this->kdt*this->p21 + (Q*kdt4)/4 + (kdt2*(this->p12 + this->kdt*this->p22))/this->kdt;
        this->p12 = this->p12 + this->kdt*this->p22 + (Q*this->kdt*kdt2)/2.0f;
        this->p21 = (2*this->kdt*this->p21 + Q*kdt4 + 2*this->p22*kdt2)/(2*this->kdt);
        this->p22 = Q*kdt2 + this->p22;
        x_hat_new_1 = x_hat_new_1 + (this->p12*ye)/(R + this->p22);
        x_hat_new_2 = x_hat_new_2 + (this->p22*ye)/(R + this->p22);
        this->p11 = this->p11 - (this->p12*this->p21)/(R + this->p22);
        this->p12 = this->p12 - (this->p12*this->p22)/(R + this->p22);
        this->p21 = -this->p21*(this->p22/(R + this->p22) - 1.0f);
        this->p22 = -this->p22*(this->p22/(R + this->p22) - 1.0f);
        this->x_hat_1 = x_hat_new_1;
        this->x_hat_2 = x_hat_new_2;

        this->state.positionIsAt(position*this->ratio);
        this->state.velocityIsAt(this->x_hat_2*this->ratio);
        this->k_prev_pos = position;
    }
}

double AMT21::position(){
    if (k_init)
        return this->state.position();
    else
        return this->readPosition();
}

float AMT21::velocity(){
    if (k_init)
        return this->state.velocity();
    else
        return 0;
}

unsigned char AMT21::checksum(uint16_t data){
    unsigned char K0=0;
    unsigned char K1=0;
    uint16_t dataK1 = data >> 1;
    while(data){
        K0^=(data &1);
        K1^=(dataK1 &1);
        data>>=2;
        dataK1>>=2;
    }
    return (!K1<<1) | !K0;
}
