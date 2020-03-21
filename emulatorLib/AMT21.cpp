#include "emulatorLib.h"

#define PI 3.14159265359

AMT21::AMT21(RawSerial &_SER, uint8_t _ID, PinName _FLOW):
SER(_SER),
FLOW(_FLOW)
{
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

void AMT21::setCov1(float num){this->sigma_a = num;}; 
float AMT21::getCov1(){return this->cov1;}

void AMT21::setCov2(float num){this->sigma_a = num;}  
float AMT21::getCov2(){return this->cov2;}

void AMT21::kmfInit(){
    this->Pp << this->cov1,0, 0,this->cov2;
    // i2 << 1,0, 0,1;
    this->x_hat << 0,0;
    this->_x_hat << 0,0;
    this->K << 0,0;
    this->F << 1,0, this->kdt,1;
    this->G << pow(this->kdt,2)/2,this->kdt;
    this->H << 0,1;    
    this->Q = pow(this->sigma_a, 2);
    this->R = pow(this->sigma_w, 2);
    this->k_prev_pos = 0.0f;
    this->k_init = 1;
}

float AMT21::kmfUpdate(){
    float raw_position = this->read();
    float position = (raw_position/16383.0f)*2*PI;
    float velocity = (position-this->k_prev_pos)/this->kdt;
    this->_x_hat = this->F*this->x_hat;
    float zp = this->H*this->x_hat;
    float ye = velocity - zp;
    this->Pp = this->F*this->Pp*this->F.transpose()+this->G*this->G.transpose()*this->Q;
    this->K = this->Pp*this->H.transpose()*(1/(this->H*this->Pp*this->H.transpose()+this->R));
    this->_x_hat += this->K*(ye);
    this->Pp = (Matrix2d::Identity()-this->K*this->H)*this->Pp;
    this->x_hat = this->_x_hat;
    this->k_prev_pos = position;
    return this->x_hat(1,0);
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
