#include "as5047.h"

AS5047::AS5047(SPI &_spi_bus, PinName _ssel_pin):
ssel(_ssel_pin),
spi(_spi_bus){
	this->ssel = 1;
	this->spi.format(8,1);
    this->spi.frequency(4000000);
}

AS5047::~AS5047(){

}

float AS5047::getPositionDegree(void){
	this->ssel = 0;
	uint8_t angle_h = this->spi.write(0xFF);
	uint8_t angle_l = this->spi.write(0xFF);
	this->ssel = 1;       
	return (((angle_h<<8) | angle_l) & 0x3FFF)*0.02197399743;
}

uint16_t AS5047::getPosition(void){
	this->ssel = 0;
	uint8_t angle_h = this->spi.write(0xFF);
	uint8_t angle_l = this->spi.write(0xFF);
	this->ssel = 1;       
	return (((angle_h<<8) | angle_l) & 0x3FFF);
}

long AS5047::getPositionCon(void){
	return this->conpo;
}

float AS5047::getPositionRadian(void){
	this->ssel = 0;
	uint8_t angle_h = this->spi.write(0xFF);
	uint8_t angle_l = this->spi.write(0xFF);
	this->ssel = 1;       
	return (((angle_h<<8) | angle_l) & 0x3FFF)*0.0003835186;
}

void AS5047::zero(uint16_t zero_point){
	uint16_t lsb = zero_point & 0x3F;
	uint16_t msb = (zero_point >> 6)&0xFF;
	/**6LSB**/
	this->ssel = 0;
	this->spi.write(0x0017);
	this->ssel = 1;
	wait_us(1);  
	this->ssel = 0;
	if (this->calEvenParity(lsb) == 1)this->spi.write(lsb | 0x8000);
	else this->spi.write(lsb & 0x7FFF);
	this->ssel = 1;
	wait_us(10);
	/**8MSB**/
	this->ssel = 0;
	this->spi.write(0x8016); 
	this->ssel = 1;
	wait_us(1);  
	this->ssel = 0;
	if (this->calEvenParity(msb) == 1)this->spi.write(msb | 0x8000);
	else this->spi.write(msb & 0x7FFF);
	this->ssel = 1;
}

unsigned char AS5047::calEvenParity(unsigned char data){
    unsigned char parity=0;

    while(data){
        parity^=(data &1);
        data>>=1;
    }
    return parity;
}

void AS5047::continuousPositionLoop(void){
	long prev_position = getPosition();
	wait(0.1);
    for (;;){
        this->conpo = getPosition()+(16383*this->wrap);
        //Wrapping
        if ((this->conpo-prev_position) > 8192){
            this->wrap--;
            this->conpo = this->conpo-(16383*(this->wrap+1))+(16383*this->wrap);
        }else if ((this->conpo-prev_position) < -8192){
            this->wrap++;
            this->conpo = this->conpo-(16383*(this->wrap-1))+(16383*this->wrap);
        }
        prev_position = this->conpo;
        ThisThread::sleep_for(1);
    }
}

void AS5047::resetCon(){
	this->wrap = 0;
}