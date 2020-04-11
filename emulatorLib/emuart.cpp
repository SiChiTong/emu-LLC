#include "emulatorLib.h"

Emuart::Emuart(RawSerial &_SER, unsigned int bufferSize):
SER(_SER),
fifo(bufferSize){
    this->bufferSize = bufferSize;
    this->setSamplingTime(0.005); 
}

Emuart::Emuart(RawSerial &_SER, unsigned int bufferSize, float samplingTime):
SER(_SER),
fifo(bufferSize){
    this->bufferSize = bufferSize; 
    this->setSamplingTime(samplingTime);
}

Emuart::~Emuart(){

}

void Emuart::init(){
    this->SER.attach(callback(this, &Emuart::rxCallback), Serial::RxIrq);
}

void Emuart::rxCallback(void){
    while (this->SER.readable()){
        this->fifo.put(this->SER.getc());
    }
}

int Emuart::parse(){
    if (this->fifo.available() > 0){
        if (this->fifo.get() == 0x7E){
            if (this->fifo.get() == 0x77){
                uint8_t crc[4];
                this->command = this->fifo.get(); //get command
                this->dataLen = this->fifo.get(); //get data length
                for (uint8_t i = 0; i <= this->dataLen+3; i++){ 
                    if (i >= this->dataLen) crc[i-this->dataLen] = this->fifo.get();
                    else this->data[i] = this->fifo.get();
                }
                this->checksum = (crc[0]<<24)|(crc[1]<<16)|(crc[2]<<8)|crc[3]; //combine crc32
                data[dataLen] = this->command;
                uint32_t expectedCrc32 = crc32((char*)this->data,this->dataLen+1); //calculate crc32
                if (expectedCrc32 != this->checksum) return -1; //if the data is wrong
                else return 1; //the data is right
            } else this->fifo.get(); //clear the buffer until it pass if statement
            return 0; //it's not start byte 2
        }else this->fifo.get(); //clear the buffer until it pass if statement
        return 0; //it's not start byte 1
    } else {
        ThisThread::sleep_for(this->samplingTime);
        return 0; //no data aviable
    }
}

void Emuart::setSamplingTime(float Ts){
    this->samplingTime = int(Ts*1000); //convert from s to ms
}

float Emuart::getSamplingTime(){
    return float(this->samplingTime*0.001f); //convert from ms to s
}

void Emuart::setBufferSize(unsigned int size){
    this->bufferSize = size; //convert from s to ms
}

unsigned int Emuart::getBufferSize(){
    return this->bufferSize; //convert from ms to s
}

void Emuart::clearInputBuffer(){
    this->fifo.clear(); //convert from s to ms
}

void Emuart::clearData(){
    memset(this->data, 0, 256);
}
