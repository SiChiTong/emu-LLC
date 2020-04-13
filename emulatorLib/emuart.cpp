#include "emulatorLib.h"

Emuart::Emuart(RawSerial &_SER, unsigned int bufferSize):
SER(_SER),
fifo(bufferSize){
    this->bufferSize = bufferSize;
    this->setSamplingTime(0.003); 
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

void Emuart::write(uint8_t _command, uint8_t _dataLen, uint8_t* _dataBuffer){
    uint8_t dataForCrc[_dataLen+1];
    this->SER.putc(0x7E);
    this->SER.putc(0x77);
    this->SER.putc(_command);
    this->SER.putc(_dataLen);
    for (int i = 0; i <= _dataLen-1; i++){
        dataForCrc[i] = _dataBuffer[i];
        this->SER.putc(_dataBuffer[i]);
    }
    dataForCrc[_dataLen] = _command;
    uint32_t crc32Send = crc32((char*)dataForCrc,_dataLen+1);
    this->SER.putc(crc32Send>>24);
    this->SER.putc((crc32Send>>16)&0x000000FF);
    this->SER.putc((crc32Send>>8)&0x000000FF);
    this->SER.putc((crc32Send)&0x000000FF);
}

void Emuart::write(uint8_t _command){
    this->SER.putc(0x7E);
    this->SER.putc(0x77);
    this->SER.putc(_command);
    this->SER.putc(0);
    uint8_t dataForCrc[1] = {_command};
    uint32_t crc32Send = crc32((char*)dataForCrc,1);
    this->SER.putc(crc32Send>>24);
    this->SER.putc((crc32Send>>16)&0x000000FF);
    this->SER.putc((crc32Send>>8)&0x000000FF);
    this->SER.putc((crc32Send)&0x000000FF);
}

void Emuart::print(char* text){
    uint8_t len = strlen(text);
    this->write(88, len, (uint8_t*)text);
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

void Emuart::clearAll(){
    this->command = 0;
    this->dataLen = 0;
    this->clearData();
}

void Emuart::clearData(){
    memset(this->data, 0, 256);
}