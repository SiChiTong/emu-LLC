#include "mbed.h"
#include "platform/mbed_thread.h"
#include "emulatorPin.h"
#include "emulatorLib.h"
#include "eeprom.h"
#include "crc.h"
#include <Eigen/Dense.h>
#include <cmath>

#define PI 3.14159265359

using namespace Eigen;

uint64_t t = 0;
AnalogIn temp_adc(ADC_TEMP);
EEPROM memory(I2C4_SDA, I2C4_SCL, 0, 16384);
RawSerial uart4(UART4_TX, UART4_RX, 1000000);
BusOut onBoardLed(LD1, LD2, LD3, LD4);

RawSerial enc1(UART5_TX, UART5_RX, 115200);
RawSerial enc2(UART2_TX, UART2_RX, 115200);

DigitalOut output_enable_1(OE1);
DigitalOut output_enable_2(OE2);
DigitalOut output_enable_3(OE3);
Actuator q2(Y1, Y7, Y5, enc2, 0x3C, FLOW_CH2);
// Actuator q4(Y12, Y10, Y4, enc2, 0x3C, FLOW_CH2);
// Actuator q5(Y15, Y11, Y19, enc2, 0x3C, FLOW_CH2);//Y13
Actuator q6(Y14, Y18, Y21, enc2, 0x3C, FLOW_CH2);
//can't use Y12,Y13

DigitalOut pilot(OD2);

Thread idleThread(osPriorityLow);
Thread errorHandle(osPriorityHigh);
Ticker enct, timer;

signed int raw_position = 0;
float position;
float velocity = 0;
float raw_velocity = 0;

float prev_pos = 0.0;

struct message{
    uint8_t     startByte = 0;
    uint8_t     command = 0;
    uint8_t     dataLen = 0;//Excluding start,dataLen, and command
    uint8_t     data[255];
    uint32_t    crc32;
};

struct message uart4Rx;

void idleRoutine(){
    for (;;){
        onBoardLed = (onBoardLed | 1) & 1;
        pilot = 1;
        ThisThread::sleep_for(500);
        onBoardLed = (onBoardLed | 2) & 2;
        pilot = 0;
        ThisThread::sleep_for(500);
    }
}

uint8_t a;
void uart4Callback(){
    if (uart4.readable()){
        uint8_t crc[4];
        uart4Rx.startByte = uart4.getc();
        a = uart4Rx.startByte;
        if (uart4Rx.startByte == 0x7E){
            uart4Rx.command = uart4.getc();
            uart4Rx.dataLen = uart4.getc();
            for (uint8_t i = 0; i <= uart4Rx.dataLen+3; i++){
                if (i >= uart4Rx.dataLen)
                    crc[i-uart4Rx.dataLen] = uart4.getc();
                else
                    uart4Rx.data[i] = uart4.getc();
            }
            uart4Rx.crc32 = (crc[0]<<24)|(crc[1]<<16)|(crc[2]<<8)|crc[3];
            uint32_t expectedCrc32 = crc32((char*)uart4Rx.data,uart4Rx.dataLen);
        }
    }
    uart4Rx.startByte = 0;
    // m2 = uart4.getc();
    // ThisThread::sleep_for(1);
    // m3 = uart4.getc();
    // q6 = m2*63;
    // q2 = m3*220;
}

void millis(){t += 1;}

void initializeMsg(){
    timer.attach(&millis, 0.001f);
    output_enable_1 = 0;
    output_enable_2 = 0;
    output_enable_3 = 0;
    ThisThread::sleep_for(10);
    uart4.printf("InternalTemp: %.2f C\n", temp_adc.read()*100);
    uart4.printf("SystemCoreClock: %ld Hz\n", SystemCoreClock);
}

void kal_test(){
    q6.encoder.kmfEstimate();
}

int main(){
    initializeMsg();

    idleThread.start(idleRoutine);
    

    uart4.attach(&uart4Callback);

    //small 1000-2000 8mstp Hz,large 3500 Hz 16mstp
    q6.stepper.enable();
    q6.stepper.setMaxFrequency(2000.0f*8.0f);
    q2.stepper.setMaxFrequency(4000.0f*16.0f);
    q6.encoder.kmfInit();
    enct.attach(&kal_test, q6.encoder.getKdt());

    output_enable_1 = 1;
    output_enable_2 = 1;
    output_enable_3 = 1;
    
    while(1){
        // q6 = 1000*8;
        // position = 4000.0f*16.0f*sin(0.5*PI*float(t)*0.001);
        // q2 = position;
        
        // uart4.printf("%.2f,%.4f\n",q6.encoder.position(), q6.encoder.velocity());
        // uart4.printf("%X\t%X\t%X\t%X\t0x%X\n", crc[0],crc[1],crc[2],crc[3], uart4Rx.crc32);
        uart4.printf("%d\t%d\t%d\t%d\t%d\t", a, uart4Rx.dataLen, uart4Rx.command,uart4Rx.data[0], uart4Rx.data[uart4Rx.dataLen-1]);
        uart4.printf("0x%X\n", uart4Rx.crc32);
        ThisThread::sleep_for(200);
    }
}
