#include "mbed.h"
#include "platform/mbed_thread.h"
#include "emulatorPin.h"
#include "emulatorLib.h"
#include "eeprom.h"
#include "crc.h"
#include "FIFO.hpp"
#include <Eigen/Dense.h>
#include <cmath>

#define PI 3.14159265359

uint64_t t = 0;
AnalogIn temp_adc(ADC_TEMP);
EEPROM memory(I2C4_SDA, I2C4_SCL, 0, 16384);
BusOut onBoardLed(LD1, LD2, LD3, LD4);

RawSerial uart4(UART4_TX, UART4_RX, 1000000);
RawSerial enc1(UART5_TX, UART5_RX, 115200);
RawSerial enc2(UART2_TX, UART2_RX, 115200);
Emuart emuart4(uart4, 1024);
int dataGood;

DigitalOut output_enable_1(OE1);
DigitalOut output_enable_2(OE2);
DigitalOut output_enable_3(OE3);
// Actuator q2(Y1, Y7, Y5, enc2, 0x3C, FLOW_CH2);
// // Actuator q4(Y12, Y10, Y4, enc2, 0x3C, FLOW_CH2);
// // Actuator q5(Y15, Y11, Y19, enc2, 0x3C, FLOW_CH2);//Y13
// Actuator q6(Y14, Y18, Y21, enc2, 0x3C, FLOW_CH2);
// //can't use Y12,Y13

Thread idleThread(osPriorityLow);
Thread errorHandle(osPriorityHigh);
Thread rxMessageParser(osPriorityNormal);
Ticker enct, timer;

signed int raw_position = 0;
float position;
float velocity = 0;
float raw_velocity = 0;

float prev_pos = 0.0;

struct message{
    uint8_t     startByte[2];
    uint8_t     command;
    uint8_t     dataLen;//Excluding start,dataLen, and command
    uint8_t     data[255];
    uint32_t    crc32;
};

struct message uart4Rx;

void idleRoutine(){
    for (;;){
        onBoardLed = (onBoardLed | 0x1) & 0xD;
        // pilot = 1;
        ThisThread::sleep_for(500);
        onBoardLed = (onBoardLed | 0x2) & 0xE;
        // pilot = 0;
        ThisThread::sleep_for(500);
    }
}

void emuart4Parser(){
    for(;;){
        dataGood = emuart4.parse();
        if (dataGood == 1){
            switch (emuart4.command){
                case 147:
                    onBoardLed[2] = emuart4.data[0];
                    break;
                case 148:
                    onBoardLed[3] = emuart4.data[0];
                    break;
            }
        } else if (dataGood == -1){
            uart4.printf("\nERROR!\n");
            // q6 = m2*63;
            // q2 = m3*220;
        }
    }
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
    // q6.encoder.kmfEstimate();
}

int main(){
    initializeMsg();

    idleThread.start(idleRoutine);
    emuart4.init();
    rxMessageParser.start(emuart4Parser);

    //small 1000-2000 8mstp Hz,large 3500 Hz 16mstp
    // q6.stepper.enable();
    // q6.stepper.setMaxFrequency(2000.0f*8.0f);
    // q2.stepper.setMaxFrequency(4000.0f*16.0f);
    // q6.encoder.kmfInit();
    // enct.attach(&kal_test, q6.encoder.getKdt());

    output_enable_1 = 1;
    output_enable_2 = 1;
    output_enable_3 = 1;

    while(1){
        // q6 = 1000*8;
        // position = 4000.0f*16.0f*sin(0.5*PI*float(t)*0.001);
        // q2 = position;
        
        // uart4.printf("%.2f,%.4f\n",q6.encoder.position(), q6.encoder.velocity());
        uart4.printf("%d\t%d\t%d\t%d\t%d\t%d\n",dataGood, emuart4.command,emuart4.data[0],emuart4.data[1],emuart4.data[2], emuart4.data[emuart4.dataLen-1]);
        ThisThread::sleep_for(200);
    }
}