#include "mbed.h"
#include "platform/mbed_thread.h"
#include "emulatorPin.h"
#include "emulatorLib.h"
#include "eeprom.h"
#include <Eigen/Dense.h>
#include <cmath>

#define PI 3.14159265359

using namespace Eigen;


AnalogIn temp_adc(ADC_TEMP);
EEPROM memory(I2C4_SDA, I2C4_SCL, 0, 16384);
RawSerial uart4(UART4_TX, UART4_RX, 1000000);
BusOut onBoardLed(LD1, LD2, LD3, LD4);

RawSerial enc1(UART5_TX, UART5_RX, 115200);
RawSerial enc2(UART2_TX, UART2_RX, 115200);

DigitalOut output_enable_1(OE1);
DigitalOut output_enable_2(OE2);
DigitalOut output_enable_3(OE3);
// Actuator q2(Y1, Y7, Y5, enc2, 0x3C, FLOW_CH2);
// Actuator q4(Y12, Y10, Y4, enc2, 0x3C, FLOW_CH2);
// Actuator q5(Y15, Y11, Y19, enc2, 0x3C, FLOW_CH2);//Y13
Actuator q6(Y14, Y18, Y21, enc2, 0x3C, FLOW_CH2);

//cna't use Y12,Y13

DigitalOut pilot(OD2);

Thread idleThread(osPriorityLow);
Ticker enct;

uint16_t raw_position = 0;
float position;
float velocity = 0;
float raw_velocity = 0;
int dt = 5;
unsigned long t = 0;
MatrixXd pp(2,2);
MatrixXd i2(2,2);
VectorXd xhat(2);
VectorXd xhat_new(2);
VectorXd K(2);
MatrixXd F(2,2);
VectorXd G(2);
RowVectorXd H(2);

float prev_pos = 0.0;

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

void uart4Callback(){
    uart4.putc(uart4.getc());
}

void initializeMsg(){
    output_enable_1 = 0;
    output_enable_2 = 0;
    output_enable_3 = 0;
    ThisThread::sleep_for(10);
    uart4.printf("InternalTemp: %.2f C\n", temp_adc.read()*100);
    uart4.printf("SystemCoreClock: %ld Hz\n", SystemCoreClock);
}

void kal_test(){
    velocity = q6.encoder.kmfUpdate();
}

int main(){
    initializeMsg();

    idleThread.start(idleRoutine);
    

    uart4.attach(&uart4Callback);

    //small 1000-1100 Hz,large 2000 Hz
    q6.stepper.enable();
    q6.stepper.setFrequency(1200.0f);
    // q6.stepper.unHold();
    q6.encoder.kmfInit();
    enct.attach(&kal_test, q6.encoder.getKdt());

    output_enable_1 = 1;
    output_enable_2 = 1;
    output_enable_3 = 1;
    while(1){
        position = 1000.0f*sin(1*PI*float(t)*0.001);
        q6 = position;
        raw_position = q6.at();
        position = (raw_position/16383.0f)*2*PI;
        raw_velocity = (position-prev_pos)/(float(dt)*0.001f);
        uart4.printf("%.2f,%.4f,%.4f,\n",position, raw_velocity, velocity);
        // uart4.printf("%d\n", raw_position);
        prev_pos = position;
        t += dt; 
        ThisThread::sleep_for(dt);
    }
}
