#include "mbed.h"
#include "platform/mbed_thread.h"
#include "emulatorPin.h"
#include "emulatorLib.h"
#include "emutil.h"
#include "emuPeripheral.h"
#include <cmath>

#define CONTROLLER_SAMPLING_T   0.0084f
#define PI                      3.14159265359
#define EMUART_BUFFER_SIZE      1024

Emuart emuart4(_UART4, EMUART_BUFFER_SIZE, 0.1);
Actuator q1(Y15, Y10, Y9, _RS485_2, 0x54, FLOW_CH2);
Actuator q2(Y1, Y7, Y5, _RS485_2, 0x3C, FLOW_CH2);
Actuator q3(Y17, Y11, Y23, _RS485_2, 0x3C, FLOW_CH2);
Actuator q4(Y3, Y0, Y2, _RS485_2, 0x3C, FLOW_CH2, 4.0f, 0.0001f, 0.0f);
Actuator q5(Y16, Y6, Y4, _RS485_2, 0x3C, FLOW_CH2, 4.0f, 0.0001f, 0.0f);
Actuator q6(Y14, Y18, Y21, _RS485_2, 0x3C, FLOW_CH2, 4.0f, 0.0001f, 0.0f);//q6
float q6_traj;
// //can't use Y8,Y12,Y13

Thread idleThread(osPriorityNormal);
Thread errorHandle(osPriorityHigh);
Thread rxMessageParser(osPriorityNormal2);
Ticker enct;

void idleRoutine(){
    for (;;){
        ONBOARD_LEDS = (ONBOARD_LEDS | 0x1) & 0xD;
        ThisThread::sleep_for(500);
        ONBOARD_LEDS = (ONBOARD_LEDS | 0x2) & 0xE;
        ThisThread::sleep_for(500);
    }
}


// q6 = m2*63;
// q6 = m3*220;

void initializeMsg(){
    SPLASH();
    OUTPUT_DISABLE();
    _UART4.printf("EMULATOR BOARD v2.6 21-6-2020 C.Thanapong\r\n");
    _UART4.printf("SystemCore Clock:  %ld Hz\r\n", GET_SYSCLK());
    _UART4.printf("InternalTemp:  %.2f C\r\n", GET_CORETEMP());
    _UART4.printf("+++++++++++++++++++++++++++++++\r\n");
}

void emuart4Parser();

void startThread(){
    emuart4.init();
    idleThread.start(idleRoutine);
    rxMessageParser.start(emuart4Parser);
}

void actuatorSetup(){
    q1.stepper.setRatio(-30.0);
    q1.stepper.setMicro(32);
    q1.encoder.setKdt(0.0084);
    q1.encoder.kmfInit();
    q1.stepper.enable();

    q2.stepper.setRatio(20.0);
    q2.stepper.setMicro(32);
    q2.stepper.disable();

    q6.setPconSat(-4.5, 4.5);
    q6.stepper.setRatio(5.18);
    q6.stepper.setMicro(32);
    q6.encoder.setKdt(0.0084);
    // q6.encoder.kmfInit();
    q6.stepper.disable();
}


float u;
Timer tau;
bool initialPosLock = 1;
void controlLoop(){
    float t = tau.read();
    q1.encoder.kmfEstimate();
    // q6.encoder.kmfEstimate();

    // if (initialPosLock){
    //     q6_traj = q6.trajectory.step(q6.at());
    //     initialPosLock = 0;
    // }

    // if (q6.trajectory.reached()){
    //     tau.reset();
    // } else {
    //     q6_traj = q6.trajectory.update(t);
    // }
    // u = q6.update(q6_traj);
}

int main(){
    initializeMsg();
    startThread();
    actuatorSetup();

    
    tau.start();
    enct.attach(&controlLoop, CONTROLLER_SAMPLING_T);
    wait(0.2);

    OUTPUT_ENABLE();

    while(1){
        // q6 = (float)1.0f;
        // emuart4.write(69, 11, hw);
        _UART4.printf("%f\t%f\n", q1.encoder.position(), q1.encoder.velocity());
        
        ThisThread::sleep_for(20);
    }
}

void emuart4Parser(){
    int dataGood = 0;
    float olSpeed, increment, travelDuration;
    for(;;){
        dataGood = emuart4.parse();
        if (dataGood == 1){
            switch (emuart4.command){
                case 147:
                    ONBOARD_LEDS[2] = emuart4.data[0];
                    break;
                case 148:
                    ONBOARD_LEDS[3] = emuart4.data[0];
                    break;
                case 41:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    q1 = olSpeed;
                    break;
                case 42:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    q2 = olSpeed;
                    break;
                case 43:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    q3 = olSpeed;
                    break;
                case 44:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    q4 = olSpeed;
                    break;
                case 45:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    q5 = olSpeed;
                    break;
                case 46:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    q6 = olSpeed;
                    break;
                case 56:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q6.trajectory.setGoal(q6.at(), q6.at()+increment, 0, 0, travelDuration);
                    break;
            }
        } else if (dataGood == -1){
            emuart4.write(0x10);
        }
    }
}