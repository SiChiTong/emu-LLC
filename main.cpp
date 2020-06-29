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
Actuator q1(Y15, Y10, Y9, _RS485_1, 0x54, FLOW_CH1, 0.5f, 0.0, 0.0);
Actuator q2(Y1, Y7, Y5, _RS485_2, 0x3C, FLOW_CH2);
Actuator q3(Y17, Y11, Y23, _RS485_2, 0x3C, FLOW_CH2);
Actuator q4(Y3, Y0, Y2, _RS485_2, 0x3C, FLOW_CH2, 4.0f, 0.0001f, 0.0f);
Actuator q5(Y16, Y6, Y4, _RS485_2, 0x3C, FLOW_CH2, 4.0f, 0.0001f, 0.0f);
Actuator q6(Y14, Y18, Y21, _RS485_2, 0x3C, FLOW_CH2, 4.0f, 0.0001f, 0.0f);
DigitalIn emergency_pin(PE_12);
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
    q1.setPconSat(-3, 3);
    q1.stepper.setDir(-1);
    q1.stepper.setRatio(30.0);
    q1.stepper.setMicro(32);
    q1.encoder.setKdt(CONTROLLER_SAMPLING_T);
    q1.encoder.kmfInit();
    q1.stepper.enable();

    q2.stepper.setRatio(40.0);
    q1.stepper.setDir(-1);
    q2.stepper.setMicro(32);
    q2.stepper.disable();

    q6.setPconSat(-4.5, 4.5);
    q6.stepper.setRatio(5.18);
    q6.stepper.setMicro(32);
    q6.encoder.setKdt(CONTROLLER_SAMPLING_T);
    // q6.encoder.kmfInit();
    q6.stepper.disable();
}


float u;
Timer tau;
float q1_traj;
bool initialPosLock = 1;
void controlLoop(){
    float t = tau.read();
    // float q1_traj;
    q1.encoder.kmfEstimate();
    // q6.encoder.kmfEstimate();

    if (initialPosLock){
        // q6_traj = q6.trajectory.step(q6.at());
        q1_traj = q1.trajectory.step(q1.at());
        initialPosLock = 0;
    }

    if (q1.trajectory.reached()){
        tau.reset();
    } else {
        q1_traj = q1.trajectory.update(t);
    }
    u = q1.update(q1_traj);
}

int main(){
    initializeMsg();
    startThread();
    OUTPUT_ENABLE();
    actuatorSetup();

    
    tau.start();
    enct.attach(&controlLoop, CONTROLLER_SAMPLING_T);
    wait(0.2);

    

    while(1){
        // q6 = (float)1.0f;
        // emuart4.write(69, 11, hw);
        // _UART4.printf("%f\t%f\t%f\n", q1.encoder.position(), q1.encoder.velocity(), q1_traj);
        
        // ThisThread::sleep_for(20);
    }
}

void emuart4Parser(){
    int dataGood = 0;
    float olSpeed, increment, travelDuration;
    for(;;){
        dataGood = emuart4.parse();
        if (dataGood == 1){
            if (emuart4.command == 0x0C || emuart4.command == 0x0D){
                int32_t q1_pos = get_float_bits(q1.at());
                int32_t q1_vel = get_float_bits(q1.encoder.velocity());
                uint8_t simpleJs[26] = {q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF, 
                                        q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF,
                                        q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF,
                                        q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF,
                                        q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF, 
                                        q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF,
                                        0xFF, 0xFE};
                uint8_t fullJs[50] = {q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0,
                                        q1_vel>>24, (q1_vel>>16)&0xFF, (q1_vel>>8)&0xFF, q1_vel&0xFF,
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0, 
                                        0, 0, 0, 0,
                                        0xFF, 0xFE};
                switch (emuart4.command){
                    case 0x0C:
                        wait(0.0005);
                        emuart4.write(0x0A, 26, simpleJs);
                        break;
                    case 0x0D:
                        wait(0.0005);
                        emuart4.write(0x0B, 50, fullJs);
                        break;
                }
            }

            switch (emuart4.command){
                case 31:
                    if (emuart4.data[0])
                        q1.stepper.enable();
                    else
                        q1.stepper.disable();
                    break;
                case 32:
                    if (emuart4.data[0])
                        q2.stepper.enable();
                    else
                        q2.stepper.disable();
                    break;
                case 33:
                    if (emuart4.data[0])
                        q3.stepper.enable();
                    else
                        q3.stepper.disable();
                    break;
                case 34:
                    if (emuart4.data[0])
                        q4.stepper.enable();
                    else
                        q4.stepper.disable();
                    break;
                case 35:
                    if (emuart4.data[0])
                        q5.stepper.enable();
                    else
                        q5.stepper.disable();
                    break;
                case 36:
                    if (emuart4.data[0])
                        q6.stepper.enable();
                    else
                        q6.stepper.disable();
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
                case 51:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at()+increment, 0, 0, travelDuration);
                    _UART4.printf("%f\t%f\n", increment, travelDuration);
                    break;
                case 56:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q6.trajectory.setGoal(q6.at(), q6.at()+increment, 0, 0, travelDuration);
                    break;
                case 147:
                    ONBOARD_LEDS[2] = emuart4.data[0];
                    break;
                case 148:
                    ONBOARD_LEDS[3] = emuart4.data[0];
                    break;
            }
        } else if (dataGood == -1){
            emuart4.write(0x10);
        }
    }
}