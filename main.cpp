#include "mbed.h"
#include "platform/mbed_thread.h"
#include "emulatorPin.h"
#include "emulatorLib.h"
#include "emutil.h"
#include "emuPeripheral.h"
#include <cmath>
#include <deque>

#define CONTROLLER_SAMPLING_T   0.01f
#define JOINTSTATE_SAMPLING_T   0.2f
#define PI                      3.14159265359
#define EMUART_BUFFER_SIZE      2048

Emuart emuart4(_UART4, EMUART_BUFFER_SIZE, 0.05f);
Actuator q1(Y15, Y10, Y9, -1, //13
            _RS485_1, 0x1C, FLOW_CH1, 
            2.0f, 0.0, 0.0);
Actuator q2(Y1, Y7, Y3, -1, //3
            _RS485_1, 0x2C, FLOW_CH1,
            1.5f, 0.0, 0.0);
Actuator q3(PB_1_ALT1, Y8, Y22, 1, //8
            _RS485_1, 0x3C, FLOW_CH1,
            2.2f, 0.0, 0.0);
Actuator q4(Y6 , Y0, Y2, 2, //4
            _RS485_2, 0x4C, FLOW_CH2, 
            1.4f, 0.00f, 0.0f);
Actuator diffA(Y16, Y12, Y5, 2, //1
            _RS485_2, 0xA0, FLOW_CH2, 
            1.5f, 0.00f, 0.0f);
Actuator diffB(Y14, Y18, Y21, 2, //14
            _RS485_2, 0xB0, FLOW_CH2, 
            1.5f, 0.00f, 0.0f);
DigitalIn EMS(PE_10);
DigitalOut gripper_flow(FLOW_CH2);

//State
bool m1, m2, m3, m4, mA, mB;
bool g_on, od1_on, od2_on, od3_on, od4_on, od5_on, od6_on, od7_on, od8_on;
//Control
Timer tau;
float q1_traj, q2_traj, q3_traj, q4_traj, a_traj, b_traj;
float q5_lat, q6_lat, q5_lvat, q6_lvat;
bool initialPosLock = 1;

Thread idleThread(osPriorityNormal);
Thread rxMessageParser(osPriorityNormal2);
Ticker enct, js_pub;

void emuart4Parser();
void jointStates();
void grip();
void release();

void idleRoutine(){
    for (;;){
        ONBOARD_LEDS = (ONBOARD_LEDS | 0x1) & 0xD;
        od8 = 1;
        od8_on = 1;
        ThisThread::sleep_for(500);
        ONBOARD_LEDS = (ONBOARD_LEDS | 0x2) & 0xE;
        od8 = 0;
        od8_on = 0;
        ThisThread::sleep_for(500);
    }
}

void initializeMsg(){
    SPLASH();
    OUTPUT_DISABLE();
    _UART4.printf("EMULATOR BOARD v2.8 28-7-2020 C.Thanapong\r\n");
    _UART4.printf("SystemCore Clock:  %ld Hz\r\n", GET_SYSCLK());
    _UART4.printf("InternalTemp:  %.2f C\r\n", GET_CORETEMP());
    _UART4.printf("+++++++++++++++++++++++++++++++\r\n");
}

void startThread(){
    emuart4.init();
    idleThread.start(idleRoutine);
    rxMessageParser.start(emuart4Parser);
}

void actuatorSetup(){
    q1.setPconSat(-3, 3);
    q1.stepper.setRatio(30.0);
    q1.stepper.setMicro(32);
    q1.encoder.setKdt(CONTROLLER_SAMPLING_T);
    q1.encoder.kmfInit();
    q1.stepper.enable();
    m1 = 1;
    wait(0.1);

    q2.setPconSat(-3, 3);
    q2.stepper.setRatio(40.0);
    q2.stepper.setMicro(32);
    q2.encoder.setKdt(CONTROLLER_SAMPLING_T);
    q2.encoder.kmfInit();
    q2.stepper.enable();
    m2 = 1;
    wait(0.1);

    q3.setPconSat(-3, 3);
    q3.stepper.setRatio(30.0);
    q3.stepper.setMicro(32);
    q3.encoder.setKdt(CONTROLLER_SAMPLING_T);
    q3.encoder.kmfInit();
    q3.stepper.enable();
    m3 = 1;
    wait(0.1);

    q4.setPconSat(-1, 1);
    q4.stepper.setRatio(55.222222);
    q4.stepper.setMicro(32);
    q4.encoder.setKdt(CONTROLLER_SAMPLING_T);
    q4.encoder.kmfInit();
    q4.encoder.setRatio(0.98611111111);
    q4.stepper.enable();
    m4 = 1;
    wait(0.1);

    diffA.setPconSat(-1.5, 1.5);
    diffA.stepper.setRatio(14);
    diffA.stepper.setMicro(32);
    diffA.encoder.setKdt(CONTROLLER_SAMPLING_T);
    diffA.encoder.kmfInit();
    diffA.stepper.enable();
    mA = 1;
    wait(0.1);

    diffB.setPconSat(-1.5, 1.5);
    diffB.stepper.setRatio(14);
    diffB.stepper.setMicro(32);
    diffB.encoder.setKdt(CONTROLLER_SAMPLING_T);
    diffB.encoder.kmfInit();
    diffB.stepper.enable();
    mB = 1;
    wait(0.1);

}

void controlLoop(){
    od1 = od1_on; od2 = od2_on; od3 = od3_on; od4 = od4_on; 
    od5 = od5_on; od6 = od6_on; od7 = od7_on; od8 = od8_on;
    wait_us(500);
    if (g_on){grip();}
    else {release();}
    wait_us(1000);
    q1.encoder.kmfEstimate();
    q4.encoder.kmfEstimate();
    wait_us(1000);
    q2.encoder.kmfEstimate();
    diffA.encoder.kmfEstimate();
    wait_us(1000);
    q3.encoder.kmfEstimate();
    diffB.encoder.kmfEstimate();

    q5_lat = (diffB.at() - diffA.at())/2.0f;
    q6_lat = q4.at()+(-diffB.at() - diffA.at())/2.0f;
    q5_lvat = (diffB.vat() - diffA.vat())/2.0f;
    q6_lvat = q4.vat()+(-diffB.vat() - diffA.vat())/2.0f;

    if (EMS.read() == 1){
        float t = tau.read();
        float v1_traj = 0;
        float v2_traj = 0;
        float v3_traj = 0;
        float v4_traj = 0;
        float vA_traj = 0;
        float vB_traj = 0;

        if (initialPosLock){
            q1_traj = q1.trajectory.step(q1.at());
            q2_traj = q2.trajectory.step(q2.at());
            q3_traj = q3.trajectory.step(q3.at());
            q4_traj = q4.trajectory.step(q4.at());
            a_traj = diffA.trajectory.step(diffA.at());
            b_traj = diffB.trajectory.step(diffB.at());
            initialPosLock = 0;
        }

        if (q1.trajectory.reached()){
            tau.reset();
            v1_traj = 0; v2_traj = 0; v3_traj = 0; v4_traj = 0; vA_traj = 0; vB_traj = 0;
            q1_traj = q1.trajectory.getQf();
            q2_traj = q2.trajectory.getQf();
            q3_traj = q3.trajectory.getQf();
            q4_traj = q4.trajectory.getQf();
            a_traj = diffA.trajectory.getQf();
            b_traj = diffB.trajectory.getQf();
        } else {
            q1_traj = q1.trajectory.getPosTraj(t);
            v1_traj = q1.trajectory.getVelTraj(t);
            q2_traj = q2.trajectory.getPosTraj(t);
            v2_traj = q2.trajectory.getVelTraj(t);
            q3_traj = q3.trajectory.getPosTraj(t);
            v3_traj = q3.trajectory.getVelTraj(t);
            q4_traj = q4.trajectory.getPosTraj(t);
            v4_traj = q4.trajectory.getVelTraj(t);
            a_traj = diffA.trajectory.getPosTraj(t);
            vA_traj = diffA.trajectory.getVelTraj(t);
            b_traj = diffB.trajectory.getPosTraj(t);
            vB_traj = diffB.trajectory.getVelTraj(t);
        }

        q1.update(q1_traj, v1_traj, q1.at(), 1);
        q2.update(q2_traj, v2_traj, q2.at(), 1);
        q3.update(q3_traj, v3_traj, q3.at(), 1);
        q4.update(q4_traj, v4_traj*0.8f, q4.at(), 1);
        diffA.update(a_traj, vA_traj, diffA.at(), 1);
        diffB.update(b_traj, vB_traj, diffB.at(), 1);
    } else {
        q1 = 0;
        q2 = 0;
        q3 = 0;
        q4 = 0;
        diffA = 0;
        diffB = 0;
        initialPosLock = 1;
    }
}

int main(){
    initializeMsg();
    startThread();
    OUTPUT_ENABLE();
    actuatorSetup();
    
    tau.start();
    enct.attach(&controlLoop, CONTROLLER_SAMPLING_T);
    wait(0.1);
    js_pub.attach(&jointStates, JOINTSTATE_SAMPLING_T);

    while(1){}
}

void emuart4Parser(){
    int dataGood = 0;
    float olSpeed, increment, travelDuration, goal;
    float i1, i2, i3, i4, i5, i6;
    uint8_t num_viapoints;
    std::deque <float> points[13]; //vps1, vps2, vps3, vps4, vps5, vps6, vvps1, vvps2, vvps3, vvps4, vvps5, vvps6, tvps;
    for(;;){
        dataGood = emuart4.parse();
        if (dataGood == 1){
            switch (emuart4.command){
                case 21:
                    od1_on = emuart4.data[0];
                    break;
                case 22:
                    od2_on = emuart4.data[0];
                    break;
                case 23:
                    od3_on = emuart4.data[0];
                    break;
                case 24:
                    od4_on = emuart4.data[0];
                    break;
                case 25:
                    od5_on = emuart4.data[0];
                    break;
                case 26:
                    od6_on = emuart4.data[0];
                    break;
                case 27:
                    od7_on = emuart4.data[0];
                    break;
                case 28:
                    od8_on = emuart4.data[0];
                    break;
                case 31:
                    m1 = emuart4.data[0];
                    if (m1)
                        q1.stepper.enable();
                    else
                        q1.stepper.disable();
                    break;
                case 32:
                    m2 = emuart4.data[0];
                    if (m2)
                        q2.stepper.enable();
                    else
                        q2.stepper.disable();
                    break;
                case 33:
                    m3 = emuart4.data[0];
                    if (m3)
                        q3.stepper.enable();
                    else
                        q3.stepper.disable();
                    break;
                case 34:
                    m4 = emuart4.data[0];
                    if (m4)
                        q4.stepper.enable();
                    else
                        q4.stepper.disable();
                    break;
                case 35:
                    mA = emuart4.data[0];
                    if (mA)
                        diffA.stepper.enable();
                    else
                        diffA.stepper.disable();
                    break;
                case 36:
                    mB = emuart4.data[0];
                    if (mB)
                        diffB.stepper.enable();
                    else
                        diffB.stepper.disable();
                    break;
                case 41:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    if (EMS.read()) q1 = olSpeed;
                    break;
                case 42:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    if (EMS.read()) q2 = olSpeed;
                    break;
                case 43:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    if (EMS.read()) q3 = olSpeed;
                    break;
                case 44:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    if (EMS.read())  q4 = olSpeed;
                    break;
                case 45:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    if (EMS.read()) diffA = olSpeed;
                    break;
                case 46:
                    olSpeed = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    if (EMS.read()) diffA = olSpeed;
                    break;
                case 51:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at()+increment, 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at(), 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at(), 0, 0, travelDuration);
                    break;
                case 52:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at()+increment, 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at(), 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at(), 0, 0, travelDuration);
                    break;
                case 53:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at()+increment, 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at(), 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at(), 0, 0, travelDuration);
                    break;
                case 54:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at()+increment, 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at()+increment, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at()+increment, 0, 0, travelDuration);
                    break;
                case 55:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at()-increment, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at()+increment, 0, 0, travelDuration);
                    break;
                case 56:
                    increment = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at()-increment, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at()-increment, 0, 0, travelDuration);
                    break;
                case 61:
                    goal = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), goal, 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at(), 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at(), 0, 0, travelDuration);
                    break;
                case 62:
                    goal = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), goal, 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at(), 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at(), 0, 0, travelDuration);
                    break;
                case 63:
                    goal = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), goal, 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at(), 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at(), 0, 0, travelDuration);
                    break;
                case 64:
                    goal = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), goal, 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), goal, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), goal, 0, 0, travelDuration);
                    break;
                case 65:
                    goal = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), -goal, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), goal, 0, 0, travelDuration);
                    break;
                case 66:
                    goal = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    travelDuration = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    q1.trajectory.setGoal(q1.at(), q1.at(), 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at(), 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at(), 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at(), 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), -goal, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), -goal, 0, 0, travelDuration);
                    break;
                case 71:
                    i1 = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    i2 = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    i3 = unpack754_32((emuart4.data[8]<<24)|(emuart4.data[9]<<16)|(emuart4.data[10]<<8)|emuart4.data[11]);
                    i4 = unpack754_32((emuart4.data[12]<<24)|(emuart4.data[13]<<16)|(emuart4.data[14]<<8)|emuart4.data[15]);
                    i5 = unpack754_32((emuart4.data[16]<<24)|(emuart4.data[17]<<16)|(emuart4.data[18]<<8)|emuart4.data[19]);
                    i6 = unpack754_32((emuart4.data[20]<<24)|(emuart4.data[21]<<16)|(emuart4.data[22]<<8)|emuart4.data[23]);
                    travelDuration = unpack754_32((emuart4.data[24]<<24)|(emuart4.data[25]<<16)|(emuart4.data[26]<<8)|emuart4.data[27]);
                    q1.trajectory.setGoal(q1.at(), q1.at()+i1, 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), q2.at()+i2, 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), q3.at()+i3, 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), q4.at()+i4, 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), diffA.at()+i4-i5-i6, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), diffB.at()+i4+i5-i6, 0, 0, travelDuration);
                    break;
                case 81:
                    i1 = unpack754_32((emuart4.data[0]<<24)|(emuart4.data[1]<<16)|(emuart4.data[2]<<8)|emuart4.data[3]);
                    i2 = unpack754_32((emuart4.data[4]<<24)|(emuart4.data[5]<<16)|(emuart4.data[6]<<8)|emuart4.data[7]);
                    i3 = unpack754_32((emuart4.data[8]<<24)|(emuart4.data[9]<<16)|(emuart4.data[10]<<8)|emuart4.data[11]);
                    i4 = unpack754_32((emuart4.data[12]<<24)|(emuart4.data[13]<<16)|(emuart4.data[14]<<8)|emuart4.data[15]);
                    i5 = unpack754_32((emuart4.data[16]<<24)|(emuart4.data[17]<<16)|(emuart4.data[18]<<8)|emuart4.data[19]);
                    i6 = unpack754_32((emuart4.data[20]<<24)|(emuart4.data[21]<<16)|(emuart4.data[22]<<8)|emuart4.data[23]);
                    travelDuration = unpack754_32((emuart4.data[24]<<24)|(emuart4.data[25]<<16)|(emuart4.data[26]<<8)|emuart4.data[27]);
                    q1.trajectory.setGoal(q1.at(), i1, 0, 0, travelDuration);
                    q2.trajectory.setGoal(q2.at(), i2, 0, 0, travelDuration);
                    q3.trajectory.setGoal(q3.at(), i3, 0, 0, travelDuration);
                    q4.trajectory.setGoal(q4.at(), i4, 0, 0, travelDuration);
                    diffA.trajectory.setGoal(diffA.at(), i4-i5-i6, 0, 0, travelDuration);
                    diffB.trajectory.setGoal(diffB.at(), i4+i5-i6, 0, 0, travelDuration);
                    break;
                case 82:
                    num_viapoints = (emuart4.dataLen)/52;
                    for (int i = 0; i < 13; i++){
                        points[i].clear();
                        for (int j = 0; j < num_viapoints; j++)
                            points[i].push_back(unpack754_32((emuart4.data[(i*4*num_viapoints)+(4*j)+0]<<24)|(emuart4.data[(i*4*num_viapoints)+(4*j)+1]<<16)|(emuart4.data[(i*4*num_viapoints)+(4*j)+2]<<8)|emuart4.data[(i*4*num_viapoints)+(4*j)+3]));
                    }
                    
                    points[0].push_front(q1.at());
                    points[1].push_front(q2.at());
                    points[2].push_front(q3.at());
                    points[3].push_front(q4.at());
                    points[4].push_front(diffA.at());
                    points[5].push_front(diffB.at());
                    points[6].push_front(0);
                    points[7].push_front(0);
                    points[8].push_front(0);
                    points[9].push_front(0);
                    points[10].push_front(0);
                    points[11].push_front(0);
                    
                    q1.trajectory.setViaPoints(points[0], points[6], points[12]);
                    q2.trajectory.setViaPoints(points[1], points[7], points[12]);
                    q3.trajectory.setViaPoints(points[2], points[8], points[12]);
                    q4.trajectory.setViaPoints(points[3], points[9], points[12]);
                    diffA.trajectory.setViaPoints(points[4], points[10], points[12]);
                    diffB.trajectory.setViaPoints(points[5], points[11], points[12]);
                    break;
                case 147:
                    ONBOARD_LEDS[2] = emuart4.data[0];
                    break;
                case 148:
                    ONBOARD_LEDS[3] = emuart4.data[0];
                    break;
                case 149:
                    g_on = emuart4.data[0];
                    break;
            }
        } else if (dataGood == -1){
            // emuart4.write(0x10);
        }
    }
}

void jointStates(){
    uint8_t status1 = 0;
    uint8_t status2 = 0;
    status1 = (m1<<7) | (m2<<6) | (m3<<5) | (m4<<4) | (mA<<3) | (mB<<2) | (g_on<<1);
    status2 = (od1_on<<7) | (od2_on<<6) | (od3_on<<5) | (od4_on<<4) | (od5_on<<3) | (od6_on<<2) | (od7_on<<1) | od8_on;
    int32_t q1_pos = get_float_bits(q1.at());
    int32_t q2_pos = get_float_bits(q2.at());
    int32_t q3_pos = get_float_bits(q3.at());
    int32_t q4_pos = get_float_bits(q4.at());
    int32_t q5_pos = get_float_bits(q5_lat);
    int32_t q6_pos = get_float_bits(q6_lat);
    int32_t q1_vel = get_float_bits(q1.vat());
    int32_t q2_vel = get_float_bits(q2.vat());
    int32_t q3_vel = get_float_bits(q3.vat());
    int32_t q4_vel = get_float_bits(q4.vat());
    int32_t q5_vel = get_float_bits(q5_lvat);
    int32_t q6_vel = get_float_bits(q6_lvat);
    uint8_t simpleJs[26] = {q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF, 
                            q2_pos>>24, (q2_pos>>16)&0xFF, (q2_pos>>8)&0xFF, q2_pos&0xFF,
                            q3_pos>>24, (q3_pos>>16)&0xFF, (q3_pos>>8)&0xFF, q3_pos&0xFF,
                            q4_pos>>24, (q4_pos>>16)&0xFF, (q4_pos>>8)&0xFF, q4_pos&0xFF,
                            q5_pos>>24, (q5_pos>>16)&0xFF, (q5_pos>>8)&0xFF, q5_pos&0xFF, 
                            q6_pos>>24, (q6_pos>>16)&0xFF, (q6_pos>>8)&0xFF, q6_pos&0xFF,
                            status1, status2};
    uint8_t fullJs[50] = {q1_pos>>24, (q1_pos>>16)&0xFF, (q1_pos>>8)&0xFF, q1_pos&0xFF, 
                            q2_pos>>24, (q2_pos>>16)&0xFF, (q2_pos>>8)&0xFF, q2_pos&0xFF, 
                            q3_pos>>24, (q3_pos>>16)&0xFF, (q3_pos>>8)&0xFF, q3_pos&0xFF, 
                            q4_pos>>24, (q4_pos>>16)&0xFF, (q4_pos>>8)&0xFF, q4_pos&0xFF,
                            q5_pos>>24, (q5_pos>>16)&0xFF, (q5_pos>>8)&0xFF, q5_pos&0xFF, 
                            q6_pos>>24, (q6_pos>>16)&0xFF, (q6_pos>>8)&0xFF, q6_pos&0xFF,
                            q1_vel>>24, (q1_vel>>16)&0xFF, (q1_vel>>8)&0xFF, q1_vel&0xFF,
                            q2_vel>>24, (q2_vel>>16)&0xFF, (q2_vel>>8)&0xFF, q2_vel&0xFF, 
                            q3_vel>>24, (q3_vel>>16)&0xFF, (q3_vel>>8)&0xFF, q3_vel&0xFF, 
                            q4_vel>>24, (q4_vel>>16)&0xFF, (q4_vel>>8)&0xFF, q4_vel&0xFF,
                            q5_vel>>24, (q5_vel>>16)&0xFF, (q5_vel>>8)&0xFF, q5_vel&0xFF, 
                            q6_vel>>24, (q6_vel>>16)&0xFF, (q6_vel>>8)&0xFF, q6_vel&0xFF,
                            status1, status2};
    emuart4.write(0x0A, 26, simpleJs);
}

void grip(){
    gripper_flow = 1;
    _RS485_2.putc(0x70);
    wait_us(75);
    gripper_flow = 0;
}

void release(){
    gripper_flow = 1;
    _RS485_2.putc(0x74);
    wait_us(75);
    gripper_flow = 0;
}