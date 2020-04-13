#include "mbed.h"
#include "platform/mbed_thread.h"
#include "emulatorPin.h"
#include "emulatorLib.h"
#include "emuPeripheral.h"
#include <Eigen/Dense.h>
#include <cmath>

#define PI              3.14159265359
#define BUFFER_SIZE     1024

Emuart emuart4(_UART4, BUFFER_SIZE);
// Actuator q2(Y1, Y7, Y5, enc2, 0x3C, FLOW_CH2);
// // Actuator q4(Y12, Y10, Y4, enc2, 0x3C, FLOW_CH2);
// // Actuator q5(Y15, Y11, Y19, enc2, 0x3C, FLOW_CH2);//Y13
// Actuator q6(Y14, Y18, Y21, enc2, 0x3C, FLOW_CH2);
// //can't use Y12,Y13

Thread idleThread(osPriorityLow);
Thread errorHandle(osPriorityHigh);
Thread rxMessageParser(osPriorityNormal);
Ticker enct;

void idleRoutine(){
    for (;;){
        ONBOARD_LEDS = (ONBOARD_LEDS | 0x1) & 0xD;
        ThisThread::sleep_for(500);
        ONBOARD_LEDS = (ONBOARD_LEDS | 0x2) & 0xE;
        ThisThread::sleep_for(500);
    }
}

void emuart4Parser(){
    int dataGood = 0;
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
            }
        } else if (dataGood == -1){
            _UART4.printf("\nERROR!\n");
            // q6 = m2*63;
            // q2 = m3*220;
        }
    }
}

void initializeMsg(){
    SPLASH();
    OUTPUT_DISABLE();
    _UART4.printf("InternalTemp: %.2f C\n", INT_TEMP.read()*100);
    _UART4.printf("SystemCoreClock: %ld Hz\n", SystemCoreClock);
}

void startThread(){
    emuart4.init();
    idleThread.start(idleRoutine);
    rxMessageParser.start(emuart4Parser);
}

void kal_test(){
    // q6.encoder.kmfEstimate();
}

int main(){
    initializeMsg();
    startThread();

    //small 1000-2000 8mstp Hz,large 3500 Hz 16mstp
    // q6.stepper.enable();
    // q6.stepper.setMaxFrequency(2000.0f*8.0f);
    // q2.stepper.setMaxFrequency(4000.0f*16.0f);
    // q6.encoder.kmfInit();
    // enct.attach(&kal_test, q6.encoder.getKdt());

    OUTPUT_ENABLE();

    uint8_t a[7] = {'a', 'b', 'c', 5, 8, 13, 21};
    while(1){
        
        // uart4.printf("%.2f,%.4f\n",q6.encoder.position(), q6.encoder.velocity());
        emuart4.write(69, 7, a);
        // uart4.printf("%d\t%d\t%d\t%d\t%d\t%d\n",sizeof(a)/sizeof(a[0]), emuart4.command,emuart4.data[0],emuart4.data[1],emuart4.data[2], emuart4.data[emuart4.dataLen-1]);
        ThisThread::sleep_for(200);
    }
}