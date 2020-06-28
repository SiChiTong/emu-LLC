#ifndef emuPeripheral_h
#define emuPeripheral_h
#include "mbed.h"
#include "emulatorPin.h"
#include "emulatorLib.h"
#include "SWO.h"

#define PC_BAUDRATE         1000000
#define AMT21_BAUDRATE      115200
#define EEPROM_SIZE         16384
#define EEPROM_ADDRESS      0

AnalogIn    INT_TEMP(ADC_TEMP);
EEPROM      MEMORY(I2C4_SDA, I2C4_SCL, EEPROM_ADDRESS, EEPROM_SIZE);
BusOut      ONBOARD_LEDS(LD1, LD2, LD3, LD4);

RawSerial   _UART4(UART4_TX, UART4_RX, PC_BAUDRATE);
RawSerial   _RS485_1(UART5_TX, UART5_RX, AMT21_BAUDRATE);
RawSerial   _RS485_2(UART2_TX, UART2_RX, AMT21_BAUDRATE);
SWO_Channel SWO("channel");

DigitalOut  OUTPUT_ENABLE_1(OE1);
DigitalOut  OUTPUT_ENABLE_2(OE2);
DigitalOut  OUTPUT_ENABLE_3(OE3);

void OUTPUT_DISABLE(){
    OUTPUT_ENABLE_1 = 0;
    OUTPUT_ENABLE_2 = 0;
    OUTPUT_ENABLE_3 = 0;
}

void OUTPUT_ENABLE(){
    OUTPUT_ENABLE_1 = 1;
    OUTPUT_ENABLE_2 = 1;
    OUTPUT_ENABLE_3 = 1;
}

void SPLASH(){
    ONBOARD_LEDS = 0b0001;
    ThisThread::sleep_for(75);
    for (int i = 0; i <= 2; i++){
        ONBOARD_LEDS = ONBOARD_LEDS << 1;
        ThisThread::sleep_for(75);
    }
    for (int i = 0; i <= 2; i++){
        ONBOARD_LEDS = ONBOARD_LEDS >> 1;
        ThisThread::sleep_for(75);
    }
    ONBOARD_LEDS = 0b0000;
}

int GET_SYSCLK(){
    return SystemCoreClock;
}

float GET_CORETEMP(){
    return INT_TEMP.read()*100.0f;
}

#endif