#include "INA.h"
INAC INA;


INA219 INA219_Main(0x40);

INA219 INA219_DR(0x41);

void INAC::init(){
    INA219_Main.begin();
    INA219_DR.begin();

    INA219_Main.setMaxCurrentShunt(12);
    INA219_DR.setMaxCurrentShunt(7);
}

float INAC::Voltage_DR(){
    return INA219_DR.getBusVoltage();
}

float INAC::Voltage(){
    return INA219_Main.getBusVoltage();
}

float INAC::Current(){
    return INA219_Main.getCurrent_mA();
}


float INAC::Current_DR(){
    filter[0] = filter[1];
    filter[1] = filter[2];
    filter[2] = filter[3];
    filter[3] = filter[4];
    filter[4] = filter[5];
    filter[5] = INA219_DR.getCurrent_mA();

    int Summe = 0;
    for(int i = 0; i<6 ; i++){Summe+=filter[i];}
    Summe = Summe/6;
    return Summe;
}
