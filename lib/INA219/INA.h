#pragma once
#include "INA219.h"


class INAC{
    private:
        int Summe;
        int filter[6];
    public:
        void init();
        float Voltage();
        float Voltage_DR();
        float Current();
        float Current_DR();
};

extern INAC INA;