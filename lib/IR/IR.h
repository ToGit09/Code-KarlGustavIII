#pragma once
#include "IR_ring.h"

class IRC {
    private:

    public:
        float Distance_raw;
        float NullCall  = 100;
        float Angle;
        float Distance;
        int IR_Values[16] ;
        float TSSP;
        bool TSSPs[8];

        void read(bool readRawData);
        int GetData(int Port);
        int GetTSSP();
        int GetTSSP(bool (&tssp)[8]);
        void init();
        void writeGains(uint16_t (&gains)[16]);
        void writeOffsets(int16_t (&offsets)[16]);

};

extern IRC IRh;